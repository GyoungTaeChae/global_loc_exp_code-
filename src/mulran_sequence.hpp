#pragma once

// Reads one MulRan sequence: Ouster .bin scans and the global_pose.csv ground
// truth. Nothing here writes into the raisin workspace; only its headers are
// read, for PointType and voxelizePcd.
//
// Formats (verified against the files in
// /home/cgt24/work/dataset/rosBagFiles/mulran):
//   sensor_data/Ouster/<timestamp_ns>.bin  packed float32 x, y, z, intensity
//   global_pose.csv                        timestamp_ns, then the 12 entries of
//                                          a row-major 3x4 base pose in UTM-52N

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <dirent.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "lidar_slam/loop_closure.hpp"
#include "lidar_slam/types.hpp"

namespace mulran
{

using Cloud = ::pcl::PointCloud<PointType>;

// Ground truth is ~100Hz; a lidar stamp whose bracketing ground truth samples
// are further apart than this sits in a dropout and is treated as uncovered
// instead of extrapolated.
constexpr double kMaxGroundTruthGapSeconds = 0.2;

// (0,0,0) is the Ouster no-return marker and makes up a large part of a frame.
constexpr double kMinValidRangeMeters = 1e-3;

/**
 * MulRan / Complex Urban vehicle -> Ouster extrinsic. The dataset ships no
 * calibration file; this is the platform convention (yaw 180 deg, pitch
 * -1.5 deg, sensor 1.7 m ahead and 1.8 m above the body origin). Both the map
 * and the query sequence get the same transform, so any residual error is a
 * constant rigid offset shared by both sides of the comparison.
 */
inline Eigen::Matrix4d baseToLidarExtrinsic() {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-1.5 * M_PI / 180., Eigen::Vector3d::UnitY()))
                                  .toRotationMatrix();
  transform.block<3, 1>(0, 3) = Eigen::Vector3d(1.7042, -0.021, 1.8047);
  return transform;
}

struct GroundTruthSample {
  std::uint64_t timestamp_ns = 0;
  Eigen::Matrix4d base_pose = Eigen::Matrix4d::Identity(); // UTM-52N absolute
};

inline std::vector<std::string> listFiles(const std::string& directory) {
  std::vector<std::string> names;
  DIR* handle = opendir(directory.c_str());
  if (handle == nullptr) {
    throw std::runtime_error("cannot open directory: " + directory);
  }
  while (const dirent* entry = readdir(handle)) {
    const std::string name = entry->d_name;
    if (name != "." && name != "..") {
      names.push_back(name);
    }
  }
  closedir(handle);
  std::sort(names.begin(), names.end());
  return names;
}

class Sequence
{
 public:
  Sequence(const std::string& directory, const Eigen::Matrix4d& base_to_lidar,
    double frame_voxel_size, std::size_t cache_capacity):
    directory_(directory),
    base_to_lidar_(base_to_lidar),
    frame_voxel_size_(frame_voxel_size),
    cache_capacity_(cache_capacity) {
    loadScanTimestamps();
    loadGroundTruth();
    resolvePosePerFrame();
  }

  const std::string& directory() const { return directory_; }
  std::size_t size() const { return scan_files_.size(); }
  std::uint64_t timestampNs(std::size_t index) const { return scan_timestamps_[index]; }
  bool hasPose(std::size_t index) const { return pose_valid_[index] != 0; }
  std::size_t poseCount() const {
    return static_cast<std::size_t>(std::count(pose_valid_.begin(), pose_valid_.end(), 1));
  }

  // First ground truth translation. Used as the shared local origin so that
  // point coordinates stay small enough for float32 arithmetic.
  Eigen::Vector3d firstGroundTruthPosition() const {
    if (ground_truth_.empty()) {
      throw std::runtime_error("no ground truth in " + directory_);
    }
    return ground_truth_.front().base_pose.block<3, 1>(0, 3);
  }

  void setOrigin(const Eigen::Vector3d& origin) { origin_ = origin; }
  const Eigen::Vector3d& origin() const { return origin_; }

  // T_local_lidar: sensor frame -> origin-shifted UTM.
  Eigen::Matrix4d lidarPose(std::size_t index) const {
    Eigen::Matrix4d pose = lidar_pose_[index];
    pose.block<3, 1>(0, 3) -= origin_;
    return pose;
  }

  Eigen::Vector3d lidarPosition(std::size_t index) const {
    return lidar_pose_[index].block<3, 1>(0, 3) - origin_;
  }

  // Raw sensor-frame scan with no-returns removed. Not cached: the query path
  // reads each frame once, while the map path uses the voxelized cache below.
  Cloud::Ptr readScan(std::size_t index) const {
    const std::string path = directory_ + "/sensor_data/Ouster/" + scan_files_[index];
    std::ifstream file(path, std::ios::binary);
    if (!file) {
      throw std::runtime_error("cannot open scan: " + path);
    }
    file.seekg(0, std::ios::end);
    const std::streamoff byte_count = file.tellg();
    file.seekg(0, std::ios::beg);
    const std::size_t point_count = static_cast<std::size_t>(byte_count) / (4 * sizeof(float));
    std::vector<float> buffer(point_count * 4);
    file.read(reinterpret_cast<char*>(buffer.data()), point_count * 4 * sizeof(float));

    auto cloud = std::make_shared<Cloud>();
    cloud->reserve(point_count);
    for (std::size_t i = 0; i < point_count; ++i) {
      const float x = buffer[i * 4 + 0];
      const float y = buffer[i * 4 + 1];
      const float z = buffer[i * 4 + 2];
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z <
          kMinValidRangeMeters * kMinValidRangeMeters) {
        continue;
      }
      PointType point;
      point.x = x;
      point.y = y;
      point.z = z;
      point.intensity = buffer[i * 4 + 3];
      point.t = 0;
      cloud->push_back(point);
    }
    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
  }

  // Sensor-frame scan already downsampled to frame_voxel_size, kept in a
  // bounded FIFO cache because neighbouring map tiles reuse most frames.
  Cloud::ConstPtr voxelizedScan(std::size_t index) {
    const auto hit = cache_.find(index);
    if (hit != cache_.end()) {
      return hit->second;
    }
    Cloud::ConstPtr voxelized =
      raisin::voxelizePcd(*readScan(index), static_cast<float>(frame_voxel_size_));
    cache_.emplace(index, voxelized);
    cache_order_.push_back(index);
    while (cache_order_.size() > cache_capacity_) {
      cache_.erase(cache_order_.front());
      cache_order_.pop_front();
    }
    return voxelized;
  }

  std::size_t cachedFrameCount() const { return cache_.size(); }

 private:
  void loadScanTimestamps() {
    // The stamp csv and the .bin listing disagree on some sequences, so the
    // filenames are the authority.
    for (const std::string& name : listFiles(directory_ + "/sensor_data/Ouster")) {
      if (name.size() < 5 || name.compare(name.size() - 4, 4, ".bin") != 0) {
        continue;
      }
      scan_files_.push_back(name);
      scan_timestamps_.push_back(std::stoull(name.substr(0, name.size() - 4)));
    }
    if (scan_files_.empty()) {
      throw std::runtime_error("no Ouster .bin files under " + directory_);
    }
  }

  void loadGroundTruth() {
    const std::string path = directory_ + "/global_pose.csv";
    std::ifstream file(path);
    if (!file) {
      throw std::runtime_error("cannot open " + path);
    }
    std::string line;
    while (std::getline(file, line)) {
      if (line.empty()) {
        continue;
      }
      std::stringstream stream(line);
      std::string field;
      std::vector<std::string> fields;
      while (std::getline(stream, field, ',')) {
        fields.push_back(field);
      }
      if (fields.size() < 13) {
        continue;
      }
      GroundTruthSample sample;
      sample.timestamp_ns = std::stoull(fields[0]);
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 4; ++column) {
          sample.base_pose(row, column) = std::stod(fields[1 + row * 4 + column]);
        }
      }
      ground_truth_.push_back(sample);
    }
    if (ground_truth_.empty()) {
      throw std::runtime_error("empty ground truth: " + path);
    }
  }

  void resolvePosePerFrame() {
    lidar_pose_.assign(scan_files_.size(), Eigen::Matrix4d::Identity());
    pose_valid_.assign(scan_files_.size(), 0);
    for (std::size_t i = 0; i < scan_files_.size(); ++i) {
      Eigen::Matrix4d base_pose;
      if (!interpolateBasePose(scan_timestamps_[i], base_pose)) {
        continue;
      }
      lidar_pose_[i] = base_pose * base_to_lidar_;
      pose_valid_[i] = 1;
    }
  }

  bool interpolateBasePose(std::uint64_t timestamp_ns, Eigen::Matrix4d& pose) const {
    const auto upper = std::lower_bound(ground_truth_.begin(), ground_truth_.end(), timestamp_ns,
      [](const GroundTruthSample& sample, std::uint64_t value) {
        return sample.timestamp_ns < value;
      });
    if (upper == ground_truth_.begin()) {
      if (upper->timestamp_ns == timestamp_ns) {
        pose = upper->base_pose;
        return true;
      }
      return false; // before the first sample
    }
    if (upper == ground_truth_.end()) {
      return false; // after the last sample
    }
    const GroundTruthSample& after = *upper;
    const GroundTruthSample& before = *(upper - 1);
    const double span_seconds =
      static_cast<double>(after.timestamp_ns - before.timestamp_ns) * 1e-9;
    if (span_seconds > kMaxGroundTruthGapSeconds) {
      return false; // ground truth dropout, do not extrapolate across it
    }
    const double ratio = span_seconds > 0.
      ? static_cast<double>(timestamp_ns - before.timestamp_ns) * 1e-9 / span_seconds
      : 0.;

    const Eigen::Quaterniond rotation_before(Eigen::Matrix3d(before.base_pose.block<3, 3>(0, 0)));
    const Eigen::Quaterniond rotation_after(Eigen::Matrix3d(after.base_pose.block<3, 3>(0, 0)));
    pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) =
      rotation_before.normalized().slerp(ratio, rotation_after.normalized()).toRotationMatrix();
    pose.block<3, 1>(0, 3) = before.base_pose.block<3, 1>(0, 3) * (1. - ratio) +
      after.base_pose.block<3, 1>(0, 3) * ratio;
    return true;
  }

  std::string directory_;
  Eigen::Matrix4d base_to_lidar_;
  double frame_voxel_size_;
  std::size_t cache_capacity_;

  std::vector<std::string> scan_files_;
  std::vector<std::uint64_t> scan_timestamps_;
  std::vector<GroundTruthSample> ground_truth_;
  std::vector<Eigen::Matrix4d> lidar_pose_; // UTM absolute, before the origin shift
  std::vector<char> pose_valid_;
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();

  std::unordered_map<std::size_t, Cloud::ConstPtr> cache_;
  std::deque<std::size_t> cache_order_;
};

} // namespace mulran
