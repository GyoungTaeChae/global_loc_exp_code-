#pragma once

// FPFH + RANSAC global localization, rewritten for the polish experiment.
//
//   scan downsample -> FPFH -> feature match -> RANSAC (polygon prerejection +
//   SVD) -> top-K candidates -> polish each candidate -> occupancy-voxel inlier
//   rescore -> best
//
// The stages before polish live in Localizer; polish is a separate PolishEngine
// so one candidate set can be handed to several polish methods and compared on
// identical input. See PIPELINE.md for the per-stage contract.

#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_slam/types.hpp"

namespace glexp
{

using Cloud = ::pcl::PointCloud<PointType>;

struct Candidate {
  double error = std::numeric_limits<double>::max(); // mean squared inlier residual
  double inlier_fraction = 0.0;
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(); // scan -> map
};

struct LocalizerParams {
  int num_threads = 8;
  double query_voxel_size = 0.5; // scan AND map downsample for feature extraction
  double normal_estimation_radius = 2.0;
  double fpfh_search_radius = 4.0;
  int ransac_max_iterations = 1000000; // candidate correspondence draws
  int ransac_matching_budget = 10000; // full inlier evaluations allowed
  int correspondence_randomness = 2; // k feature-nearest map points per scan point
  double similarity_threshold = 0.5; // polygon edge-length prerejection
  double max_correspondence_distance = 1.0; // inlier voxel resolution [m]
  double min_inlier_fraction = 0.25; // candidate must clear this to be kept

  // Top-K stage. Raw RANSAC hits cluster tightly around a few poses, so
  // candidates closer than these thresholds to an already-kept one are merged
  // into it instead of taking a slot.
  int top_k = 10;
  double suppress_translation = 2.0; // [m]
  double suppress_rotation_degrees = 10.0;
};

struct LocalizeStats {
  int query_points = 0;
  int evaluated = 0; // transforms scored against the occupancy voxels
  int passed = 0; // of those, how many cleared min_inlier_fraction
  int distinct_candidates = 0; // survivors of the top-K suppression
  double feature_ms = 0.0; // scan downsample + FPFH + feature kNN
  double ransac_ms = 0.0;
};

/**
 * Holds the map-side state (features, occupancy voxels) and produces pose
 * candidates for a scan. Everything is const after setMap, so localize() may be
 * called repeatedly on one map.
 */
class Localizer
{
 public:
  explicit Localizer(const LocalizerParams& params): params_(params) {}

  /**
   * Caches the map FPFH descriptors and the occupancy voxel set. Slow; the
   * caller is expected to reuse one Localizer over many scans.
   */
  void setMap(const Cloud::ConstPtr& map);

  const Cloud::ConstPtr& map() const { return map_; }
  std::size_t featureCount() const { return map_features_ ? map_features_->size() : 0; }

  /** Scan downsampled to query_voxel_size; also what polish should be fed. */
  Cloud::ConstPtr downsample(const Cloud::ConstPtr& scan) const;

  /**
   * Runs stages 1-4 on an already downsampled scan.
   * @return candidates ordered best-first, empty when nothing cleared
   *         min_inlier_fraction
   */
  std::vector<Candidate> localize(const Cloud::ConstPtr& query, LocalizeStats* stats) const;

  /** Occupancy-voxel score of one transform; the stage-6 rescore. */
  Candidate score(const Cloud& query, const Eigen::Matrix4f& transformation) const;

 private:
  using FeatureCloud = ::pcl::PointCloud<::pcl::FPFHSignature33>;

  FeatureCloud::ConstPtr extractFpfh(const Cloud::ConstPtr& cloud) const;
  void buildOccupancyVoxels(const Cloud& map);
  Eigen::Vector3i voxelCoord(const Eigen::Vector4f& point) const;
  Eigen::Vector4f voxelCenter(const Eigen::Vector3i& coord) const;

  struct Vector3iHash {
    std::size_t operator()(const Eigen::Vector3i& index) const;
  };

  LocalizerParams params_;
  Cloud::ConstPtr map_; // map at its native resolution, source of the voxel set
  Cloud::ConstPtr map_feature_cloud_; // downsampled map, aligned 1:1 with map_features_
  FeatureCloud::ConstPtr map_features_;
  ::pcl::KdTreeFLANN<::pcl::FPFHSignature33> feature_tree_;
  std::unordered_set<Eigen::Vector3i, Vector3iHash, std::equal_to<Eigen::Vector3i>,
    Eigen::aligned_allocator<Eigen::Vector3i>>
    occupancy_voxels_;
};

enum class PolishMethod
{
  None,
  Icp, // pcl::IterativeClosestPoint, point-to-point
  Gicp, // nano_gicp::NanoGICP
  Ndt // pclomp::NormalDistributionsTransform
};

bool parsePolishMethod(const std::string& name, PolishMethod* method);
const char* polishMethodName(PolishMethod method);

struct PolishParams {
  PolishMethod method = PolishMethod::Icp;
  int num_threads = 8; // GICP and NDT only; PCL's ICP is single threaded
  int max_iterations = 20;
  double max_correspondence_distance = 2.0; // ICP and GICP
  double transformation_epsilon = 1e-3;
  double rotation_epsilon = 2e-3; // GICP only; nano_gicp's own default
  double init_lambda_factor = 1e-9; // GICP only
  int correspondence_randomness = 20; // GICP covariance neighbourhood
  double ndt_resolution = 2.0;
  double ndt_step_size = 0.1;
  double ndt_outlier_ratio = 0.55; // pclomp's own default
  // 0 = polish sees the whole downsampled scan. Otherwise the scan is strided
  // down to at most this many points before polish; scoring still uses the
  // whole scan, so the score stays comparable across settings.
  int polish_points = 0;
  // false = polish aligns against the same map tile the candidates came from.
  // true = against the whole map session, which is what the plugin does: it
  // binds reg_loc_ to globalmap_ once at load time.
  bool full_map_target = false;
  std::string label; // used in the csv; defaults to method + key params
};

std::string defaultPolishLabel(const PolishParams& params);

/**
 * One polish method bound to one map. Target-side setup (kd-tree, covariances,
 * NDT voxel grid) happens in setTarget so it is shared by every candidate and
 * every query on that map.
 */
class PolishEngine
{
 public:
  explicit PolishEngine(const PolishParams& params);
  ~PolishEngine();

  const PolishParams& params() const { return params_; }

  /** Heavy target-side setup: kd-tree, GICP covariances or the NDT voxel grid. */
  void setTarget(const Cloud::ConstPtr& map);

  /** Strides the scan down to params().polish_points, or returns it unchanged. */
  Cloud::ConstPtr thinForPolish(const Cloud::ConstPtr& scan) const;

  /**
   * Refines initial by aligning query to the map. refined always receives the
   * method's final pose.
   * @return the method's convergence flag, reported for the record only
   */
  bool refine(const Cloud::ConstPtr& query, const Eigen::Matrix4f& initial,
    Eigen::Matrix4f* refined);

 private:
  struct Impl;
  PolishParams params_;
  std::unique_ptr<Impl> impl_;
};

} // namespace glexp
