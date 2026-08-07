#include "global_localization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <sstream>

#include <omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <nano_gicp/nano_gicp.h>
#include <pclomp/ndt_omp.h>

#include "lidar_slam/loop_closure.hpp"

namespace glexp
{

namespace
{

double elapsedMs(const std::chrono::steady_clock::time_point& start) {
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(now - start).count();
}

double rotationDifferenceDegrees(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  const Eigen::Matrix3f relative = a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
  const float trace = std::min(3.f, std::max(-1.f, relative.trace()));
  return std::acos(std::min(1.f, std::max(-1.f, (trace - 1.f) * 0.5f))) * 180.f / M_PI;
}

bool betterCandidate(const Candidate& a, const Candidate& b) {
  if (a.inlier_fraction != b.inlier_fraction) {
    return a.inlier_fraction > b.inlier_fraction;
  }
  return a.error < b.error;
}

} // namespace

std::size_t Localizer::Vector3iHash::operator()(const Eigen::Vector3i& index) const {
  // boost::hash_combine without the boost dependency
  std::size_t seed = 0;
  for (int i = 0; i < 3; ++i) {
    seed ^= std::hash<int>()(index[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  return seed;
}

Localizer::FeatureCloud::ConstPtr Localizer::extractFpfh(const Cloud::ConstPtr& cloud) const {
  auto normals = std::make_shared<::pcl::PointCloud<::pcl::Normal>>();
  ::pcl::NormalEstimationOMP<PointType, ::pcl::Normal> normal_estimation;
  normal_estimation.setNumberOfThreads(params_.num_threads);
  normal_estimation.setRadiusSearch(params_.normal_estimation_radius);
  normal_estimation.setInputCloud(cloud);
  normal_estimation.compute(*normals);

  auto features = std::make_shared<FeatureCloud>();
  ::pcl::FPFHEstimationOMP<PointType, ::pcl::Normal, ::pcl::FPFHSignature33> fpfh_estimation;
  fpfh_estimation.setNumberOfThreads(params_.num_threads);
  fpfh_estimation.setRadiusSearch(params_.fpfh_search_radius);
  fpfh_estimation.setInputCloud(cloud);
  fpfh_estimation.setInputNormals(normals);
  fpfh_estimation.compute(*features);
  return features;
}

Cloud::ConstPtr Localizer::downsample(const Cloud::ConstPtr& scan) const {
  if (params_.query_voxel_size <= 0.) {
    return scan;
  }
  return raisin::voxelizePcd(*scan, static_cast<float>(params_.query_voxel_size));
}

void Localizer::setMap(const Cloud::ConstPtr& map) {
  map_ = map;
  // Feature matching runs on a downsampled copy of the map, at the same
  // resolution as the scan; the occupancy check below uses the full map.
  map_feature_cloud_ = downsample(map);
  map_features_ = extractFpfh(map_feature_cloud_);
  if (!map_features_->empty()) {
    feature_tree_.setInputCloud(map_features_);
  }
  buildOccupancyVoxels(*map);
}

void Localizer::buildOccupancyVoxels(const Cloud& map) {
  occupancy_voxels_.clear();
  // A voxel counts as occupied when some map point lies within one
  // correspondence distance of its centre, so each point marks the voxels in
  // its 27-neighbourhood that satisfy that.
  for (const auto& point : map.points) {
    const Eigen::Vector4f position(point.x, point.y, point.z, 1.f);
    const Eigen::Vector3i coord = voxelCoord(position);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const Eigen::Vector3i neighbor = coord + Eigen::Vector3i(dx, dy, dz);
          const Eigen::Vector4f center = voxelCenter(neighbor);
          if ((center - position).squaredNorm() <
              params_.max_correspondence_distance * params_.max_correspondence_distance) {
            occupancy_voxels_.insert(neighbor);
          }
        }
      }
    }
  }
}

Candidate Localizer::score(const Cloud& query, const Eigen::Matrix4f& transformation) const {
  Candidate candidate;
  candidate.transformation = transformation;
  int num_inliers = 0;
  double errors = 0.0;
  for (const auto& point : query.points) {
    const Eigen::Vector4f transformed =
      transformation * Eigen::Vector4f(point.x, point.y, point.z, 1.f);
    const Eigen::Vector3i coord = voxelCoord(transformed);
    if (occupancy_voxels_.find(coord) != occupancy_voxels_.end()) {
      ++num_inliers;
      errors += (transformed - voxelCenter(coord)).squaredNorm();
    }
  }
  candidate.inlier_fraction =
    query.empty() ? 0.0 : static_cast<double>(num_inliers) / static_cast<double>(query.size());
  candidate.error = num_inliers > 0 ? errors / num_inliers : std::numeric_limits<double>::max();
  return candidate;
}

std::vector<Candidate> Localizer::localize(
  const Cloud::ConstPtr& query, LocalizeStats* stats) const {
  std::vector<Candidate> selected;
  if (!map_features_ || map_features_->empty() || query->size() < 3) {
    return selected;
  }
  stats->query_points = static_cast<int>(query->size());

  const auto feature_start = std::chrono::steady_clock::now();
  const FeatureCloud::ConstPtr query_features = extractFpfh(query);

  // For each scan point, the k most feature-similar map points. RANSAC draws
  // correspondences from these lists instead of querying the tree per iteration.
  const int similar_count = std::max(1, params_.correspondence_randomness);
  std::vector<::pcl::Indices> similar_features(query->size());
#pragma omp parallel for num_threads(params_.num_threads)
  for (int i = 0; i < static_cast<int>(query->size()); ++i) {
    std::vector<float> squared_distances;
    feature_tree_.nearestKSearch(
      query_features->at(i), similar_count, similar_features[i], squared_distances);
  }
  stats->feature_ms = elapsedMs(feature_start);

  const auto ransac_start = std::chrono::steady_clock::now();
  const int thread_count = std::max(1, params_.num_threads);
  // A per-thread evaluation budget instead of a shared atomic: the shared
  // counter made the stopping point depend on thread interleaving, so repeat
  // runs disagreed on the candidate set.
  const int budget_per_thread =
    std::max(1, (params_.ransac_matching_budget + thread_count - 1) / thread_count);

  std::vector<std::vector<Candidate>> thread_candidates(thread_count);
  std::vector<int> thread_evaluated(thread_count, 0);

#pragma omp parallel num_threads(thread_count)
  {
    const int thread_id = omp_get_thread_num();
    // Deterministic per-thread seeds (same scheme as hdl_global_localization).
    std::mt19937 generator(
      thread_id * 8191 + thread_id + map_feature_cloud_->size() + query->size());

    ::pcl::registration::TransformationEstimationSVD<PointType, PointType> svd_estimation;
    ::pcl::registration::CorrespondenceRejectorPoly<PointType, PointType> polygon_rejection;
    polygon_rejection.setInputTarget(map_feature_cloud_);
    polygon_rejection.setInputSource(query);
    polygon_rejection.setCardinality(3);
    polygon_rejection.setSimilarityThreshold(params_.similarity_threshold);

    std::uniform_int_distribution<int> scan_index(0, static_cast<int>(query->size()) - 1);
    int evaluated = 0;
    for (int iteration = thread_id; iteration < params_.ransac_max_iterations;
         iteration += thread_count) {
      if (evaluated >= budget_per_thread) {
        break;
      }

      // 3 distinct scan points, each matched to a random one of its k candidates
      ::pcl::Indices samples(3);
      for (int i = 0; i < 3; ++i) {
        bool duplicated = true;
        while (duplicated) {
          samples[i] = scan_index(generator);
          duplicated = false;
          for (int j = 0; j < i; ++j) {
            if (samples[j] == samples[i]) {
              duplicated = true;
            }
          }
        }
      }
      ::pcl::Indices correspondences(3);
      for (int i = 0; i < 3; ++i) {
        const auto& candidates = similar_features[samples[i]];
        if (candidates.empty()) {
          correspondences[i] = 0;
        } else if (candidates.size() == 1) {
          correspondences[i] = candidates[0];
        } else {
          std::uniform_int_distribution<int> pick(0, static_cast<int>(candidates.size()) - 1);
          correspondences[i] = candidates[pick(generator)];
        }
      }

      if (!polygon_rejection.thresholdPolygon(samples, correspondences)) {
        continue;
      }

      Eigen::Matrix4f transformation;
      svd_estimation.estimateRigidTransformation(
        *query, samples, *map_feature_cloud_, correspondences, transformation);

      ++evaluated;
      const Candidate candidate = score(*query, transformation);
      if (candidate.inlier_fraction <= params_.min_inlier_fraction) {
        continue;
      }
      thread_candidates[thread_id].push_back(candidate);
    }
    thread_evaluated[thread_id] = evaluated;
  }

  std::vector<Candidate> all;
  for (int thread_id = 0; thread_id < thread_count; ++thread_id) {
    stats->evaluated += thread_evaluated[thread_id];
    all.insert(all.end(), thread_candidates[thread_id].begin(), thread_candidates[thread_id].end());
  }
  stats->passed = static_cast<int>(all.size());
  std::stable_sort(all.begin(), all.end(), betterCandidate);

  // Top-K with suppression: RANSAC returns many near-identical hits, and
  // handing polish ten copies of one pose wastes the budget.
  for (const Candidate& candidate : all) {
    if (static_cast<int>(selected.size()) >= params_.top_k) {
      break;
    }
    bool duplicate = false;
    for (const Candidate& kept : selected) {
      const double translation_difference =
        (candidate.transformation.block<3, 1>(0, 3) - kept.transformation.block<3, 1>(0, 3))
          .norm();
      if (translation_difference < params_.suppress_translation &&
          rotationDifferenceDegrees(candidate.transformation, kept.transformation) <
            params_.suppress_rotation_degrees) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      selected.push_back(candidate);
    }
  }
  stats->distinct_candidates = static_cast<int>(selected.size());
  stats->ransac_ms = elapsedMs(ransac_start);
  return selected;
}

Eigen::Vector3i Localizer::voxelCoord(const Eigen::Vector4f& point) const {
  return (point.array() / params_.max_correspondence_distance - 0.5).floor().cast<int>().head<3>();
}

Eigen::Vector4f Localizer::voxelCenter(const Eigen::Vector3i& coord) const {
  const float resolution = static_cast<float>(params_.max_correspondence_distance);
  const Eigen::Vector3f origin = (coord.cast<float>().array() + 0.5f) * resolution;
  const Eigen::Vector3f offset = Eigen::Vector3f::Ones() * resolution * 0.5f;
  return (Eigen::Vector4f() << origin + offset, 1.f).finished();
}

bool parsePolishMethod(const std::string& name, PolishMethod* method) {
  if (name == "none") {
    *method = PolishMethod::None;
  } else if (name == "icp") {
    *method = PolishMethod::Icp;
  } else if (name == "gicp") {
    *method = PolishMethod::Gicp;
  } else if (name == "ndt") {
    *method = PolishMethod::Ndt;
  } else {
    return false;
  }
  return true;
}

const char* polishMethodName(PolishMethod method) {
  switch (method) {
    case PolishMethod::None:
      return "none";
    case PolishMethod::Icp:
      return "icp";
    case PolishMethod::Gicp:
      return "gicp";
    case PolishMethod::Ndt:
      return "ndt";
  }
  return "unknown";
}

std::string defaultPolishLabel(const PolishParams& params) {
  std::ostringstream label;
  label << polishMethodName(params.method);
  switch (params.method) {
    case PolishMethod::None:
      break;
    case PolishMethod::Icp:
    case PolishMethod::Gicp:
      label << "_d" << params.max_correspondence_distance << "_i" << params.max_iterations;
      break;
    case PolishMethod::Ndt:
      label << "_r" << params.ndt_resolution << "_i" << params.max_iterations;
      break;
  }
  return label.str();
}

struct PolishEngine::Impl {
  ::pcl::IterativeClosestPoint<PointType, PointType> icp;
  nano_gicp::NanoGICP<PointType, PointType> gicp;
  pclomp::NormalDistributionsTransform<PointType, PointType> ndt;
  Cloud::ConstPtr source; // last source handed to the engine
  Cloud aligned; // scratch output, reused across candidates
};

PolishEngine::PolishEngine(const PolishParams& params):
  params_(params), impl_(std::make_unique<Impl>()) {
  switch (params_.method) {
    case PolishMethod::None:
      break;
    case PolishMethod::Icp:
      impl_->icp.setMaxCorrespondenceDistance(params_.max_correspondence_distance);
      impl_->icp.setMaximumIterations(params_.max_iterations);
      impl_->icp.setTransformationEpsilon(params_.transformation_epsilon);
      impl_->icp.setEuclideanFitnessEpsilon(0.); // iterate until the pose stops moving
      break;
    case PolishMethod::Gicp:
      impl_->gicp.setNumThreads(params_.num_threads);
      impl_->gicp.setCorrespondenceRandomness(params_.correspondence_randomness);
      impl_->gicp.setMaxCorrespondenceDistance(params_.max_correspondence_distance);
      impl_->gicp.setMaximumIterations(params_.max_iterations);
      impl_->gicp.setTransformationEpsilon(params_.transformation_epsilon);
      break;
    case PolishMethod::Ndt:
      impl_->ndt.setNumThreads(params_.num_threads);
      impl_->ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
      // Resolution must be set before setInputTarget: the voxel grid is built
      // there and setResolution only rebuilds it once a source exists.
      impl_->ndt.setResolution(static_cast<float>(params_.ndt_resolution));
      impl_->ndt.setStepSize(params_.ndt_step_size);
      impl_->ndt.setMaximumIterations(params_.max_iterations);
      impl_->ndt.setTransformationEpsilon(params_.transformation_epsilon);
      break;
  }
}

PolishEngine::~PolishEngine() = default;

void PolishEngine::setTarget(const Cloud::ConstPtr& map) {
  impl_->source.reset();
  switch (params_.method) {
    case PolishMethod::None:
      break;
    case PolishMethod::Icp:
      impl_->icp.setInputTarget(map);
      break;
    case PolishMethod::Gicp:
      impl_->gicp.setInputTarget(map);
      impl_->gicp.calculateTargetCovariances();
      break;
    case PolishMethod::Ndt:
      impl_->ndt.setInputTarget(map);
      break;
  }
}

bool PolishEngine::refine(
  const Cloud::ConstPtr& query, const Eigen::Matrix4f& initial, Eigen::Matrix4f* refined) {
  *refined = initial;
  if (params_.method == PolishMethod::None) {
    return true;
  }

  // Source-side setup is per scan, not per candidate.
  const bool new_source = impl_->source != query;
  impl_->source = query;

  // The result is taken whatever the convergence flag says. The three
  // implementations define convergence differently, and the occupancy rescore
  // that follows already rejects a polish that made the pose worse; dropping
  // results on the flag alone would compare the flags, not the methods.
  switch (params_.method) {
    case PolishMethod::None:
      return true;
    case PolishMethod::Icp:
      if (new_source) {
        impl_->icp.setInputSource(query);
      }
      impl_->icp.align(impl_->aligned, initial);
      *refined = impl_->icp.getFinalTransformation();
      return impl_->icp.hasConverged();
    case PolishMethod::Gicp:
      if (new_source) {
        impl_->gicp.setInputSource(query);
        impl_->gicp.calculateSourceCovariances();
      }
      impl_->gicp.align(impl_->aligned, initial);
      *refined = impl_->gicp.getFinalTransformation();
      return impl_->gicp.hasConverged();
    case PolishMethod::Ndt:
      if (new_source) {
        impl_->ndt.setInputSource(query);
      }
      impl_->ndt.align(impl_->aligned, initial);
      *refined = impl_->ndt.getFinalTransformation();
      return impl_->ndt.hasConverged();
  }
  return false;
}

} // namespace glexp
