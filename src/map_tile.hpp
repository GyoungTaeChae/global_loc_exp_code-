#pragma once

// Builds the prior map the localizer searches: a disc of accumulated
// ground-truth-posed frames from the map session, centred on a coarse grid
// point. Query frames that snap to the same grid point share one tile, so the
// expensive per-tile work (FPFH over the map, polish target structures) is paid
// once for many queries.

#include <cmath>

#include <Eigen/Core>

#include "mulran_sequence.hpp"

namespace glexp
{

struct MapTileParams {
  double radius = 80.0; // accumulate map frames whose GT position is within this [m]
  double voxel_size = 0.5; // map resolution [m]
  double grid_size = 24.0; // tile centre spacing [m]
  int min_frames = 20; // tiles thinner than this are skipped
};

/**
 * Snaps a position to the centre of its tile. The z of the tile centre is left
 * at the query height: the disc test is horizontal only.
 */
inline Eigen::Vector3d tileCenter(const Eigen::Vector3d& position, double grid_size) {
  Eigen::Vector3d center = position;
  center.x() = std::floor(position.x() / grid_size + 0.5) * grid_size;
  center.y() = std::floor(position.y() / grid_size + 0.5) * grid_size;
  return center;
}

/**
 * Accumulates every map-session frame whose ground-truth position falls inside
 * params.radius of center and keeps the points inside that same disc.
 *
 * @param excluded_frames  frames within this many indices of exclude_center are
 *                         dropped; use it when map and query are the same
 *                         session so the query scan cannot match itself.
 */
template <typename SequenceType>
mulran::Cloud::Ptr buildMapTile(SequenceType& map_sequence, const Eigen::Vector3d& center,
  const MapTileParams& params, long exclude_center, int excluded_frames,
  int* contributing_frames) {
  const double radius_squared = params.radius * params.radius;
  auto accumulated = std::make_shared<mulran::Cloud>();
  *contributing_frames = 0;

  for (std::size_t frame = 0; frame < map_sequence.size(); ++frame) {
    if (!map_sequence.hasPose(frame)) {
      continue;
    }
    if (exclude_center >= 0 &&
        std::abs(static_cast<long>(frame) - exclude_center) <= excluded_frames) {
      continue;
    }
    const Eigen::Vector3d offset = map_sequence.lidarPosition(frame) - center;
    if (offset.head<2>().squaredNorm() >= radius_squared) {
      continue;
    }

    const auto voxelized = map_sequence.voxelizedScan(frame);
    const Eigen::Matrix4f pose = map_sequence.lidarPose(frame).template cast<float>();
    const Eigen::Vector3f center_float = center.cast<float>();
    // No reserve() per frame on purpose: resizing exactly on every frame turns
    // the accumulation into O(n^2) copying over several hundred frames.
    for (const PointType& point : voxelized->points) {
      const Eigen::Vector3f world =
        pose.block<3, 3>(0, 0) * Eigen::Vector3f(point.x, point.y, point.z) +
        pose.block<3, 1>(0, 3);
      const float dx = world.x() - center_float.x();
      const float dy = world.y() - center_float.y();
      if (dx * dx + dy * dy >= radius_squared) {
        continue;
      }
      PointType transformed = point;
      transformed.x = world.x();
      transformed.y = world.y();
      transformed.z = world.z();
      accumulated->push_back(transformed);
    }
    ++(*contributing_frames);
  }
  accumulated->width = accumulated->size();
  accumulated->height = 1;
  accumulated->is_dense = false;

  return raisin::voxelizePcd(*accumulated, static_cast<float>(params.voxel_size));
}

} // namespace glexp
