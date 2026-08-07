// lidar_slam::Point is a custom point type, so libpcl ships no precompiled
// instances of the templates this experiment uses. The impl headers are pulled
// in here and instantiated once, keeping them out of the pipeline sources.

#include "lidar_slam/types.hpp"

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/impl/feature.hpp>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/features/impl/fpfh_omp.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/impl/correspondence_estimation.hpp>
#include <pcl/registration/impl/correspondence_rejection_poly.hpp>
#include <pcl/registration/impl/icp.hpp>
#include <pcl/registration/impl/transformation_estimation_svd.hpp>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

template class pcl::VoxelGrid<PointType>;
template class pcl::KdTreeFLANN<PointType>;
template class pcl::search::Search<PointType>;
template class pcl::search::KdTree<PointType>;
template class pcl::Feature<PointType, pcl::Normal>;
template class pcl::NormalEstimation<PointType, pcl::Normal>;
template class pcl::NormalEstimationOMP<PointType, pcl::Normal>;
template class pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33>;
template class pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33>;
template class pcl::registration::CorrespondenceRejectorPoly<PointType, PointType>;
template class pcl::registration::TransformationEstimationSVD<PointType, PointType>;

// Polish stage: point-to-point ICP and NDT. GICP comes from nano_gicp, which
// already instantiates itself for PointType.
template class pcl::Registration<PointType, PointType, float>;
template class pcl::IterativeClosestPoint<PointType, PointType, float>;
template class pcl::registration::CorrespondenceEstimation<PointType, PointType, float>;
template class pclomp::VoxelGridCovariance<PointType>;
template class pclomp::NormalDistributionsTransform<PointType, PointType>;
