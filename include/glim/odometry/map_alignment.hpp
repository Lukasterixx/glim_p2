#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace spdlog {
class logger;
}

namespace gtsam_points {
class PointCloudCPU;
class NearestNeighborSearch;
}  // namespace gtsam_points

namespace glim {

/// @brief Parameters for the global (no-initial-guess) alignment of a live scan cloud onto a saved map.
struct MapAlignerParams {
  double voxel_resolution = 0.5;   ///< Voxel downsample applied to both the map and the source cloud before FPFH
  double fpfh_radius = 2.5;        ///< FPFH feature search radius [m]
  int dof = 4;                     ///< 4 (XYZ + yaw, any-yaw planar) or 6 (full SE3)
  double search_radius_m = 1.5;    ///< Reject a solution whose translation exceeds this (assumed spawn radius)
  double min_inlier_rate = 0.5;    ///< Reject a solution below this RANSAC/GNC inlier rate
  std::string registration = "RANSAC";  ///< "RANSAC" (default; falls back to GNC) or "GNC"
  int num_threads = 4;
};

/// @brief Result of a global alignment attempt.
struct MapAlignmentResult {
  bool success = false;
  Eigen::Isometry3d T_target_source = Eigen::Isometry3d::Identity();  ///< saved-map <- source (world-frame) cloud
  double inlier_rate = 0.0;
};

/**
 * @brief Robust global alignment of an accumulated scan cloud onto a previously-saved GLIM map.
 *
 * Loads a `GlobalMapping::save` dump (or takes a pre-loaded map cloud), builds an FPFH feature
 * target, and registers a source point cloud (given in a world/odom frame) onto it with NO initial
 * guess via `gtsam_points::estimate_pose_ransac`/`estimate_pose_gnc`. Used both for the sim one-shot
 * initial alignment (external frontend) and the real-robot localizer bootstrap.
 */
class MapAligner {
public:
  /// @brief Load a saved-map directory and build the FPFH target.
  MapAligner(const std::string& map_path, const MapAlignerParams& params, std::shared_ptr<spdlog::logger> logger = nullptr);
  /// @brief Build the FPFH target from an already-loaded, world-frame map cloud.
  MapAligner(const std::shared_ptr<gtsam_points::PointCloudCPU>& map_cloud, const MapAlignerParams& params, std::shared_ptr<spdlog::logger> logger = nullptr);
  ~MapAligner();

  /// @brief True if the target was built successfully and align() can run.
  bool valid() const;

  /// @brief Number of feature points in the target (0 if invalid).
  size_t target_size() const;

  /**
   * @brief Register the source cloud onto the saved map, applying the inlier/translation gates.
   * @param source_points  Source points in a world/odom frame (homogeneous XYZ1).
   * @param params         Alignment parameters (dof, gates, threads).
   * @return               T_target_source = saved-map <- source, with success flagged by the gates.
   */
  MapAlignmentResult align(const std::vector<Eigen::Vector4d>& source_points, const MapAlignerParams& params) const;

  /// @brief Load all submaps under `map_path` and concatenate their world-frame points (nullptr on failure).
  static std::shared_ptr<gtsam_points::PointCloudCPU> load_map_cloud(const std::string& map_path, std::shared_ptr<spdlog::logger> logger = nullptr);

private:
  void build(const std::shared_ptr<gtsam_points::PointCloudCPU>& map_cloud, const MapAlignerParams& params, const std::shared_ptr<spdlog::logger>& logger);

  std::shared_ptr<gtsam_points::PointCloudCPU> target_;
  std::shared_ptr<gtsam_points::NearestNeighborSearch> target_tree_;
  std::vector<Eigen::Matrix<double, 33, 1>> target_fpfh_;  // kept alive: the feature tree points into it
  std::shared_ptr<gtsam_points::NearestNeighborSearch> target_fpfh_tree_;
};

}  // namespace glim
