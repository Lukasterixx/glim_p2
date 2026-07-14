#pragma once

#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/odometry/odometry_estimation_imu.hpp>

namespace gtsam_points {
class PointCloudCPU;
class GaussianVoxelMapCPU;
class GaussianVoxelMapGPU;
class CUDAStream;
class StreamTempBufferRoundRobin;
}  // namespace gtsam_points

namespace spdlog {
class logger;
}

namespace rclcpp {
class Node;
}

namespace glim {

class MapAligner;

/**
 * @brief Parameters for OdometryEstimationLocalizer
 */
struct OdometryEstimationLocalizerParams : public OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationLocalizerParams();
  virtual ~OdometryEstimationLocalizerParams();

public:
  // Prior map
  std::string prior_map_path;  ///< Directory produced by GlobalMapping::save

  // Registration params (same knobs as OdometryEstimationCPU, VGICP path)
  bool enable_gpu;                      ///< Use GPU VGICP (GaussianVoxelMapGPU + IntegratedVGICPFactorGPU) instead of CPU
  std::string registration_type;        ///< Registration type (only "VGICP" is supported for now)
  int max_iterations;                   ///< Maximum number of scan-matching iterations (CPU path only)
  double target_downsampling_rate;      ///< Downsampling rate applied to the prior map points at load time (<=0 disables)
  double vgicp_resolution;              ///< Voxelmap resolution (for VGICP)
  int vgicp_voxelmap_levels;            ///< Multi-resolution voxelmap levels (for VGICP)
  double vgicp_voxelmap_scaling_factor; ///< Multi-resolution voxelmap scaling factor (for VGICP)

  // Live rolling local map ("mapping for robustness, not for persistence").
  // A short LRU-bounded voxelmap fed by the live scans, matched against IN ADDITION to the fixed
  // prior map. It fills geometric gaps where the prior map is too sparse for a stable frame-to-map
  // fit, without ever growing or saving the prior map. The prior map stays the stronger anchor.
  bool enable_live_mapping;             ///< Also match each scan against a rolling live local map (CPU path)
  double live_map_resolution;           ///< Live voxelmap resolution (m); defaults to vgicp_resolution
  int live_map_lru_thresh;              ///< LRU horizon in inserts; ~scans kept (e.g. 75 ≈ 7s @ 10 Hz)
  double live_map_downsampling_rate;    ///< Random-subsample rate for the live-map source/insert (keeps prior stronger)
  bool live_map_gap_only;               ///< Insert only points where the prior map is thin (prevents the live map from competing with the prior)
  int live_map_gap_min_points;          ///< A finest-level prior voxel with >= this many points counts as "covered" (its area gets no live points)

  // Runtime relocalization (RViz "2D Pose Estimate")
  bool enable_relocalization;   ///< Subscribe to a pose topic and re-seed the estimator on request
  std::string initialpose_topic;  ///< Topic carrying geometry_msgs/PoseWithCovarianceStamped (map frame)

  // Automatic global-localization bootstrap (no initial guess): accumulate a few seconds
  // of scans, then FPFH+RANSAC/GNC-register them onto the prior map to find the start pose.
  bool auto_bootstrap;                  ///< Run the automatic global bootstrap at startup (replaces manual initialpose)
  double bootstrap_accumulation_secs;   ///< Seconds of LiDAR to accumulate before attempting the global fix
  double bootstrap_search_radius_m;     ///< Reject a bootstrap whose frame-origin translation exceeds this (spawn radius)
  bool bootstrap_refine;                ///< Fine-register the coarse FPFH pose with VGICP against the prior map before accepting it
  double bootstrap_refine_max_correction;  ///< Reject a refinement that moves more than this from the coarse pose (m)
  double bootstrap_max_z_offset;        ///< If >=0: the LiDAR always starts at the map's recording height, so reject a coarse offset with |z| above this and pin z=0 before refining (-1 disables)
  int bootstrap_dof;                    ///< 4 (XYZ+yaw, any-yaw planar) or 6 (full SE3) for the global registration
  double bootstrap_min_inlier_rate;     ///< Reject a bootstrap below this RANSAC/GNC inlier rate
  double bootstrap_voxel_resolution;    ///< Voxel downsample applied to the map + accumulated cloud before FPFH
  double bootstrap_fpfh_radius;         ///< FPFH feature search radius (m)
  std::string bootstrap_registration;   ///< "RANSAC" (default) or "GNC"; RANSAC falls back to GNC on failure

  // Offset publishing (so downstream nodes learn where the robot is in the saved map)
  bool publish_tf;                      ///< Also broadcast the saved_map->odom TF (never touches Isaac's `map`)
  std::string saved_map_frame_id;       ///< Frame name for the loaded map (default "saved_map", NOT "map")
  std::string offset_topic;             ///< PoseWithCovarianceStamped carrying T_savedmap_odom
  std::string localized_odom_topic;     ///< nav_msgs/Odometry carrying the live pose in the saved-map frame
};

/**
 * @brief LiDAR-IMU odometry that localizes within a fixed, pre-loaded prior map.
 *
 * This subclass reuses the entire OdometryEstimationIMU machinery (IMU preintegration,
 * fixed-lag smoother, deskewing, marginalization). The only differences from
 * OdometryEstimationCPU are:
 *   - the VGICP target voxelmap is built once from a saved map and never updated, and
 *   - each live scan is registered against that fixed target.
 *
 * The prior map enters the optimization only as the *fixed target* of unary matching-cost
 * factors, never as a GTSAM variable, so it cannot drift or grow.
 *
 * Frame handling (offset-based): the smoother runs forever in its own continuous world frame
 * (the frame it booted in) and is NEVER snapped — a multi-meter pose jump through the
 * fixed-lag smoother would contradict the already-inserted IMU factors and blow up the
 * velocity/bias states. Instead, global alignment (auto-bootstrap or /initialpose) only
 * updates the external offset `T_map_world` (saved-map <- estimator-world), which is applied
 * as the fixed *target pose* of the prior-map matching factors and composed into everything
 * published in the saved-map frame (localized_odom, saved_map->odom offset/TF).
 */
class OdometryEstimationLocalizer : public OdometryEstimationIMU {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationLocalizer(const OdometryEstimationLocalizerParams& params = OdometryEstimationLocalizerParams());
  virtual ~OdometryEstimationLocalizer() override;

  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;

  /**
   * @brief Request a relocalization: the map offset is redefined so the current pose maps to `T_map_imu`.
   *
   * The smoother itself is never touched (its continuous world frame stays put); only
   * `T_map_world` is recomputed as `T_map_imu * T_world_imu_current^-1`. The prior-map matching
   * factors immediately pull the estimate into fine alignment from there.
   *
   * @param T_map_imu  Requested IMU pose in the prior-map frame
   * @note  Thread-safe. The request is consumed on the odometry thread, not the caller's thread.
   */
  void request_relocalization(const Eigen::Isometry3d& T_map_imu);

private:
  virtual void create_frame(EstimationFrame::Ptr& frame) override;
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) override;
  virtual void fallback_smoother() override;

  /// @brief Load the prior map and build the fixed multi-resolution VGICP target.
  void load_prior_map(const std::string& path);

  /// @brief Build the FPFH/kd-tree registration target for the automatic global bootstrap (from `map_cloud`).
  void build_bootstrap_target(const std::shared_ptr<gtsam_points::PointCloudCPU>& map_cloud);

  /// @brief Accumulate the current scan (in the estimator's world frame) into the bootstrap buffer.
  void accumulate_bootstrap_frame(const int current);

  /// @brief Worker body: FPFH+RANSAC/GNC-register the accumulated cloud onto the prior map (runs off the odometry thread).
  void run_bootstrap(std::vector<Eigen::Vector4d> src_points);

  /// @brief Fine-register the accumulated cloud onto the prior map with VGICP, starting from the coarse
  ///        FPFH pose. Updates `T_map_world_coarse` in place; false = rejected (re-accumulate and retry).
  bool refine_bootstrap_alignment(const std::vector<Eigen::Vector4d>& src_points, Eigen::Isometry3d& T_map_world_coarse);

  /// @brief Install a new saved-map <- estimator-world offset (odometry thread only). Clears the
  ///        offset-dependent state (live map, bootstrap local model) and re-publishes the prior-map viz.
  void apply_map_offset(const Eigen::Isometry3d& T_map_world_new, const char* source);

  /// @brief (Re-)publish the prior map as a pseudo-keyframe posed at T_map_world^-1 so viewers render it
  ///        aligned with the estimator's continuous world frame.
  void publish_prior_map_viz();

  gtsam::NonlinearFactorGraph create_factors_cpu(const int current, gtsam::Values& new_values);
  gtsam::NonlinearFactorGraph create_factors_gpu(const int current, gtsam::Values& new_values);

  // Implemented in odometry_estimation_localizer_ros2.cpp
  void setup_ros();

private:
  bool use_gpu;                         ///< Effective GPU mode (enable_gpu requested AND CUDA available)
  Eigen::Isometry3d last_T_target_imu;  ///< Last IMU pose in the estimator world frame (CPU path: constant-velocity prediction)

  // Saved-map <- estimator-world offset (the ONLY place global alignment lives; the smoother is
  // never snapped). Identity until the bootstrap/initialpose defines it. Written and read on the
  // odometry thread only.
  Eigen::Isometry3d T_map_world;

  // Fixed prior-map target (world/map frame). Built once, never updated.
  std::vector<std::shared_ptr<gtsam_points::GaussianVoxelMapCPU>> target_voxelmaps;      ///< CPU VGICP target
  std::vector<std::shared_ptr<gtsam_points::GaussianVoxelMapGPU>> target_voxelmaps_gpu;  ///< GPU VGICP target
  EstimationFrame::ConstPtr prior_map_frame;  ///< Prior map points as a pseudo-keyframe (for visualization)

  // Live rolling local map (world/map frame). LRU-bounded, updated from the live scans, never saved.
  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> live_voxelmap;  ///< CPU VGICP live target (nullptr if disabled/GPU)
  std::mt19937 live_map_mt;                                          ///< RNG for live-map downsampling

  /// @brief Insert the (downsampled, gap-filtered) scan `frame_index` into the live rolling map at
  ///        its estimator-world pose `T_world_frame`. Used by both the CPU (current frame, post-solve)
  ///        and GPU (previous frame, already finalized by the smoother) paths.
  void update_live_map(int frame_index, const Eigen::Isometry3d& T_world_frame);

  // CUDA stream + temp buffer round-robin (GPU path only)
  std::unique_ptr<gtsam_points::CUDAStream> stream;
  std::unique_ptr<gtsam_points::StreamTempBufferRoundRobin> stream_buffer_roundrobin;

  // Pending relocalization request (set from the ROS thread, consumed on the odometry thread)
  std::mutex reloc_mutex;
  bool reloc_pending;
  Eigen::Isometry3d reloc_T_map_imu;

  // --- Automatic global-localization bootstrap ---
  // Shared FPFH+RANSAC/GNC global aligner built from the prior map (read-only after construction).
  std::shared_ptr<MapAligner> aligner;

  // Accumulation + worker state (accumulate/consume on the odometry thread; run_bootstrap on the worker).
  std::vector<Eigen::Vector4d> bootstrap_accum_points;  ///< Points in the estimator world frame, gathered over the window
  double bootstrap_first_stamp;                         ///< Stamp of the first accumulated scan (-1 = none yet)

  // Scan-to-local-model matching during the bootstrap window. Without it the pre-fix trajectory
  // is IMU dead-reckoning only, which drifts quadratically with any attitude/bias error and
  // smears the accumulated bootstrap submap. Built in the estimator's world frame; freed once
  // the map fix is applied.
  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> bootstrap_local_voxelmap;
  int bootstrap_last_inserted = -1;  ///< id of the last frame folded into bootstrap_local_voxelmap
  std::thread bootstrap_thread;
  std::atomic<bool> bootstrapping;   ///< A worker is currently registering
  std::atomic<bool> bootstrap_ready; ///< The worker finished; bootstrap_correction/bootstrap_success are valid
  bool bootstrap_success;            ///< Worker result: registration passed the gates
  Eigen::Isometry3d bootstrap_correction;  ///< estimator-world -> saved-map (worker output)
  std::atomic<bool> bootstrapped;    ///< The map fix has been applied (set on odometry thread, read when publishing)

  // Private ROS node + its own spin thread (created in setup_ros; only when relocalization is enabled).
  std::shared_ptr<rclcpp::Node> node;
  std::thread ros_spin_thread;
  std::atomic<bool> ros_running{false};

  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim
