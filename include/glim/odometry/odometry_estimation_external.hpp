#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/odometry/odometry_estimation_base.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace spdlog {
class logger;
}

namespace glim {

class CloudCovarianceEstimation;
class MapAligner;

/**
 * @brief Parameters for OdometryEstimationExternal
 */
struct OdometryEstimationExternalParams {
public:
  OdometryEstimationExternalParams();
  ~OdometryEstimationExternalParams();

public:
  int num_threads;          ///< Number of threads for covariance estimation
  double lookup_timeout;    ///< Max time [sec] to wait for the external transform to become available
  std::string odom_frame_id;   ///< Parent frame of the external transform (world frame for mapping)
  std::string lidar_frame_id;  ///< Child frame of the external transform (LiDAR frame)

  // --- One-shot localization against a saved map (sim-mode) ---
  // When `localize` is on and a saved map exists, accumulate a few seconds of scans, globally align
  // them onto the map ONCE (no drift correction: the external transform is already ground-truth), and
  // publish the resulting saved_map->odom offset. Mapping/visualization continue unchanged.
  bool localize;                    ///< Enable the one-shot initial alignment against a prior map
  std::string prior_map_path;       ///< Directory of a GlobalMapping::save dump to localize against
  double accumulation_secs;         ///< Seconds of LiDAR to accumulate before the one-shot alignment
  double search_radius_m;           ///< Reject an alignment whose translation exceeds this (spawn radius)
  int dof;                          ///< 4 (XYZ+yaw) or 6 (full SE3) for the global registration
  double min_inlier_rate;           ///< Reject an alignment below this inlier rate
  double align_voxel_resolution;    ///< Voxel downsample for the map + accumulated cloud before FPFH
  double align_fpfh_radius;         ///< FPFH feature search radius [m]
  std::string registration;         ///< "RANSAC" (default; falls back to GNC) or "GNC"
  bool publish_tf;                  ///< Also broadcast the saved_map->odom static TF (never touches `map`)
  std::string saved_map_frame_id;   ///< Frame name for the loaded map (default "saved_map", NOT "map")
  std::string offset_topic;         ///< PoseWithCovarianceStamped carrying the saved_map->odom offset
};

/**
 * @brief "Sim-mode" odometry that trusts an external `odom -> lidar` transform (e.g., from a simulator)
 *        instead of estimating the sensor pose. It looks up the transform for each scan via TF and
 *        emits the points at that pose so the mapping backend can build a map from ground-truth poses.
 * @note  Requires no IMU. Publishes no TF. Runs its own private ROS2 node + TF listener.
 */
class OdometryEstimationExternal : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationExternal(const OdometryEstimationExternalParams& params = OdometryEstimationExternalParams());
  virtual ~OdometryEstimationExternal() override;

  virtual bool requires_imu() const override { return false; }

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;

private:
  // Implemented in odometry_estimation_external_ros2.cpp
  void setup_ros();
  bool lookup_T_odom_lidar(const double stamp, Eigen::Isometry3d& T_odom_lidar);
  void publish_offset();  ///< Publish the (constant) saved_map->odom offset once aligned

  // One-shot localization helpers
  void localization_step(const EstimationFrame::Ptr& new_frame, const Eigen::Isometry3d& T_odom_lidar);
  void run_alignment(std::vector<Eigen::Vector4d> src_points);  ///< Worker body (off the odometry thread)

private:
  OdometryEstimationExternalParams params;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  long frame_count;                ///< Number of frames processed so far
  EstimationFrame::Ptr last_frame; ///< Previous frame, held back one step before marginalization

  // ROS / TF (created in setup_ros)
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  // One-shot localization (sim-mode): align accumulated scans onto a saved map, publish the offset.
  std::shared_ptr<MapAligner> aligner;
  std::vector<Eigen::Vector4d> accum_points;  ///< Scans accumulated in the odom frame for the one-shot align
  double accum_first_stamp;                   ///< Stamp of the first accumulated scan (-1 = none yet)
  std::thread align_thread;
  std::atomic<bool> aligning;    ///< A worker alignment is in flight
  std::atomic<bool> align_done;  ///< The worker finished; align_success/T_savedmap_odom are valid
  bool align_success;            ///< Worker result: alignment passed the gates
  std::atomic<bool> aligned;     ///< The offset has been found + published (one-shot latch)
  Eigen::Isometry3d T_savedmap_odom;  ///< The published offset (saved_map <- odom)

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr offset_pub;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf;

  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim
