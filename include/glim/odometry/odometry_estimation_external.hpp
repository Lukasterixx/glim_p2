#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/odometry/odometry_estimation_base.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace spdlog {
class logger;
}

namespace glim {

class CloudCovarianceEstimation;

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

private:
  OdometryEstimationExternalParams params;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  long frame_count;                ///< Number of frames processed so far
  EstimationFrame::Ptr last_frame; ///< Previous frame, held back one step before marginalization

  // ROS / TF (created in setup_ros)
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim
