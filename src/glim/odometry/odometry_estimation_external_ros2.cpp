#include <glim/odometry/odometry_estimation_external.hpp>

#include <cmath>
#include <chrono>
#include <thread>
#include <string>

#include <spdlog/spdlog.h>

#include <tf2/time.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace glim {

void OdometryEstimationExternal::setup_ros() {
  // The host (glim_ros2) process has already called rclcpp::init, so we only create
  // a private node + TF listener. spin_thread=true gives the listener its own executor
  // thread, so /tf and /tf_static are serviced independently of the odometry thread.
  node = rclcpp::Node::make_shared("glim_sim_odometry");
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, true);
}

bool OdometryEstimationExternal::lookup_T_odom_lidar(const double stamp, Eigen::Isometry3d& T_odom_lidar) {
  // Convert the scan stamp (seconds since epoch) to a tf2::TimePoint the same way
  // tf2_ros converts message header stamps, so buffered TF stamps and this query align.
  const tf2::TimePoint time_point(std::chrono::duration_cast<tf2::Duration>(std::chrono::duration<double>(stamp)));

  // Wait (in wall-clock) up to lookup_timeout for the transform to arrive. We poll the
  // buffer with a zero-timeout lookup rather than lookupTransform's clock-based timeout
  // so this behaves the same under sim time (/clock) and system time.
  const int max_attempts = std::max(1, static_cast<int>(std::ceil(params.lookup_timeout / 0.005)));
  std::string last_error;
  for (int attempt = 0; attempt < max_attempts; attempt++) {
    try {
      const geometry_msgs::msg::TransformStamped tf = tf_buffer->lookupTransform(params.odom_frame_id, params.lidar_frame_id, time_point);
      T_odom_lidar = tf2::transformToEigen(tf);
      return true;
    } catch (const tf2::TransformException& e) {
      last_error = e.what();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // Throttle: missing transforms are expected briefly at startup before the simulator
  // begins publishing, so avoid flooding the log.
  static long fail_count = 0;
  if (fail_count++ % 50 == 0) {
    logger->warn("failed to look up transform {} -> {} at stamp={:.6f} ({})", params.odom_frame_id, params.lidar_frame_id, stamp, last_error);
  }
  return false;
}

}  // namespace glim
