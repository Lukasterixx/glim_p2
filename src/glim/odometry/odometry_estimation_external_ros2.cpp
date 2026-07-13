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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace glim {

void OdometryEstimationExternal::setup_ros() {
  // The host (glim_ros2) process has already called rclcpp::init, so we only create
  // a private node + TF listener. spin_thread=true gives the listener its own executor
  // thread, so /tf and /tf_static are serviced independently of the odometry thread.
  node = rclcpp::Node::make_shared("glim_sim_odometry");
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, true);

  if (params.localize) {
    // Latched: the offset is constant (Isaac's odom is ground-truth), so one publish reaches late joiners.
    offset_pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/" + params.offset_topic,
      rclcpp::QoS(1).transient_local());
    if (params.publish_tf) {
      static_tf = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    }
  }
}

void OdometryEstimationExternal::publish_offset() {
  if (!offset_pub) {
    return;
  }

  // T_savedmap_odom expresses the current odom frame within the saved map.
  const Eigen::Quaterniond q(T_savedmap_odom.linear());
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = node->now();
  msg.header.frame_id = params.saved_map_frame_id;
  msg.pose.pose.position.x = T_savedmap_odom.translation().x();
  msg.pose.pose.position.y = T_savedmap_odom.translation().y();
  msg.pose.pose.position.z = T_savedmap_odom.translation().z();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();
  offset_pub->publish(msg);

  if (params.publish_tf && static_tf) {
    geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(T_savedmap_odom);
    tf.header.stamp = node->now();
    tf.header.frame_id = params.saved_map_frame_id;
    tf.child_frame_id = params.odom_frame_id;
    static_tf->sendTransform(tf);
  }

  logger->info("published saved_map->odom offset on ~/{} (frame '{}'){}", params.offset_topic, params.saved_map_frame_id, params.publish_tf ? " + static TF" : "");
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
