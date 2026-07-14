#include <glim/odometry/odometry_estimation_localizer.hpp>

#include <chrono>
#include <memory>

#include <spdlog/spdlog.h>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <glim/util/config.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/odometry_estimation_imu.hpp>

namespace glim {

namespace {
// Convert GLIM's double-seconds stamp to a ROS time without pulling in ros_cloud_converter
// (which would drag a sensor_msgs dependency into this target).
builtin_interfaces::msg::Time to_ros_time(double t_sec) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(t_sec);
  stamp.nanosec = static_cast<uint32_t>((t_sec - static_cast<double>(stamp.sec)) * 1e9);
  return stamp;
}
}  // namespace

void OdometryEstimationLocalizer::setup_ros() {
  const auto params = static_cast<const OdometryEstimationLocalizerParams*>(this->params.get());

  // The host (glim_ros2) process already called rclcpp::init, so we only create a private node.
  node = rclcpp::Node::make_shared("glim_localizer");

  // --- Frame ids (shared with the rest of glim_ros2) ---
  const Config config_ros(GlobalConfig::get_config_path("config_ros"));
  const std::string odom_frame_id = config_ros.param<std::string>("glim_ros", "odom_frame_id", "odom");
  std::string imu_frame_id = config_ros.param<std::string>("glim_ros", "imu_frame_id", "");
  std::string lidar_frame_id = config_ros.param<std::string>("glim_ros", "lidar_frame_id", "");
  if (imu_frame_id.empty()) imu_frame_id = "imu";
  if (lidar_frame_id.empty()) lidar_frame_id = "livox_frame";

  const std::string saved_map_frame_id = params->saved_map_frame_id;
  const bool publish_tf = params->publish_tf;

  // --- Manual relocalization (RViz "2D Pose Estimate") ---
  auto initialpose_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    params->initialpose_topic,
    rclcpp::QoS(1),
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
      Eigen::Isometry3d T_map_imu;
      tf2::fromMsg(msg->pose.pose, T_map_imu);
      this->request_relocalization(T_map_imu);
    });

  // --- Offset publishers + a tf2 listener for odom->lidar ---
  // Latched offset (saved_map -> odom): the constant frame offset downstream nodes want.
  auto offset_pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/" + params->offset_topic,
    rclcpp::QoS(1).transient_local());
  // Live robot pose in the saved-map frame.
  auto localized_odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(
    "~/" + params->localized_odom_topic,
    rclcpp::SystemDefaultsQoS());

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  if (publish_tf) {
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*node);
  }

  auto logger = this->logger;

  // Publish the localization each time the estimator finalizes a new frame. `frame->T_world_imu`
  // lives in the estimator's continuous world frame; composing the map offset (T_map_world,
  // maintained by the bootstrap / initialpose) yields the pose in the saved-map frame. This
  // callback runs synchronously on the odometry thread, where T_map_world is written.
  OdometryEstimationCallbacks::on_update_new_frame.add(
    [this, offset_pub, localized_odom_pub, tf_buffer, tf_broadcaster, logger, odom_frame_id, imu_frame_id, lidar_frame_id, saved_map_frame_id, publish_tf](
      const EstimationFrame::ConstPtr& frame) {
      if (!this->bootstrapped || !frame) {
        return;
      }
      // Read T_lidar_imu from params here (not captured by value) to avoid Eigen alignment
      // issues from storing a fixed-size Eigen type inside a std::function.
      const Eigen::Isometry3d T_lidar_imu = static_cast<const OdometryEstimationLocalizerParams*>(this->params.get())->T_lidar_imu;

      const Eigen::Isometry3d T_savedmap_imu = this->T_map_world * frame->T_world_imu;
      const auto stamp = to_ros_time(frame->stamp);

      // Live pose in the saved-map frame.
      {
        const Eigen::Quaterniond q(T_savedmap_imu.linear());
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = saved_map_frame_id;
        odom.child_frame_id = imu_frame_id;
        odom.pose.pose.position.x = T_savedmap_imu.translation().x();
        odom.pose.pose.position.y = T_savedmap_imu.translation().y();
        odom.pose.pose.position.z = T_savedmap_imu.translation().z();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        localized_odom_pub->publish(odom);
      }

      // Offset saved_map -> odom = T_savedmap_imu * (T_odom_lidar * T_lidar_imu)^-1.
      // Needs the live odom->lidar TF (published by Isaac / the drivers). Non-blocking lookup.
      Eigen::Isometry3d T_odom_lidar;
      bool have_odom = false;
      try {
        auto tf = tf_buffer->lookupTransform(odom_frame_id, lidar_frame_id, tf2::timeFromSec(frame->stamp));
        T_odom_lidar = tf2::transformToEigen(tf);
        have_odom = true;
      } catch (const tf2::TransformException&) {
        try {
          auto tf = tf_buffer->lookupTransform(odom_frame_id, lidar_frame_id, tf2::TimePointZero);
          T_odom_lidar = tf2::transformToEigen(tf);
          have_odom = true;
        } catch (const tf2::TransformException&) {
          have_odom = false;
        }
      }

      if (!have_odom) {
        return;
      }

      const Eigen::Isometry3d T_odom_imu = T_odom_lidar * T_lidar_imu;
      const Eigen::Isometry3d T_savedmap_odom = T_savedmap_imu * T_odom_imu.inverse();

      geometry_msgs::msg::PoseWithCovarianceStamped offset;
      offset.header.stamp = stamp;
      offset.header.frame_id = saved_map_frame_id;
      const Eigen::Quaterniond qo(T_savedmap_odom.linear());
      offset.pose.pose.position.x = T_savedmap_odom.translation().x();
      offset.pose.pose.position.y = T_savedmap_odom.translation().y();
      offset.pose.pose.position.z = T_savedmap_odom.translation().z();
      offset.pose.pose.orientation.x = qo.x();
      offset.pose.pose.orientation.y = qo.y();
      offset.pose.pose.orientation.z = qo.z();
      offset.pose.pose.orientation.w = qo.w();
      offset_pub->publish(offset);

      if (publish_tf && tf_broadcaster) {
        geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(T_savedmap_odom);
        tf.header.stamp = stamp;
        tf.header.frame_id = saved_map_frame_id;
        tf.child_frame_id = odom_frame_id;
        tf_broadcaster->sendTransform(tf);
      }
    });

  logger->info("localizer ROS: initialpose='{}', offset topic='~/{}', localized odom='~/{}', publish_tf={}", params->initialpose_topic, params->offset_topic, params->localized_odom_topic, publish_tf);

  // Spin the private node on its own thread. Captured shared_ptrs keep the subscription,
  // publishers, and tf listener/broadcaster alive for the node's lifetime.
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  ros_running = true;
  ros_spin_thread = std::thread([this, executor, initialpose_sub, offset_pub, localized_odom_pub, tf_buffer, tf_listener, tf_broadcaster]() {
    while (ros_running && rclcpp::ok()) {
      executor->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
}

}  // namespace glim
