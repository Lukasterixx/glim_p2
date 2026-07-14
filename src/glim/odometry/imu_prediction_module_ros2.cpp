#include <glim/odometry/imu_prediction_module.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#define GLIM_ROS2
#include <glim/util/config.hpp>
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {

std::vector<GenericTopicSubscription::Ptr> IMUPredictionModule::create_subscriptions(rclcpp::Node& node) {
  pred_odom_pub = node.create_publisher<nav_msgs::msg::Odometry>("~/predicted_odom", rclcpp::SystemDefaultsQoS());
  if (predict_odom_tf) {
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  }

  return {};
}

void IMUPredictionModule::publish_pred_frame(const EstimationFrame::ConstPtr& frame) {
  const double dt = frame->stamp - last_publish_time;
  if (dt < min_publish_interval) {
    // Avoid too frequent publishing, ROS2 can be slow to handle too many messages
    return;
  }
  // Update here (not only when the odom topic has a subscriber) so both the TF and the topic
  // stay throttled to max_publish_rate regardless of who is listening.
  last_publish_time = frame->stamp;

  if (imu_frame_id.empty()) {
    imu_frame_id = GlobalConfig::instance()->param<std::string>("meta", "imu_frame_id", "");

    if (imu_frame_id.empty()) {
      logger->warn("IMU frame ID is not set. Using 'imu' as default.");
      imu_frame_id = "imu";
    } else {
      logger->info("auto-detected IMU frame ID: {}", imu_frame_id);
    }
  }
  if (base_frame_id.empty()) {
    base_frame_id = imu_frame_id;
  }

  const Eigen::Isometry3d T_odom_imu(frame->T_world_imu);
  const Eigen::Quaterniond quat_odom_imu(T_odom_imu.linear());
  const Eigen::Vector3d v_odom_imu = frame->v_world_imu;

  // High-rate odom->base TF from the predicted pose. When predict_odom_tf is set, this module is
  // the SOLE owner of the odom->base edge (rviz_viewer suppresses its own), so downstream nav gets
  // the responsive predicted transform. base==imu here (GLIM's default); the general base!=imu case
  // needs the imu->base extrinsic and is left to the rviz_viewer path.
  if (predict_odom_tf && tf_broadcaster) {
    if (base_frame_id == imu_frame_id) {
      geometry_msgs::msg::TransformStamped trans;
      trans.header.stamp = from_sec(frame->stamp + tf_time_offset);
      trans.header.frame_id = odom_frame_id;
      trans.child_frame_id = base_frame_id;
      trans.transform.translation.x = T_odom_imu.translation().x();
      trans.transform.translation.y = T_odom_imu.translation().y();
      trans.transform.translation.z = T_odom_imu.translation().z();
      trans.transform.rotation.x = quat_odom_imu.x();
      trans.transform.rotation.y = quat_odom_imu.y();
      trans.transform.rotation.z = quat_odom_imu.z();
      trans.transform.rotation.w = quat_odom_imu.w();
      tf_broadcaster->sendTransform(trans);
    } else {
      static bool warned = false;
      if (!warned) {
        logger->warn("predict_odom_tf requires base_frame_id == imu_frame_id; not broadcasting predicted odom TF (base='{}', imu='{}')", base_frame_id, imu_frame_id);
        warned = true;
      }
    }
  }

  if (pred_odom_pub->get_subscription_count()) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = from_sec(frame->stamp);
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = imu_frame_id;
    odom.pose.pose.position.x = T_odom_imu.translation().x();
    odom.pose.pose.position.y = T_odom_imu.translation().y();
    odom.pose.pose.position.z = T_odom_imu.translation().z();
    odom.pose.pose.orientation.x = quat_odom_imu.x();
    odom.pose.pose.orientation.y = quat_odom_imu.y();
    odom.pose.pose.orientation.z = quat_odom_imu.z();
    odom.pose.pose.orientation.w = quat_odom_imu.w();

    odom.twist.twist.linear.x = v_odom_imu.x();
    odom.twist.twist.linear.y = v_odom_imu.y();
    odom.twist.twist.linear.z = v_odom_imu.z();

    pred_odom_pub->publish(odom);

    logger->debug("published predicted odom (stamp={:.6f})", frame->stamp);
  }
}

}  // namespace glim
