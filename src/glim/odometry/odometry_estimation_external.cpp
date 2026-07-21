#include <glim/odometry/odometry_estimation_external.hpp>

#include <spdlog/spdlog.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/odometry/callbacks.hpp>

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

OdometryEstimationExternalParams::OdometryEstimationExternalParams() {
  Config config(GlobalConfig::get_config_path("config_odometry"));
  num_threads = config.param<int>("odometry_estimation", "num_threads", 4);
  lookup_timeout = config.param<double>("odometry_estimation", "lookup_timeout", 0.1);

  // Frame IDs are shared with the ROS wrapper config
  Config config_ros(GlobalConfig::get_config_path("config_ros"));
  odom_frame_id = config_ros.param<std::string>("glim_ros", "odom_frame_id", "odom");
  lidar_frame_id = config_ros.param<std::string>("glim_ros", "lidar_frame_id", "livox_frame");
}

OdometryEstimationExternalParams::~OdometryEstimationExternalParams() {}

OdometryEstimationExternal::OdometryEstimationExternal(const OdometryEstimationExternalParams& params)
: params(params),
  frame_count(0),
  logger(create_module_logger("odom_ext")) {
  logger->info("starting external (sim-mode) odometry estimation");
  logger->info("using external transform {} -> {}", params.odom_frame_id, params.lidar_frame_id);

  if (params.lidar_frame_id.empty()) {
    logger->warn("lidar_frame_id is empty; external transform lookup will fail. Set glim_ros/lidar_frame_id in config_ros.json");
  }

  covariance_estimation.reset(new CloudCovarianceEstimation(params.num_threads));
  setup_ros();

  logger->info("ready");
}

OdometryEstimationExternal::~OdometryEstimationExternal() {
  // Do not call rclcpp::shutdown here; the ROS2 context is owned by the host node.
  tf_listener.reset();
  tf_buffer.reset();
  node.reset();
}

void OdometryEstimationExternal::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  // Sim-mode ignores IMU: the pose comes entirely from the external transform.
  (void)stamp;
  (void)linear_acc;
  (void)angular_vel;
}

EstimationFrame::ConstPtr OdometryEstimationExternal::insert_frame(const PreprocessedFrame::Ptr& raw_frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  Callbacks::on_insert_frame(raw_frame);

  // Look up the external sensor pose for this scan.
  Eigen::Isometry3d T_odom_lidar = Eigen::Isometry3d::Identity();
  if (!lookup_T_odom_lidar(raw_frame->stamp, T_odom_lidar)) {
    // No transform available for this scan; drop it so the pipeline keeps running.
    // Return the last valid state (or nullptr for the very first frame).
    return last_frame;
  }

  // Build the estimation frame (mirrors OdometryEstimationCT::insert_frame).
  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = frame_count++;
  new_frame->stamp = raw_frame->stamp;
  new_frame->T_lidar_imu.setIdentity();
  new_frame->v_world_imu.setZero();
  new_frame->imu_bias.setZero();
  new_frame->raw_frame = raw_frame;

  gtsam_points::PointCloudCPU::Ptr frame_cpu(new gtsam_points::PointCloudCPU(raw_frame->points));
  frame_cpu->add_times(raw_frame->times);
  covariance_estimation->estimate(raw_frame->points, raw_frame->neighbors, frame_cpu->normals_storage, frame_cpu->covs_storage);
  frame_cpu->normals = frame_cpu->normals_storage.data();
  frame_cpu->covs = frame_cpu->covs_storage.data();

  new_frame->frame = frame_cpu;
  new_frame->frame_id = FrameID::LIDAR;
  new_frame->set_T_world_sensor(FrameID::LIDAR, T_odom_lidar);

  Callbacks::on_new_frame(new_frame);

  // Marginalize the previous frame (poses are external and never revised, so a
  // one-frame lag is enough to keep the latest frame "active" for viewers).
  if (last_frame) {
    marginalized_frames.push_back(last_frame);
  }
  Callbacks::on_marginalized_frames(marginalized_frames);

  last_frame = new_frame;

  const std::vector<EstimationFrame::ConstPtr> active_frames = {new_frame};
  Callbacks::on_update_new_frame(new_frame);
  Callbacks::on_update_frames(active_frames);

  return new_frame;
}

std::vector<EstimationFrame::ConstPtr> OdometryEstimationExternal::get_remaining_frames() {
  std::vector<EstimationFrame::ConstPtr> remaining;
  if (last_frame) {
    remaining.push_back(last_frame);
    last_frame.reset();
  }
  return remaining;
}

}  // namespace glim
