#include <glim/odometry/odometry_estimation_external.hpp>

#include <spdlog/spdlog.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/odometry/map_alignment.hpp>

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

  // One-shot localization against a saved map (sim-mode)
  localize = config.param<bool>("odometry_estimation", "localize", false);
  prior_map_path = config.param<std::string>("odometry_estimation", "prior_map_path", "/tmp/dump");
  accumulation_secs = config.param<double>("odometry_estimation", "accumulation_secs", 3.0);
  search_radius_m = config.param<double>("odometry_estimation", "search_radius_m", 1.5);
  dof = config.param<int>("odometry_estimation", "dof", 4);
  min_inlier_rate = config.param<double>("odometry_estimation", "min_inlier_rate", 0.5);
  align_voxel_resolution = config.param<double>("odometry_estimation", "align_voxel_resolution", 0.5);
  align_fpfh_radius = config.param<double>("odometry_estimation", "align_fpfh_radius", 2.5);
  registration = config.param<std::string>("odometry_estimation", "registration", "RANSAC");
  publish_tf = config.param<bool>("odometry_estimation", "publish_tf", false);
  saved_map_frame_id = config.param<std::string>("odometry_estimation", "saved_map_frame_id", "saved_map");
  offset_topic = config.param<std::string>("odometry_estimation", "offset_topic", "map_localization");
}

OdometryEstimationExternalParams::~OdometryEstimationExternalParams() {}

OdometryEstimationExternal::OdometryEstimationExternal(const OdometryEstimationExternalParams& params)
: params(params),
  frame_count(0),
  accum_first_stamp(-1.0),
  aligning(false),
  align_done(false),
  align_success(false),
  aligned(false),
  logger(create_module_logger("odom_ext")) {
  logger->info("starting external (sim-mode) odometry estimation");
  logger->info("using external transform {} -> {}", params.odom_frame_id, params.lidar_frame_id);

  if (params.lidar_frame_id.empty()) {
    logger->warn("lidar_frame_id is empty; external transform lookup will fail. Set glim_ros/lidar_frame_id in config_ros.json");
  }

  T_savedmap_odom.setIdentity();

  covariance_estimation.reset(new CloudCovarianceEstimation(params.num_threads));
  setup_ros();

  // One-shot localization: build the FPFH alignment target from the saved map (if present).
  if (params.localize) {
    MapAlignerParams ap;
    ap.voxel_resolution = params.align_voxel_resolution;
    ap.fpfh_radius = params.align_fpfh_radius;
    ap.dof = params.dof;
    ap.search_radius_m = params.search_radius_m;
    ap.min_inlier_rate = params.min_inlier_rate;
    ap.registration = params.registration;
    ap.num_threads = params.num_threads;
    aligner = std::make_shared<MapAligner>(params.prior_map_path, ap, logger);
    if (aligner->valid()) {
      logger->info("sim-mode localization enabled: will align onto '{}' after {:.1f}s of scans", params.prior_map_path, params.accumulation_secs);
    } else {
      logger->warn("sim-mode localization requested but no usable map at '{}'; running as a plain mapping session", params.prior_map_path);
    }
  }

  logger->info("ready");
}

OdometryEstimationExternal::~OdometryEstimationExternal() {
  if (align_thread.joinable()) {
    align_thread.join();
  }
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

  // One-shot localization against a saved map (does not affect the pose/mapping; only publishes the offset).
  localization_step(new_frame, T_odom_lidar);

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

void OdometryEstimationExternal::localization_step(const EstimationFrame::Ptr& new_frame, const Eigen::Isometry3d& T_odom_lidar) {
  if (!params.localize || !aligner || !aligner->valid() || aligned.load()) {
    return;
  }

  // Apply a finished alignment (publish the offset) or clear the window and retry on failure.
  if (align_done.load()) {
    if (align_thread.joinable()) {
      align_thread.join();
    }
    aligning = false;
    align_done = false;
    if (align_success) {
      aligned = true;
      publish_offset();
    } else {
      accum_points.clear();
      accum_first_stamp = -1.0;
    }
    return;
  }

  if (aligning.load()) {
    return;
  }

  // Accumulate this scan in the odom frame (T_odom_lidar * p_lidar).
  if (!new_frame->frame || new_frame->frame->size() == 0) {
    return;
  }
  if (accum_first_stamp < 0.0) {
    accum_first_stamp = new_frame->stamp;
  }
  const Eigen::Matrix4d T = T_odom_lidar.matrix();
  const auto* pts = new_frame->frame->points;
  const size_t n = new_frame->frame->size();
  const size_t stride = std::max<size_t>(1, n / 20000);  // keep the buffer bounded on dense scans
  accum_points.reserve(accum_points.size() + n / stride + 1);
  for (size_t i = 0; i < n; i += stride) {
    accum_points.emplace_back(T * pts[i]);
  }

  // Once the window has elapsed, run the one-shot global alignment on a worker thread.
  if (new_frame->stamp - accum_first_stamp >= params.accumulation_secs && !accum_points.empty()) {
    std::vector<Eigen::Vector4d> snapshot;
    snapshot.swap(accum_points);
    accum_first_stamp = -1.0;
    aligning = true;
    if (align_thread.joinable()) {
      align_thread.join();
    }
    align_thread = std::thread([this, s = std::move(snapshot)]() mutable { this->run_alignment(std::move(s)); });
  }
}

void OdometryEstimationExternal::run_alignment(std::vector<Eigen::Vector4d> src_points) {
  MapAlignerParams ap;
  ap.voxel_resolution = params.align_voxel_resolution;
  ap.fpfh_radius = params.align_fpfh_radius;
  ap.dof = params.dof;
  ap.search_radius_m = params.search_radius_m;
  ap.min_inlier_rate = params.min_inlier_rate;
  ap.registration = params.registration;
  ap.num_threads = params.num_threads;

  align_success = false;
  const auto result = aligner->align(src_points, ap);
  if (result.success) {
    T_savedmap_odom = result.T_target_source;
    align_success = true;
    const Eigen::Vector3d t = T_savedmap_odom.translation();
    const double yaw = std::atan2(T_savedmap_odom.linear()(1, 0), T_savedmap_odom.linear()(0, 0));
    logger->info("sim-mode initial alignment accepted: saved_map<-odom x={:.2f} y={:.2f} z={:.2f} yaw={:.1f}deg inlier_rate={:.2f}", t.x(), t.y(), t.z(), yaw * 180.0 / M_PI, result.inlier_rate);
  } else {
    logger->warn("sim-mode initial alignment did not pass the gates; re-accumulating and retrying");
  }
  align_done = true;
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
