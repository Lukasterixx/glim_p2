#include <glim/odometry/odometry_estimation_localizer.hpp>

#include <cmath>
#include <filesystem>

#include <spdlog/spdlog.h>

#include <gtsam_points/config.hpp>  // defines GTSAM_POINTS_USE_CUDA (must precede the CUDA #ifdef below)

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/odometry/map_alignment.hpp>

#ifdef GTSAM_POINTS_USE_CUDA
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/cuda/cuda_stream.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#endif

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/odometry/callbacks.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::V;  // IMU velocity
using gtsam::symbol_shorthand::X;  // IMU pose (T_world_imu)

using Callbacks = OdometryEstimationCallbacks;

OdometryEstimationLocalizerParams::OdometryEstimationLocalizerParams() {
  Config config(GlobalConfig::get_config_path("config_odometry"));

  prior_map_path = config.param<std::string>("odometry_estimation", "prior_map_path", "");

  enable_gpu = config.param<bool>("odometry_estimation", "enable_gpu", false);
  registration_type = config.param<std::string>("odometry_estimation", "registration_type", "VGICP");
  max_iterations = config.param<int>("odometry_estimation", "max_iterations", 5);
  target_downsampling_rate = config.param<double>("odometry_estimation", "target_downsampling_rate", 1.0);

  vgicp_resolution = config.param<double>("odometry_estimation", "vgicp_resolution", 0.5);
  vgicp_voxelmap_levels = config.param<int>("odometry_estimation", "vgicp_voxelmap_levels", 2);
  vgicp_voxelmap_scaling_factor = config.param<double>("odometry_estimation", "vgicp_voxelmap_scaling_factor", 2.0);

  enable_relocalization = config.param<bool>("odometry_estimation", "enable_relocalization", true);
  initialpose_topic = config.param<std::string>("odometry_estimation", "initialpose_topic", "/initialpose");

  auto_bootstrap = config.param<bool>("odometry_estimation", "auto_bootstrap", true);
  bootstrap_accumulation_secs = config.param<double>("odometry_estimation", "bootstrap_accumulation_secs", 3.0);
  bootstrap_search_radius_m = config.param<double>("odometry_estimation", "bootstrap_search_radius_m", 1.5);
  bootstrap_dof = config.param<int>("odometry_estimation", "bootstrap_dof", 4);
  bootstrap_min_inlier_rate = config.param<double>("odometry_estimation", "bootstrap_min_inlier_rate", 0.5);
  bootstrap_voxel_resolution = config.param<double>("odometry_estimation", "bootstrap_voxel_resolution", 0.5);
  bootstrap_fpfh_radius = config.param<double>("odometry_estimation", "bootstrap_fpfh_radius", 2.5);
  bootstrap_registration = config.param<std::string>("odometry_estimation", "bootstrap_registration", "RANSAC");

  publish_tf = config.param<bool>("odometry_estimation", "publish_tf", false);
  saved_map_frame_id = config.param<std::string>("odometry_estimation", "saved_map_frame_id", "saved_map");
  offset_topic = config.param<std::string>("odometry_estimation", "offset_topic", "map_localization");
  localized_odom_topic = config.param<std::string>("odometry_estimation", "localized_odom_topic", "localized_odom");
}

OdometryEstimationLocalizerParams::~OdometryEstimationLocalizerParams() {}

OdometryEstimationLocalizer::OdometryEstimationLocalizer(const OdometryEstimationLocalizerParams& params)
: OdometryEstimationIMU(std::make_unique<OdometryEstimationLocalizerParams>(params)),
  use_gpu(false),
  reloc_pending(false),
  bootstrap_first_stamp(-1.0),
  bootstrapping(false),
  bootstrap_ready(false),
  bootstrap_success(false),
  bootstrapped(false),
  logger(create_module_logger("localizer")) {
  last_T_target_imu.setIdentity();
  reloc_T_map_imu.setIdentity();
  bootstrap_correction.setIdentity();

  if (params.registration_type != "VGICP") {
    logger->warn("only VGICP is supported by the localizer for now; falling back to VGICP (requested '{}')", params.registration_type);
  }

  use_gpu = params.enable_gpu;
#ifndef GTSAM_POINTS_USE_CUDA
  if (use_gpu) {
    logger->warn("enable_gpu=true but gtsam_points was built without CUDA; falling back to the CPU VGICP path");
    use_gpu = false;
  }
#endif

  if (use_gpu) {
#ifdef GTSAM_POINTS_USE_CUDA
    stream.reset(new gtsam_points::CUDAStream());
    stream_buffer_roundrobin.reset(new gtsam_points::StreamTempBufferRoundRobin());
    target_voxelmaps_gpu.resize(params.vgicp_voxelmap_levels);
    for (int i = 0; i < params.vgicp_voxelmap_levels; i++) {
      const double resolution = params.vgicp_resolution * std::pow(params.vgicp_voxelmap_scaling_factor, i);
      target_voxelmaps_gpu[i] = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution);
    }
    logger->info("localizer using GPU VGICP ({} voxelmap levels)", params.vgicp_voxelmap_levels);
#endif
  } else {
    target_voxelmaps.resize(params.vgicp_voxelmap_levels);
    for (int i = 0; i < params.vgicp_voxelmap_levels; i++) {
      const double resolution = params.vgicp_resolution * std::pow(params.vgicp_voxelmap_scaling_factor, i);
      target_voxelmaps[i] = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
    }
    logger->info("localizer using CPU VGICP ({} voxelmap levels)", params.vgicp_voxelmap_levels);
  }

  load_prior_map(params.prior_map_path);

  // Always set up the ROS node: it publishes the map-localization offset, and (when enabled)
  // subscribes to manual relocalization poses.
  setup_ros();

  logger->info("localizer ready");
}

OdometryEstimationLocalizer::~OdometryEstimationLocalizer() {
  ros_running = false;
  if (ros_spin_thread.joinable()) {
    ros_spin_thread.join();
  }
  if (bootstrap_thread.joinable()) {
    bootstrap_thread.join();
  }
  node.reset();
}

void OdometryEstimationLocalizer::load_prior_map(const std::string& path) {
  namespace fs = std::filesystem;

  if (path.empty()) {
    logger->error("prior_map_path is empty; the localizer has no map to localize against. Set odometry_estimation/prior_map_path in the config.");
    return;
  }
  if (!fs::is_directory(path)) {
    logger->error("prior_map_path '{}' is not a directory (expected a GlobalMapping::save dump)", path);
    return;
  }

  const auto localizer_params = static_cast<const OdometryEstimationLocalizerParams*>(this->params.get());

  // In GPU mode we still build one coarse CPU voxelmap purely so the prior map can be visualized.
  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> viz_voxelmap;
  if (use_gpu) {
    const double coarse = localizer_params->vgicp_resolution * std::pow(localizer_params->vgicp_voxelmap_scaling_factor, std::max(0, localizer_params->vgicp_voxelmap_levels - 1));
    viz_voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(coarse);
  }

  std::mt19937 mt;
  size_t total_points = 0;
  int num_submaps = 0;

  // Concatenated world-frame map points, used to build the FPFH bootstrap target.
  std::vector<Eigen::Vector4d> map_points_all;

  // Iterate submap subdirectories 000000, 000001, ... until one fails to load.
  for (int i = 0;; i++) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "/%06d", i);
    const std::string submap_path = path + buffer;
    if (!fs::is_directory(submap_path)) {
      break;
    }

    SubMap::Ptr submap = SubMap::load(submap_path);
    if (!submap || !submap->frame) {
      logger->warn("failed to load submap {}; stopping prior-map load", submap_path);
      break;
    }

    // Optionally downsample the prior points once, then transform into the map/world frame.
    gtsam_points::PointCloud::ConstPtr src = submap->frame;
    if (localizer_params->target_downsampling_rate > 0.0 && localizer_params->target_downsampling_rate < 0.99) {
      src = gtsam_points::random_sampling(submap->frame, localizer_params->target_downsampling_rate, mt);
    }
    auto transformed = gtsam_points::transform(src, submap->T_world_origin);

    if (use_gpu) {
#ifdef GTSAM_POINTS_USE_CUDA
      auto transformed_gpu = gtsam_points::PointCloudGPU::clone(*transformed);
      for (auto& voxelmap : target_voxelmaps_gpu) {
        voxelmap->insert(*transformed_gpu);
      }
      viz_voxelmap->insert(*transformed);
#endif
    } else {
      for (auto& voxelmap : target_voxelmaps) {
        voxelmap->insert(*transformed);
      }
    }

    if (localizer_params->auto_bootstrap) {
      map_points_all.insert(map_points_all.end(), transformed->points, transformed->points + transformed->size());
    }

    total_points += transformed->size();
    num_submaps++;
  }

  if (num_submaps == 0) {
    logger->error("no submaps loaded from '{}'; localization will not work", path);
    return;
  }

  const size_t num_levels = use_gpu ? target_voxelmaps_gpu.size() : target_voxelmaps.size();
  logger->info("loaded prior map: {} submaps, {} target points inserted into {} {} voxelmap level(s)", num_submaps, total_points, num_levels, use_gpu ? "GPU" : "CPU");

  // Build the FPFH/kd-tree target for the automatic global bootstrap.
  if (localizer_params->auto_bootstrap && !map_points_all.empty()) {
    build_bootstrap_target(std::make_shared<gtsam_points::PointCloudCPU>(map_points_all));
  }

  // Publish the coarsest-level voxel centroids as a pseudo-keyframe so viewers can render the prior map.
  auto viz_source = use_gpu ? viz_voxelmap : (target_voxelmaps.empty() ? nullptr : target_voxelmaps.back());
  if (viz_source) {
    EstimationFrame::Ptr frame(new EstimationFrame);
    frame->id = -1;
    frame->stamp = 0.0;
    frame->T_lidar_imu = params->T_lidar_imu;
    frame->T_world_lidar.setIdentity();
    frame->T_world_imu.setIdentity();
    frame->v_world_imu.setZero();
    frame->imu_bias.setZero();
    frame->frame_id = FrameID::WORLD;
    frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(viz_source->voxel_points());
    prior_map_frame = frame;

    std::vector<EstimationFrame::ConstPtr> keyframes = {frame};
    Callbacks::on_update_keyframes(keyframes);
  }
}

static MapAlignerParams make_aligner_params(const OdometryEstimationLocalizerParams* params) {
  MapAlignerParams ap;
  ap.voxel_resolution = params->bootstrap_voxel_resolution;
  ap.fpfh_radius = params->bootstrap_fpfh_radius;
  ap.dof = params->bootstrap_dof;
  ap.search_radius_m = params->bootstrap_search_radius_m;
  ap.min_inlier_rate = params->bootstrap_min_inlier_rate;
  ap.registration = params->bootstrap_registration;
  ap.num_threads = params->num_threads;
  return ap;
}

void OdometryEstimationLocalizer::build_bootstrap_target(const std::shared_ptr<gtsam_points::PointCloudCPU>& map_cloud) {
  const auto params = static_cast<const OdometryEstimationLocalizerParams*>(this->params.get());
  aligner = std::make_shared<MapAligner>(map_cloud, make_aligner_params(params), logger);
}

void OdometryEstimationLocalizer::accumulate_bootstrap_frame(const int current) {
  const auto& frame = frames[current];
  if (!frame || !frame->frame || frame->frame->size() == 0) {
    return;
  }
  if (bootstrap_first_stamp < 0.0) {
    bootstrap_first_stamp = frame->stamp;
  }

  // The matching cloud is in the IMU frame; lift it into the estimator's world frame (IMU-driven,
  // locally consistent even before the map fix) so the buffer is one rigid multi-scan submap.
  const Eigen::Matrix4d T = frame->T_world_imu.matrix();
  const auto* pts = frame->frame->points;
  const size_t n = frame->frame->size();
  const size_t stride = std::max<size_t>(1, n / 20000);  // keep the buffer bounded on dense scans
  bootstrap_accum_points.reserve(bootstrap_accum_points.size() + n / stride + 1);
  for (size_t i = 0; i < n; i += stride) {
    bootstrap_accum_points.emplace_back(T * pts[i]);
  }
}

void OdometryEstimationLocalizer::run_bootstrap(std::vector<Eigen::Vector4d> src_points) {
  const auto params = static_cast<const OdometryEstimationLocalizerParams*>(this->params.get());
  bootstrap_success = false;

  if (aligner && aligner->valid()) {
    const auto result = aligner->align(src_points, make_aligner_params(params));
    if (result.success) {
      bootstrap_correction = result.T_target_source;
      bootstrap_success = true;
      const Eigen::Vector3d t = bootstrap_correction.translation();
      const double yaw = std::atan2(bootstrap_correction.linear()(1, 0), bootstrap_correction.linear()(0, 0));
      logger->info("bootstrap accepted: x={:.2f} y={:.2f} z={:.2f} yaw={:.1f}deg inlier_rate={:.2f}", t.x(), t.y(), t.z(), yaw * 180.0 / M_PI, result.inlier_rate);
    } else {
      logger->warn("bootstrap did not pass the inlier/translation gates; re-accumulating and retrying");
    }
  }
  bootstrap_ready = true;
}

void OdometryEstimationLocalizer::create_frame(EstimationFrame::Ptr& new_frame) {
#ifdef GTSAM_POINTS_USE_CUDA
  // The GPU VGICP factor needs its source cloud on the device.
  if (use_gpu && new_frame->frame && new_frame->frame->size()) {
    new_frame->frame = gtsam_points::PointCloudGPU::clone(*new_frame->frame);
  }
#endif
}

void OdometryEstimationLocalizer::request_relocalization(const Eigen::Isometry3d& T_map_imu) {
  std::lock_guard<std::mutex> lock(reloc_mutex);
  reloc_pending = true;
  reloc_T_map_imu = T_map_imu;
  reloc_T_map_imu.linear() = Eigen::Quaterniond(reloc_T_map_imu.linear()).normalized().toRotationMatrix();
  logger->info(
    "relocalization requested at t=[{:.2f}, {:.2f}, {:.2f}]",
    reloc_T_map_imu.translation().x(),
    reloc_T_map_imu.translation().y(),
    reloc_T_map_imu.translation().z());
}

EstimationFrame::ConstPtr OdometryEstimationLocalizer::insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  // Re-publish the prior map periodically so late-subscribing viewers still get it.
  if (prior_map_frame) {
    static int tick = 0;
    if ((tick++ % 200) == 0) {
      std::vector<EstimationFrame::ConstPtr> keyframes = {prior_map_frame};
      Callbacks::on_update_keyframes(keyframes);
    }
  }
  return OdometryEstimationIMU::insert_frame(frame, marginalized_frames);
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizer::create_factors(const int current, const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) {
  const int last = current - 1;
  const auto localizer_params = static_cast<const OdometryEstimationLocalizerParams*>(this->params.get());

  // Consume a pending relocalization request (set from the ROS thread or the auto-bootstrap worker).
  bool reloc = false;
  Eigen::Isometry3d reloc_pose = Eigen::Isometry3d::Identity();
  {
    std::lock_guard<std::mutex> lock(reloc_mutex);
    if (reloc_pending) {
      reloc = true;
      reloc_pose = reloc_T_map_imu;
      reloc_pending = false;
    }
  }

  // --- Automatic global-localization bootstrap ---
  // Until the map fix is applied, accumulate scans and (once the window elapses) FPFH+RANSAC-register
  // them onto the prior map on a worker thread. Meanwhile contribute no matching factors, so the base
  // smoother dead-reckons on IMU (locally consistent) instead of snapping to a wrong map pose.
  if (localizer_params->auto_bootstrap && aligner && aligner->valid() && !bootstrapped) {
    if (reloc) {
      // A manual /initialpose (or an already-applied fix) takes over; disable the auto-bootstrap.
      bootstrapped = true;
    } else {
      // Apply a finished bootstrap, or clear the window and retry on failure.
      if (bootstrap_ready.load()) {
        if (bootstrap_thread.joinable()) {
          bootstrap_thread.join();
        }
        bootstrapping = false;
        bootstrap_ready = false;
        if (bootstrap_success) {
          const Eigen::Isometry3d T_map_imu = bootstrap_correction * frames[current]->T_world_imu;
          request_relocalization(T_map_imu);  // snapped on the next frame
          bootstrapped = true;
        } else {
          bootstrap_accum_points.clear();
          bootstrap_first_stamp = -1.0;
        }
      }

      // Keep accumulating and launch the worker once the accumulation window has elapsed.
      if (!bootstrapping.load()) {
        accumulate_bootstrap_frame(current);
        if (bootstrap_first_stamp >= 0.0 && (frames[current]->stamp - bootstrap_first_stamp) >= localizer_params->bootstrap_accumulation_secs &&
            !bootstrap_accum_points.empty()) {
          std::vector<Eigen::Vector4d> snapshot;
          snapshot.swap(bootstrap_accum_points);
          bootstrap_first_stamp = -1.0;
          bootstrapping = true;
          if (bootstrap_thread.joinable()) {
            bootstrap_thread.join();
          }
          bootstrap_thread = std::thread([this, s = std::move(snapshot)]() mutable { this->run_bootstrap(std::move(s)); });
        }
      }

      // Dead-reckon on IMU this frame (no map matching yet).
      last_T_target_imu = frames[current]->T_world_imu;
      return gtsam::NonlinearFactorGraph();
    }
  }

  if (current == 0) {
    last_T_target_imu = frames[current]->T_world_imu;
    return gtsam::NonlinearFactorGraph();
  }

  // If the incoming scan has no points, we cannot match: keep the base class's IMU-predicted
  // pose for X(current) (already inserted into the smoother via the IMU factor) and coast.
  if (!frames[current]->frame || frames[current]->frame->size() == 0) {
    logger->warn("empty scan at frame {}; coasting on IMU prediction", current);
    last_T_target_imu = frames[current]->T_world_imu;
    (void)last;
    return gtsam::NonlinearFactorGraph();
  }

  if (use_gpu) {
    return create_factors_gpu(current, new_values, reloc, reloc_pose);
  }
  return create_factors_cpu(current, new_values, reloc, reloc_pose);
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizer::create_factors_cpu(const int current, gtsam::Values& new_values, bool reloc, const Eigen::Isometry3d& reloc_pose) {
  const auto params = static_cast<const OdometryEstimationLocalizerParams*>(this->params.get());
  const int last = current - 1;

  // Seed the local scan-matching optimization. On relocalization, jump the seed to the requested pose.
  Eigen::Isometry3d pred_T_target_imu;
  if (reloc) {
    pred_T_target_imu = reloc_pose;
  } else {
    const Eigen::Isometry3d pred_T_last_current = frames[last]->T_world_imu.inverse() * frames[current]->T_world_imu;
    pred_T_target_imu = last_T_target_imu * pred_T_last_current;
  }

  gtsam::Values values;
  values.insert(X(current), gtsam::Pose3(pred_T_target_imu.matrix()));

  // Frame-to-fixed-prior-map VGICP factors (target is the pre-loaded map, never updated).
  gtsam::NonlinearFactorGraph matching_cost_factors;
  for (const auto& voxelmap : target_voxelmaps) {
    auto vgicp_factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), X(current), voxelmap, frames[current]->frame);
    vgicp_factor->set_num_threads(params->num_threads);
    matching_cost_factors.add(vgicp_factor);
  }

  gtsam::NonlinearFactorGraph graph;
  graph.add(matching_cost_factors);

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(params->max_iterations);
  lm_params.setAbsoluteErrorTol(0.1);

  gtsam::Pose3 last_estimate = values.at<gtsam::Pose3>(X(current));
  lm_params.termination_criteria = [&](const gtsam::Values& values) {
    const gtsam::Pose3 current_pose = values.at<gtsam::Pose3>(X(current));
    const gtsam::Pose3 delta = last_estimate.inverse() * current_pose;

    const double delta_t = delta.translation().norm();
    const double delta_r = Eigen::AngleAxisd(delta.rotation().matrix()).angle();
    last_estimate = current_pose;

    if (delta_t < 1e-10 && delta_r < 1e-10) {
      return false;
    }
    return delta_t < 1e-3 && delta_r < 1e-3 * M_PI / 180.0;
  };

  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);

#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(this->tbb_task_arena.get());
  arena->execute([&] {
#endif
    values = optimizer.optimize();
#ifdef GTSAM_USE_TBB
  });
#endif

  const Eigen::Isometry3d T_target_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(current)).matrix());
  Eigen::Isometry3d T_last_current = last_T_target_imu.inverse() * T_target_imu;
  T_last_current.linear() = Eigen::Quaterniond(T_last_current.linear()).normalized().toRotationMatrix();
  frames[current]->T_world_imu = frames[last]->T_world_imu * T_last_current;
  new_values.insert_or_assign(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));

  gtsam::NonlinearFactorGraph factors;

  if (reloc) {
    // Snap the trajectory onto the requested pose: strong absolute prior on X(current),
    // and skip the (now meaningless) relative factor to the pre-relocalization frame.
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), gtsam::Pose3(T_target_imu.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e6));
    logger->info("relocalization applied at frame {}", current);
  } else {
    // Inject the frame-to-map matching result into the smoother, mirroring OdometryEstimationCPU.
    factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      X(last),
      X(current),
      gtsam::Pose3(T_last_current.matrix()),
      gtsam::noiseModel::Isotropic::Precision(6, 1e3));
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), gtsam::Pose3(T_target_imu.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));
  }

  last_T_target_imu = T_target_imu;

  return factors;
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizer::create_factors_gpu(const int current, gtsam::Values& new_values, bool reloc, const Eigen::Isometry3d& reloc_pose) {
  gtsam::NonlinearFactorGraph factors;

#ifdef GTSAM_POINTS_USE_CUDA
  if (reloc) {
    // Jump the state to the requested pose and pin it hard; the VGICP factors below refine it.
    new_values.insert_or_assign(X(current), gtsam::Pose3(reloc_pose.matrix()));
    frames[current]->T_world_imu = reloc_pose;
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), gtsam::Pose3(reloc_pose.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e6));
    logger->info("relocalization applied at frame {}", current);
  }

  // Tightly-coupled frame-to-fixed-prior-map VGICP factors, added directly to the smoother.
  // The prior-map voxelmaps are already in the world/map frame, so the fixed target pose is identity.
  for (const auto& voxelmap : target_voxelmaps_gpu) {
    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;

    auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(gtsam::Pose3(), X(current), voxelmap, frames[current]->frame, stream, buffer);
    factor->set_enable_surface_validation(true);
    factors.add(factor);
  }
#else
  (void)current;
  (void)new_values;
  (void)reloc;
  (void)reloc_pose;
#endif

  return factors;
}

void OdometryEstimationLocalizer::fallback_smoother() {}

}  // namespace glim
