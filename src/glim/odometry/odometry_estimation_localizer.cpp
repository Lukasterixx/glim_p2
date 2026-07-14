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
#include <gtsam_points/features/covariance_estimation.hpp>
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

  enable_live_mapping = config.param<bool>("odometry_estimation", "enable_live_mapping", true);
  live_map_resolution = config.param<double>("odometry_estimation", "live_map_resolution", vgicp_resolution);
  live_map_lru_thresh = config.param<int>("odometry_estimation", "live_map_lru_thresh", 75);
  live_map_downsampling_rate = config.param<double>("odometry_estimation", "live_map_downsampling_rate", 0.5);
  live_map_gap_only = config.param<bool>("odometry_estimation", "live_map_gap_only", true);
  live_map_gap_min_points = config.param<int>("odometry_estimation", "live_map_gap_min_points", 10);

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
  bootstrap_refine = config.param<bool>("odometry_estimation", "bootstrap_refine", true);
  bootstrap_refine_max_correction = config.param<double>("odometry_estimation", "bootstrap_refine_max_correction", 1.0);
  bootstrap_max_z_offset = config.param<double>("odometry_estimation", "bootstrap_max_z_offset", -1.0);

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
  T_map_world.setIdentity();

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

  // CPU multi-resolution voxelmaps are always built: they are the matching target on the CPU
  // path, and in GPU mode they serve the bootstrap fine-refinement, the live-map gap check,
  // and the prior-map visualization.
  target_voxelmaps.resize(params.vgicp_voxelmap_levels);
  for (int i = 0; i < params.vgicp_voxelmap_levels; i++) {
    const double resolution = params.vgicp_resolution * std::pow(params.vgicp_voxelmap_scaling_factor, i);
    target_voxelmaps[i] = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
  }

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
    logger->info("localizer using CPU VGICP ({} voxelmap levels)", params.vgicp_voxelmap_levels);
  }

  // Live rolling local map (both CPU and GPU paths): a single LRU-bounded CPU voxelmap fed by the
  // live scans, matched alongside the fixed prior map so scan-matching stays stable where the prior
  // is too sparse. It stays on the CPU even in GPU mode because GPU voxelmaps cannot be updated
  // incrementally (no LRU); the small gap-filtered CPU target is cheap to match with a CPU factor.
  if (params.enable_live_mapping) {
    live_voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(params.live_map_resolution);
    live_voxelmap->set_lru_horizon(params.live_map_lru_thresh);
    logger->info(
      "localizer live mapping enabled (resolution={:.2f}m, lru={} inserts, downsample={:.2f})",
      params.live_map_resolution,
      params.live_map_lru_thresh,
      params.live_map_downsampling_rate);
  }

  load_prior_map(params.prior_map_path);

  // With no auto-bootstrap (or no usable aligner target), the configured init pose is trusted:
  // the estimator world frame IS the map frame (offset = identity) and publishing starts at once.
  if (!params.auto_bootstrap || !aligner || !aligner->valid()) {
    bootstrapped = true;
  }

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

    // CPU voxelmaps are always populated (matching target on CPU; refine/gap-check/viz on GPU).
    for (auto& voxelmap : target_voxelmaps) {
      voxelmap->insert(*transformed);
    }
    if (use_gpu) {
#ifdef GTSAM_POINTS_USE_CUDA
      auto transformed_gpu = gtsam_points::PointCloudGPU::clone(*transformed);
      for (auto& voxelmap : target_voxelmaps_gpu) {
        voxelmap->insert(*transformed_gpu);
      }
#endif
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

  logger->info(
    "loaded prior map: {} submaps, {} target points inserted into {} CPU voxelmap level(s){}",
    num_submaps,
    total_points,
    target_voxelmaps.size(),
    use_gpu ? " (+GPU copies)" : "");

  // Build the FPFH/kd-tree target for the automatic global bootstrap.
  if (localizer_params->auto_bootstrap && !map_points_all.empty()) {
    build_bootstrap_target(std::make_shared<gtsam_points::PointCloudCPU>(map_points_all));
  }

  // Keep the coarsest-level voxel centroids (in map coordinates) as a pseudo-keyframe; it is
  // published posed at T_map_world^-1 so viewers render the prior map aligned with the estimator
  // frame both before and after the offset is defined.
  auto viz_source = target_voxelmaps.empty() ? nullptr : target_voxelmaps.back();
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

    publish_prior_map_viz();
  }
}

void OdometryEstimationLocalizer::publish_prior_map_viz() {
  if (!prior_map_frame) {
    return;
  }
  // Share the (map-frame) point cloud; only the pose differs. FrameID::IMU makes viewers apply
  // T_world_imu = T_map_world^-1, i.e. render the map in the estimator world frame.
  EstimationFrame::Ptr posed = prior_map_frame->clone();
  posed->frame_id = FrameID::IMU;
  posed->T_world_imu = T_map_world.inverse();
  posed->T_world_lidar = posed->T_world_imu;

  std::vector<EstimationFrame::ConstPtr> keyframes = {posed};
  Callbacks::on_update_keyframes(keyframes);
}

void OdometryEstimationLocalizer::apply_map_offset(const Eigen::Isometry3d& T_map_world_new, const char* source) {
  T_map_world = T_map_world_new;
  T_map_world.linear() = Eigen::Quaterniond(T_map_world.linear()).normalized().toRotationMatrix();
  bootstrapped = true;

  // Offset-dependent state built under the old (or no) offset is stale:
  // the live map was gap-filtered against wrong prior regions, and the bootstrap local model is done.
  if (live_voxelmap) {
    live_voxelmap->clear();
  }
  bootstrap_local_voxelmap.reset();

  publish_prior_map_viz();

  const Eigen::Vector3d t = T_map_world.translation();
  const double yaw = std::atan2(T_map_world.linear()(1, 0), T_map_world.linear()(0, 0));
  logger->info("map offset set by {}: t=[{:.2f}, {:.2f}, {:.2f}] yaw={:.1f}deg", source, t.x(), t.y(), t.z(), yaw * 180.0 / M_PI);
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
      // FPFH+RANSAC is coarse (tens of cm / a few deg on voxelized features). Fine-register with
      // VGICP before accepting: the offset is installed once and must be right from frame one.
      Eigen::Isometry3d correction = result.T_target_source;
      bool ok = true;

      // Fixed-start-height constraint: the LiDAR boots at the same height the map was recorded at
      // and both frames are gravity-aligned, so the true offset has z ~= 0. A large z is a
      // misregistration (e.g. matched onto the wrong floor/structure) -> reject. Otherwise pin
      // z = 0 and let the fine refinement settle the exact value.
      if (params->bootstrap_max_z_offset >= 0.0) {
        const double z = correction.translation().z();
        if (std::abs(z) > params->bootstrap_max_z_offset) {
          logger->warn("bootstrap rejected: |z offset| {:.2f}m exceeds bootstrap_max_z_offset {:.2f}m", std::abs(z), params->bootstrap_max_z_offset);
          ok = false;
        } else {
          correction.translation().z() = 0.0;
        }
      }

      if (ok && params->bootstrap_refine) {
        ok = refine_bootstrap_alignment(src_points, correction);
      }

      if (ok) {
        bootstrap_correction = correction;
        bootstrap_success = true;
        const Eigen::Vector3d t = bootstrap_correction.translation();
        const double yaw = std::atan2(bootstrap_correction.linear()(1, 0), bootstrap_correction.linear()(0, 0));
        logger->info("bootstrap accepted: x={:.2f} y={:.2f} z={:.2f} yaw={:.1f}deg inlier_rate={:.2f}", t.x(), t.y(), t.z(), yaw * 180.0 / M_PI, result.inlier_rate);
      } else {
        logger->warn("bootstrap refinement rejected the coarse pose; re-accumulating and retrying");
      }
    } else {
      logger->warn("bootstrap did not pass the inlier/translation gates; re-accumulating and retrying");
    }
  }
  bootstrap_ready = true;
}

bool OdometryEstimationLocalizer::refine_bootstrap_alignment(const std::vector<Eigen::Vector4d>& src_points, Eigen::Isometry3d& T_map_world_coarse) {
  const auto params = static_cast<const OdometryEstimationLocalizerParams*>(this->params.get());

  // Reuses the CPU prior-map voxelmaps (read-only; built in both CPU and GPU modes).
  if (target_voxelmaps.empty()) {
    logger->warn("bootstrap refine skipped (prior-map voxelmaps unavailable)");
    return true;
  }

  auto src = std::make_shared<gtsam_points::PointCloudCPU>(src_points);
  auto downsampled = gtsam_points::voxelgrid_sampling(src, params->bootstrap_voxel_resolution, params->num_threads);
  if (!downsampled || downsampled->size() < 100) {
    logger->warn("bootstrap refine: accumulated cloud too small after downsampling");
    return false;
  }
  downsampled->add_covs(gtsam_points::estimate_covariances(downsampled->points, downsampled->size(), 10, params->num_threads));

  // One pose variable: T_map_world. Prior-map voxelmaps are in the map frame (fixed identity
  // target pose); the source cloud is in the estimator world frame.
  gtsam::Values values;
  values.insert(gtsam::Key(0), gtsam::Pose3(T_map_world_coarse.matrix()));

  gtsam::NonlinearFactorGraph graph;
  std::vector<const gtsam_points::IntegratedVGICPFactor*> vgicp_factors;  // owned by `graph`
  for (const auto& voxelmap : target_voxelmaps) {
    auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), gtsam::Key(0), voxelmap, downsampled);
    factor->set_num_threads(params->num_threads);
    graph.add(factor);
    vgicp_factors.push_back(factor.get());
  }

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(30);
  lm_params.setAbsoluteErrorTol(1e-3);

  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
  values = optimizer.optimize();

  const Eigen::Isometry3d refined(values.at<gtsam::Pose3>(gtsam::Key(0)).matrix());
  const Eigen::Isometry3d delta = T_map_world_coarse.inverse() * refined;
  const double delta_t = delta.translation().norm();
  const double delta_r = Eigen::AngleAxisd(delta.linear()).angle();
  const double inlier_rate = vgicp_factors.front()->inlier_fraction();  // finest level, at the solution

  logger->info("bootstrap refine: moved {:.2f}m / {:.1f}deg from the coarse pose, inlier_rate={:.2f}", delta_t, delta_r * 180.0 / M_PI, inlier_rate);

  if (delta_t > params->bootstrap_refine_max_correction || delta_r > 15.0 * M_PI / 180.0) {
    logger->warn("bootstrap refine: correction exceeds the trust gate ({:.2f}m / {:.1f}deg)", delta_t, delta_r * 180.0 / M_PI);
    return false;
  }
  if (inlier_rate < params->bootstrap_min_inlier_rate) {
    logger->warn("bootstrap refine: inlier rate {:.2f} below gate {:.2f}", inlier_rate, params->bootstrap_min_inlier_rate);
    return false;
  }

  T_map_world_coarse = refined;
  return true;
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
      publish_prior_map_viz();
    }
  }
  return OdometryEstimationIMU::insert_frame(frame, marginalized_frames);
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizer::create_factors(const int current, const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) {
  const int last = current - 1;
  const auto localizer_params = static_cast<const OdometryEstimationLocalizerParams*>(this->params.get());

  // Consume a pending relocalization request (set from the ROS thread). The smoother is never
  // snapped: the request only redefines the saved-map<-world offset so the CURRENT pose maps to
  // the requested map pose. Prior-map matching pulls it into fine alignment from there.
  {
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
    if (reloc && frames[current]) {
      apply_map_offset(reloc_pose * frames[current]->T_world_imu.inverse(), "initialpose");
    }
  }

  // --- Automatic global-localization bootstrap ---
  // Until the map offset is known, accumulate scans and (once the window elapses) FPFH+RANSAC-register
  // them onto the prior map on a worker thread. Meanwhile contribute no PRIOR-MAP matching factors
  // (matching through an unknown offset would be meaningless), but DO keep the trajectory LiDAR-tight by
  // matching each scan against a local voxelmap of the previous scans: IMU-only dead-reckoning drifts
  // quadratically with any attitude/bias error and smears the accumulated bootstrap submap.
  if (localizer_params->auto_bootstrap && aligner && aligner->valid() && !bootstrapped) {
    // Install a finished bootstrap (falls through to normal prior-map matching below), or clear the
    // window and retry on failure.
    if (bootstrap_ready.load()) {
      if (bootstrap_thread.joinable()) {
        bootstrap_thread.join();
      }
      bootstrapping = false;
      bootstrap_ready = false;
      if (bootstrap_success) {
        apply_map_offset(bootstrap_correction, "auto-bootstrap");
      } else {
        bootstrap_accum_points.clear();
        bootstrap_first_stamp = -1.0;
      }
    }

    if (!bootstrapped) {
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

      gtsam::NonlinearFactorGraph bootstrap_factors;

      // Fold the previous frame (whose pose the smoother has already refined) into the local model.
      if (last >= 0 && last > bootstrap_last_inserted && frames[last] && frames[last]->frame && frames[last]->frame->size()) {
        if (!bootstrap_local_voxelmap) {
          bootstrap_local_voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(localizer_params->vgicp_resolution);
        }
        auto transformed = gtsam_points::transform(frames[last]->frame, frames[last]->T_world_imu);
        bootstrap_local_voxelmap->insert(*transformed);
        bootstrap_last_inserted = last;
      }

      // Scan-to-local-model VGICP, jointly optimized with the IMU factor (same tightly-coupled
      // structure as the prior-map matching, but against the local model; world frame stays arbitrary).
      if (bootstrap_local_voxelmap && frames[current]->frame && frames[current]->frame->size()) {
        auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), X(current), bootstrap_local_voxelmap, frames[current]->frame);
        factor->set_num_threads(localizer_params->num_threads);
        bootstrap_factors.add(factor);
      }

      last_T_target_imu = frames[current]->T_world_imu;
      return bootstrap_factors;
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
    return create_factors_gpu(current, new_values);
  }
  return create_factors_cpu(current, new_values);
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizer::create_factors_cpu(const int current, gtsam::Values& new_values) {
  const auto params = static_cast<const OdometryEstimationLocalizerParams*>(this->params.get());
  const int last = current - 1;

  // Seed the local scan-matching optimization from the constant-velocity prediction. All poses
  // here live in the estimator's continuous world frame; the map offset enters only as the fixed
  // pose of the prior-map target below, so nothing ever jumps.
  const Eigen::Isometry3d pred_T_last_current = frames[last]->T_world_imu.inverse() * frames[current]->T_world_imu;
  const Eigen::Isometry3d pred_T_target_imu = last_T_target_imu * pred_T_last_current;

  gtsam::Values values;
  values.insert(X(current), gtsam::Pose3(pred_T_target_imu.matrix()));

  // Frame-to-fixed-prior-map VGICP factors. The prior map (map frame) is posed into the estimator
  // world frame with the fixed target pose T_world_map = T_map_world^-1; it is never a variable.
  const gtsam::Pose3 T_world_map(T_map_world.inverse().matrix());
  gtsam::NonlinearFactorGraph matching_cost_factors;
  for (const auto& voxelmap : target_voxelmaps) {
    auto vgicp_factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(T_world_map, X(current), voxelmap, frames[current]->frame);
    vgicp_factor->set_num_threads(params->num_threads);
    matching_cost_factors.add(vgicp_factor);
  }

  // Additional frame-to-live-local-map VGICP factor. The live map is a short rolling buffer of
  // recent scans holding (in gap-only mode) ONLY geometry the prior map lacks, so scan points in
  // prior-covered regions find no live voxels and the prior fully owns the solve there; in prior
  // gaps the live map supplies the correspondences that keep the fit stable. VGICP cost scales
  // with correspondence count, so without the gap-only insert filter this factor would out-vote
  // the sparse prior everywhere and drag the estimate into self-reinforcing drift.
  gtsam_points::PointCloud::ConstPtr live_src;
  if (live_voxelmap && live_voxelmap->num_voxels() > 0) {
    live_src = frames[current]->frame;
    if (params->live_map_downsampling_rate > 0.0 && params->live_map_downsampling_rate < 0.99) {
      live_src = gtsam_points::random_sampling(frames[current]->frame, params->live_map_downsampling_rate, live_map_mt);
    }
    if (live_src && live_src->size() > 0) {
      auto live_factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), X(current), live_voxelmap, live_src);
      live_factor->set_num_threads(params->num_threads);
      matching_cost_factors.add(live_factor);
    }
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

  // Inject the frame-to-map matching result into the smoother, mirroring OdometryEstimationCPU.
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
    X(last),
    X(current),
    gtsam::Pose3(T_last_current.matrix()),
    gtsam::noiseModel::Isotropic::Precision(6, 1e3));
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), gtsam::Pose3(T_target_imu.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));

  last_T_target_imu = T_target_imu;

  // Fold this scan into the rolling live map at its solved map pose (LRU drops old scans).
  update_live_map(current, T_target_imu);

  return factors;
}

void OdometryEstimationLocalizer::update_live_map(const int frame_index, const Eigen::Isometry3d& T_world_frame) {
  if (!live_voxelmap) {
    return;
  }
  const auto params = static_cast<const OdometryEstimationLocalizerParams*>(this->params.get());
  gtsam_points::PointCloud::ConstPtr frame = frames[frame_index]->frame;
  if (!frame || frame->size() == 0) {
    return;
  }
  if (params->live_map_downsampling_rate > 0.0 && params->live_map_downsampling_rate < 0.99) {
    frame = gtsam_points::random_sampling(frame, params->live_map_downsampling_rate, live_map_mt);
  }
  gtsam_points::PointCloud::ConstPtr transformed = gtsam_points::transform(frame, T_world_frame);

  // Gap-only insertion: keep only points falling where the finest prior-map level is thin
  // (< live_map_gap_min_points in the voxel). VGICP cost scales with the number of
  // correspondences, so a live map that overlaps well-covered prior regions out-votes the
  // prior and drags the estimate into a self-reinforcing drift loop. By construction the
  // live map then only holds geometry the prior lacks: no live voxels where the prior is
  // good -> no live correspondences there -> the prior fully owns the solve.
  // The prior voxels live in the map frame while `transformed` is in the estimator world
  // frame, so the lookup goes through the map offset.
  if (params->live_map_gap_only && !target_voxelmaps.empty()) {
    const auto& prior = target_voxelmaps.front();
    const size_t min_points = params->live_map_gap_min_points > 0 ? params->live_map_gap_min_points : 0;
    const Eigen::Matrix4d M_map_world = T_map_world.matrix();
    transformed = gtsam_points::filter(transformed, [&](const Eigen::Vector4d& pt) {
      const int voxel_id = prior->lookup_voxel_index(prior->voxel_coord(M_map_world * pt));
      return voxel_id < 0 || prior->lookup_voxel(voxel_id).num_points < min_points;
    });
  }

  if (transformed->size() == 0) {
    return;
  }
  live_voxelmap->insert(*transformed);

  if ((frame_index % 100) == 0) {
    logger->debug("live map: inserted {} gap points this frame, {} voxels total", transformed->size(), live_voxelmap->num_voxels());
  }
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizer::create_factors_gpu(const int current, gtsam::Values& new_values) {
  gtsam::NonlinearFactorGraph factors;

#ifdef GTSAM_POINTS_USE_CUDA
  const auto params = static_cast<const OdometryEstimationLocalizerParams*>(this->params.get());
  const int last = current - 1;

  // Fold the previous (smoother-optimized) frame into the rolling live map before adding this
  // frame's factors. GPU voxelmaps have no incremental/LRU insertion, so the live map stays a CPU
  // GaussianVoxelMapCPU and is matched with a CPU factor below; the GPU source cloud keeps its CPU
  // points+covs, so the CPU-side ops work unchanged. (CPU path folds the current frame post-solve;
  // here we fold last, whose pose the smoother has already finalized -- a 1-frame lag, harmless.)
  if (live_voxelmap && last >= 0 && frames[last] && frames[last]->frame && frames[last]->frame->size()) {
    update_live_map(last, frames[last]->T_world_imu);
  }

  // Tightly-coupled frame-to-fixed-prior-map VGICP factors (GPU), added directly to the smoother.
  // The prior-map voxelmaps are in the map frame; pose them into the estimator world frame with
  // the fixed target pose T_world_map (relocalization only changes this pose, never the states).
  const gtsam::Pose3 T_world_map(T_map_world.inverse().matrix());
  for (const auto& voxelmap : target_voxelmaps_gpu) {
    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;

    auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(T_world_map, X(current), voxelmap, frames[current]->frame, stream, buffer);
    factor->set_enable_surface_validation(true);
    factors.add(factor);
  }

  // Additional frame-to-live-local-map VGICP factor (CPU target in the estimator world frame, so a
  // fixed identity target pose). Tightly coupled alongside the GPU prior factors. Gap-only insertion
  // keeps the (small) live map from competing with the prior; it only fills geometry the prior lacks.
  // The sampled source (covs preserved) is owned by the factor, so it outlives this call.
  if (live_voxelmap && live_voxelmap->num_voxels() > 0 && frames[current]->frame && frames[current]->frame->size()) {
    gtsam_points::PointCloud::ConstPtr live_src = frames[current]->frame;
    if (params->live_map_downsampling_rate > 0.0 && params->live_map_downsampling_rate < 0.99) {
      live_src = gtsam_points::random_sampling(frames[current]->frame, params->live_map_downsampling_rate, live_map_mt);
    }
    if (live_src && live_src->size() > 0) {
      auto live_factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), X(current), live_voxelmap, live_src);
      live_factor->set_num_threads(params->num_threads);
      factors.add(live_factor);
    }
  }
#else
  (void)current;
  (void)new_values;
#endif

  return factors;
}

void OdometryEstimationLocalizer::fallback_smoother() {}

}  // namespace glim
