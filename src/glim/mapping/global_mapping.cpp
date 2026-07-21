#include <glim/mapping/global_mapping.hpp>

#include <cmath>
#include <map>
#include <unordered_set>
#include <spdlog/spdlog.h>
#include <boost/filesystem.hpp>

#include <gtsam/base/serialization.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseRotationPrior.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/config.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/rotate_vector3_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/isam2_ext_dummy.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>

#include <glim/util/config.hpp>
#include <glim/util/serialization.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/reloc_override.hpp>
#include <glim/odometry/map_alignment.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

using Callbacks = GlobalMappingCallbacks;

GlobalMappingParams::GlobalMappingParams() {
  Config config(GlobalConfig::get_config_path("config_global_mapping"));

  enable_imu = config.param<bool>("global_mapping", "enable_imu", true);
  enable_optimization = config.param<bool>("global_mapping", "enable_optimization", true);

  enable_between_factors = config.param<bool>("global_mapping", "create_between_factors", false);
  between_registration_type = config.param<std::string>("global_mapping", "between_registration_type", "GICP");
  registration_error_factor_type = config.param<std::string>("global_mapping", "registration_error_factor_type", "VGICP");
  submap_voxel_resolution = config.param<double>("global_mapping", "submap_voxel_resolution", 1.0);
  submap_voxel_resolution_max = config.param<double>("global_mapping", "submap_voxel_resolution_max", submap_voxel_resolution);
  submap_voxel_resolution_dmin = config.param<double>("global_mapping", "submap_voxel_resolution_dmin", 5.0);
  submap_voxel_resolution_dmax = config.param<double>("global_mapping", "submap_voxel_resolution_dmax", 20.0);

  submap_voxelmap_levels = config.param<int>("global_mapping", "submap_voxelmap_levels", 2);
  submap_voxelmap_scaling_factor = config.param<double>("global_mapping", "submap_voxelmap_scaling_factor", 2.0);

  randomsampling_rate = config.param<double>("global_mapping", "randomsampling_rate", 1.0);
  max_implicit_loop_distance = config.param<double>("global_mapping", "max_implicit_loop_distance", 100.0);
  min_implicit_loop_overlap = config.param<double>("global_mapping", "min_implicit_loop_overlap", 0.1);

  enable_gpu = registration_error_factor_type.find("GPU") != std::string::npos;

  use_isam2_dogleg = config.param<bool>("global_mapping", "use_isam2_dogleg", false);
  isam2_relinearize_skip = config.param<int>("global_mapping", "isam2_relinearize_skip", 1);
  isam2_relinearize_thresh = config.param<double>("global_mapping", "isam2_relinearize_thresh", 0.1);

  init_pose_damping_scale = config.param<double>("global_mapping", "init_pose_damping_scale", 1e10);

  // Session continuation
  continue_from_map_path = config.param<std::string>("global_mapping", "continue_from_map_path", "");

  // Auto-continue: when no prior-map path is configured, resume from the default shutdown dump
  // (/tmp/dump) whenever a valid dump is present there — i.e. GLIM "starts where it left off"
  // automatically. A valid dump is one that GlobalMapping::load() can open, whose gate is the
  // presence of graph.txt (see load() below). To force a fresh mapping run, delete the dump
  // (the web dump-browser exposes this). Setting continue_from_map_path explicitly still wins.
  // (In sim mode the prior map is still loaded and the relocalization is computed + reported, but
  // the session is NOT re-anchored onto it — see apply_relocalization below.)
  if (continue_from_map_path.empty()) {
    const std::string auto_dump = "/tmp/dump";
    boost::system::error_code ec;
    if (boost::filesystem::exists(auto_dump + "/graph.txt", ec)) {
      continue_from_map_path = auto_dump;
      spdlog::info("global_mapping: auto-continue from existing dump at {} (delete the dump to map fresh)", auto_dump);
    }
  }

  // Sim mode (glim_ros/publish_tf=false in config_ros) => compute + report the relocalization but do
  // NOT apply it. On the real robot (publish_tf=true, or unset) apply it normally (re-anchor + TF).
  apply_relocalization = Config(GlobalConfig::get_config_path("config_ros")).param<bool>("glim_ros", "publish_tf", true);

  save_map_path = config.param<std::string>("global_mapping", "save_map_path", "");
  freeze_loaded_map = config.param<bool>("global_mapping", "freeze_loaded_map", true);
  freeze_prior_precision = config.param<double>("global_mapping", "freeze_prior_precision", 1e10);
  reloc_voxel_resolution = config.param<double>("global_mapping", "reloc_voxel_resolution", 0.5);
  reloc_fpfh_radius = config.param<double>("global_mapping", "reloc_fpfh_radius", 2.5);
  reloc_dof = config.param<int>("global_mapping", "reloc_dof", 4);
  reloc_search_radius_m = config.param<double>("global_mapping", "reloc_search_radius_m", 1000.0);
  reloc_start_area_radius_m = config.param<double>("global_mapping", "reloc_start_area_radius_m", 0.0);
  reloc_min_inlier_rate = config.param<double>("global_mapping", "reloc_min_inlier_rate", 0.3);
  reloc_registration = config.param<std::string>("global_mapping", "reloc_registration", "RANSAC");
  num_threads = config.param<int>("global_mapping", "num_threads", 4);

  reloc_method = config.param<std::string>("global_mapping", "reloc_method", "yaw_sweep");
  reloc_yaw_step_deg = config.param<double>("global_mapping", "reloc_yaw_step_deg", 15.0);
  reloc_max_submaps = config.param<int>("global_mapping", "reloc_max_submaps", 10);
  reloc_refine = config.param<bool>("global_mapping", "reloc_refine", true);
  reloc_refine_max_correction = config.param<double>("global_mapping", "reloc_refine_max_correction", 1.0);
  reloc_refine_min_inlier_rate = config.param<double>("global_mapping", "reloc_refine_min_inlier_rate", 0.6);
}

GlobalMappingParams::~GlobalMappingParams() {}

GlobalMapping::GlobalMapping(const GlobalMappingParams& params) : params(params) {
#ifndef GTSAM_POINTS_USE_CUDA
  if (params.enable_gpu) {
    logger->error("GPU-based factors cannot be used because GLIM is built without GPU option!!");
  }
#endif

  session_id = 0;
  imu_integration.reset(new IMUIntegration);

  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  gtsam::ISAM2Params isam2_params;
  if (params.use_isam2_dogleg) {
    gtsam::ISAM2DoglegParams dogleg_params;
    isam2_params.setOptimizationParams(dogleg_params);
  }
  isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);

  if (params.enable_optimization) {
    isam2.reset(new gtsam_points::ISAM2Ext(isam2_params));
  } else {
    isam2.reset(new gtsam_points::ISAM2ExtDummy(isam2_params));
  }

#ifdef GTSAM_POINTS_USE_CUDA
  stream_buffer_roundrobin = std::make_shared<gtsam_points::StreamTempBufferRoundRobin>(64);
#endif

#ifdef GTSAM_USE_TBB
  tbb_task_arena = std::make_shared<tbb::task_arena>(1);
#endif
}

GlobalMapping::~GlobalMapping() {}

void GlobalMapping::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  ensure_prior_map_loaded();
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
  if (params.enable_imu) {
    imu_integration->insert_imu(stamp, linear_acc, angular_vel);
  }
}

void GlobalMapping::ensure_prior_map_loaded() {
  if (prior_map_load_done) {
    return;
  }
  prior_map_load_done = true;  // set first so the callbacks fired by load() cannot re-enter this

  if (params.continue_from_map_path.empty()) {
    return;
  }

  logger->info("continuation: loading prior map from {}", params.continue_from_map_path);
  if (!load(params.continue_from_map_path)) {
    logger->error("continuation: failed to load prior map from {}; starting a fresh map instead", params.continue_from_map_path);
    return;
  }

  // The next submap to arrive becomes the first NEW submap of this session.
  continuation_start_id = submaps.size();

  // A continuation was attempted: save() redirects to the continuation output folder from here on,
  // even if we later fall back to fresh mapping (so the fresh map still lands beside the loaded one).
  continuation_save_redirect = true;

  // Build the FPFH relocalization target.
  MapAlignerParams ap;
  ap.voxel_resolution = params.reloc_voxel_resolution;
  ap.fpfh_radius = params.reloc_fpfh_radius;
  ap.dof = params.reloc_dof;
  ap.search_radius_m = params.reloc_search_radius_m;
  ap.min_inlier_rate = params.reloc_min_inlier_rate;
  ap.registration = params.reloc_registration;
  ap.num_threads = params.num_threads;

  // Full concatenated map cloud (map frame). Used for the VGICP target below and, for the FPFH
  // method, as the source of the coarse-registration features.
  auto full_map_cloud = MapAligner::load_map_cloud(params.continue_from_map_path, logger);

  // Map-frame VGICP target (two resolution levels), built from the WHOLE map. This is the fine-fit
  // target for both the yaw-sweep relocalizer and the FPFH refine. It MUST be the whole map, not the
  // start area: the LiDAR sees tens of metres and the accumulated source spans the retraced route, so
  // a start-area-only target leaves most source points without a correspondence and makes the inlier
  // fraction meaningless (it falls as the robot drives away — the bug behind the persistent failures).
  if (full_map_cloud && (params.reloc_method == "yaw_sweep" || params.reloc_refine)) {
    auto cov_cloud = gtsam_points::voxelgrid_sampling(full_map_cloud, params.reloc_voxel_resolution * 0.5, params.num_threads);
    cov_cloud->add_covs(gtsam_points::estimate_covariances(cov_cloud->points, cov_cloud->size(), 10, params.num_threads));
    reloc_refine_voxelmaps.clear();
    for (int i = 0; i < 2; i++) {
      const double resolution = params.reloc_voxel_resolution * std::pow(2.0, i);
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
      voxelmap->insert(*cov_cloud);
      reloc_refine_voxelmaps.push_back(voxelmap);
    }
    logger->info("continuation: VGICP relocalization target ready ({} map points, {} levels)", cov_cloud->size(), reloc_refine_voxelmaps.size());

#ifdef GTSAM_POINTS_USE_CUDA
    // GPU mirror of the same target: the yaw-sweep runs dozens of VGICP fits, so pushing each fit onto
    // the GPU turns a multi-minute, CPU-bound (num_threads-capped) sweep into a GPU-bound one — the win
    // that matters on the Jetson, which has a strong GPU but few CPU cores. The GPU voxelmap wants a
    // GPU-side source, so clone the cov cloud up once and reuse it for every level.
    if (params.enable_gpu) {
      auto cov_cloud_gpu = gtsam_points::PointCloudGPU::clone(*cov_cloud);
      reloc_refine_voxelmaps_gpu.clear();
      for (int i = 0; i < 2; i++) {
        const double resolution = params.reloc_voxel_resolution * std::pow(2.0, i);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution);
        voxelmap->insert(*cov_cloud_gpu);
        reloc_refine_voxelmaps_gpu.push_back(voxelmap);
      }
      logger->info("continuation: GPU VGICP relocalization target ready ({} levels)", reloc_refine_voxelmaps_gpu.size());
    }
#endif
  }

  // FPFH coarse-registration target (only for the "fpfh" method). If a start-area radius is set, build
  // it from ONLY the map points near the START of the prior run to remove whole-map self-similarity
  // that dilutes FPFH correspondences; otherwise use the whole map.
  if (params.reloc_method == "fpfh") {
    std::shared_ptr<gtsam_points::PointCloudCPU> fpfh_cloud;
    if (params.reloc_start_area_radius_m > 0.0 && !submaps.empty() && submaps.front()->frame) {
      const Eigen::Vector3d start = submaps.front()->T_world_origin.translation();
      const double r2 = params.reloc_start_area_radius_m * params.reloc_start_area_radius_m;
      std::vector<Eigen::Vector4d> pts;
      for (const auto& sm : submaps) {
        if (!sm->frame) {
          continue;
        }
        const Eigen::Matrix4d T = sm->T_world_origin.matrix();
        for (size_t i = 0; i < sm->frame->size(); i++) {
          const Eigen::Vector4d p = T * sm->frame->points[i];
          if ((p.head<3>() - start).squaredNorm() <= r2) {
            pts.push_back(p);
          }
        }
      }
      if (pts.size() >= 100) {
        fpfh_cloud = std::make_shared<gtsam_points::PointCloudCPU>(pts);
        logger->info("continuation: FPFH target limited to the start area (r={:.1f} m): {} points", params.reloc_start_area_radius_m, pts.size());
      }
    }
    if (!fpfh_cloud) {
      fpfh_cloud = full_map_cloud;
    }
    if (fpfh_cloud) {
      aligner = std::make_shared<MapAligner>(fpfh_cloud, ap, logger);
    }
  }

  // Relocalization is ready when its target exists: the VGICP voxelmaps for "yaw_sweep", or a valid
  // FPFH aligner for "fpfh".
  const bool reloc_ready = (params.reloc_method == "yaw_sweep") ? !reloc_refine_voxelmaps.empty() : (aligner && aligner->valid());
  if (reloc_ready) {
    continuation_mode = true;
    logger->info("continuation: {} prior submaps loaded; the new session will be relocalized onto them (method={})", continuation_start_id, params.reloc_method);
  } else {
    logger->error("continuation: failed to build the relocalization target (method={}); the loaded map will not be extended with the new session", params.reloc_method);
  }
}

std::vector<Eigen::Vector4d> GlobalMapping::accumulate_pending_source() const {
  // Accumulate ALL pending submaps into one odom-frame cloud. The submaps are mutually consistent
  // through odometry, so a moving start just grows the source geometry with every retry.
  std::vector<Eigen::Vector4d> src;
  for (const auto& sm : pending_submaps) {
    if (!sm->frame || sm->frame->size() == 0) {
      continue;
    }
    const Eigen::Matrix4d T = sm->T_world_origin.matrix();
    const auto* pts = sm->frame->points;
    const size_t n = sm->frame->size();
    const size_t stride = std::max<size_t>(1, n / 40000);  // bound the source on dense submaps
    src.reserve(src.size() + n / stride + 1);
    for (size_t i = 0; i < n; i += stride) {
      src.emplace_back(T * pts[i]);
    }
  }
  return src;
}

void GlobalMapping::try_relocalize_pending() {
  std::vector<Eigen::Vector4d> src = accumulate_pending_source();
  if (src.empty()) {
    logger->warn("continuation: pending submaps hold no points yet; retrying with the next submap");
    return;
  }

  const int attempt = pending_submaps.size();

  // An operator override (the website's confirm — see RelocOverride) short-circuits the autonomous
  // search entirely. Checked BEFORE the sweep/RANSAC so a confirm does not have to sit out another
  // multi-second attempt whose answer it is about to discard.
  {
    Eigen::Isometry3d T_override;
    if (RelocOverride::take(T_override)) {
      apply_operator_override(src, T_override, attempt);
      return;
    }
  }

  bool ok = false;
  double coarse_inlier = -1.0;                       // FPFH-only diagnostic
  double sweep_best_inlier = -1.0;                    // yaw-sweep: best hypothesis inlier (candidate gate)
  Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();

  if (params.reloc_method == "yaw_sweep") {
    if (reloc_refine_voxelmaps.empty()) {
      logger->error("continuation: no VGICP relocalization target; placing the new session at identity offset");
      reloc_abandoned = true;
      return;
    }
    ok = relocalize_yaw_sweep(src, correction, &sweep_best_inlier);
  } else {  // "fpfh"
    if (!aligner || !aligner->valid()) {
      logger->error("continuation: no FPFH relocalization target; placing the new session at identity offset");
      reloc_abandoned = true;
      return;
    }
    MapAlignerParams ap;
    ap.voxel_resolution = params.reloc_voxel_resolution;
    ap.fpfh_radius = params.reloc_fpfh_radius;
    ap.dof = params.reloc_dof;
    ap.search_radius_m = params.reloc_search_radius_m;
    ap.min_inlier_rate = params.reloc_min_inlier_rate;
    ap.registration = params.reloc_registration;
    ap.num_threads = params.num_threads;

    const auto result = aligner->align(src, ap);
    coarse_inlier = result.inlier_rate;
    ok = result.success;
    correction = result.T_target_source;
    if (ok && params.reloc_refine) {
      ok = refine_relocalization(src, correction);
    }
  }

  // Re-check the mailbox: the search above takes seconds, and an override that arrived during it
  // (including the one that just broke the yaw sweep out of its loop) supersedes whatever it
  // concluded. Without this the confirm would wait for the NEXT submap, which is exactly the delay
  // the early break was meant to avoid.
  {
    Eigen::Isometry3d T_override;
    if (RelocOverride::take(T_override)) {
      apply_operator_override(src, T_override, attempt);
      return;
    }
  }

  correction.linear() = Eigen::Quaterniond(correction.linear()).normalized().toRotationMatrix();

  // Broadcast this attempt's best-fit hypothesis as a visualization candidate — accepted OR rejected
  // — so a viewer/website can watch the relocalizer step through its choices. yaw_sweep always leaves
  // its best hypothesis in `correction` (skip only when nothing landed within the spawn radius:
  // sweep_best_inlier < 0); fpfh's `correction` is only meaningful when it succeeded.
  const bool has_candidate = (params.reloc_method == "yaw_sweep") ? (sweep_best_inlier >= 0.0) : ok;
  if (has_candidate) {
    GlobalMappingCallbacks::on_relocalization_candidate(correction, attempt, ok);
  }

  if (ok) {
    accept_relocalization(src, correction, attempt, params.reloc_method);
    return;
  }

  // reloc_max_submaps <= 0 means UNBOUNDED: keep buffering and retrying forever rather than falling
  // back to fresh mapping. This is the mode that makes an operator override meaningful — the human
  // needs the session to still be waiting, in continuation mode, whenever they get around to
  // confirming. The escape hatch is unchanged: save() flushes whatever is still buffered.
  const bool bounded = params.reloc_max_submaps > 0;
  if (bounded && attempt >= params.reloc_max_submaps) {
    reloc_abandoned = true;  // consumed by the fresh-mapping fallback in insert_submap
    logger->error("continuation: relocalization abandoned after {} attempts", attempt);
  } else {
    const std::string of = bounded ? fmt::format("/{}", params.reloc_max_submaps) : std::string("/unbounded");
    if (params.reloc_method == "fpfh") {
      logger->warn(
        "continuation: relocalization attempt {}{} failed (coarse inlier_rate={:.2f}); buffering submaps and retrying with more geometry "
        "(an operator override on ~/reloc_override accepts a pose outright)",
        attempt,
        of,
        coarse_inlier);
    } else {
      logger->warn(
        "continuation: relocalization attempt {}{} failed; buffering submaps and retrying with more geometry "
        "(an operator override on ~/reloc_override accepts a pose outright)",
        attempt,
        of);
    }
  }

  // Announce the failure on ~/reloc_progress so the website's bar resets instead of sitting at a
  // full amber sweep: "failed" -> the bar empties and refills when the next attempt streams,
  // "abandoned" -> terminal, no further attempts are coming. total=0 keeps frac at 0.
  GlobalMappingCallbacks::on_relocalization_progress(0, 0, attempt, reloc_abandoned ? "abandoned" : "failed");
}

void GlobalMapping::accept_relocalization(const std::vector<Eigen::Vector4d>& src, const Eigen::Isometry3d& correction, int attempt, const std::string& method) {
  const Eigen::Vector3d t = correction.translation();
  const double yaw = std::atan2(correction.linear()(1, 0), correction.linear()(0, 0));

  // Always REPORT the accepted alignment on the latched ~/map_offset topic (+ the rviz log). This is
  // the boot alignment for late-joining consumers on the robot AND the readout for the sim
  // spawn-alignment robustness test. Reporting is independent of whether we apply it below.
  GlobalMappingCallbacks::on_relocalized(correction);
  // Snap the progress bar to complete so the website flips from "waiting" to "ready" in lockstep
  // with the map_offset that on_relocalized emits.
  GlobalMappingCallbacks::on_relocalization_progress(1, 1, attempt, "done");
  // Hand the matched source cloud + accepted pose to the viewer so it can overlay a white preview
  // on the loaded prior map (visual confirmation of where the new session locked on). Skipped when
  // there is no geometry yet (an override confirmed before the first submap closed) — an empty
  // preview would blank the viewer's overlay rather than add to it.
  if (!src.empty()) {
    GlobalMappingCallbacks::on_relocalization_preview(src, correction);
  }

  if (params.apply_relocalization) {
    // Real robot: APPLY it — re-anchor the new session onto the prior map.
    T_map_odom = correction;
    relocalized = true;
    logger->info(
      "continuation: relocalized the new session onto the prior map (method={}, attempt {}): t=[{:.2f}, {:.2f}, {:.2f}] yaw={:.1f}deg",
      method,
      attempt,
      t.x(),
      t.y(),
      t.z(),
      yaw * 180.0 / M_PI);
  } else {
    // Sim mode: computed + reported only. Do NOT re-anchor — build the map fresh in the simulator's
    // odom frame so it keeps piggybacking Isaac Sim's ground-truth TF (byte-for-byte the same
    // alignment as the robot, minus the application). Consumed by insert_submap below.
    reloc_report_only_done = true;
    logger->info(
      "continuation: [sim/report-only] alignment COMPUTED but NOT applied (method={}, attempt {}): t=[{:.2f}, {:.2f}, {:.2f}] yaw={:.1f}deg — mapping fresh "
      "in the odom frame",
      method,
      attempt,
      t.x(),
      t.y(),
      t.z(),
      yaw * 180.0 / M_PI);
  }
}

void GlobalMapping::apply_pending_reloc_override() {
  // Applied the moment the override lands, NOT on the next submap. Relocalization attempts are
  // driven by submap arrivals, so consuming it there meant the operator confirmed and then nothing
  // visibly happened until the robot moved enough to close a submap — and nav could not start,
  // because it waits on the map_offset that acceptance publishes. This runs on the global-mapping
  // thread (polled by AsyncGlobalMapping under its mutex), so it is safe to touch the graph state.
  if (!continuation_mode || relocalized || reloc_abandoned || reloc_report_only_done) {
    return;  // nothing to relocalize onto, or already resolved; leave the mailbox for a fresh session
  }

  Eigen::Isometry3d T_override;
  if (!RelocOverride::take(T_override)) {
    return;
  }

  // Whatever geometry has accumulated so far — possibly none, if the confirm beat the first submap.
  // apply_operator_override handles an empty source by accepting the pose verbatim.
  apply_operator_override(accumulate_pending_source(), T_override, static_cast<int>(pending_submaps.size()));

  // Same tail insert_submap runs after an attempt resolves. Skipping it here would strand every
  // buffered submap: once `relocalized` is set, insert_submap never re-enters the branch that
  // flushes them, and save() would later append the session's opening geometry AFTER everything
  // that followed it — writing a graph with backwards odometry deltas to disk.
  settle_relocalization_outcome();
}

void GlobalMapping::apply_operator_override(const std::vector<Eigen::Vector4d>& src, const Eigen::Isometry3d& T_override, int attempt) {
  Eigen::Isometry3d correction = T_override;
  std::string method = "operator";

  // Refine, but NEVER reject. The operator either accepted a candidate the inlier gate had thrown
  // out, or hand-placed the scan by eye — both are poses worth polishing with a bounded VGICP fit,
  // and neither is a pose we are entitled to veto. refine_relocalization already applies exactly the
  // gates we want (max correction + min inlier rate) and mutates its argument ONLY on success, so
  // reuse it and simply reinterpret its rejection: not "relocalization failed", just "the operator's
  // pose stands as given". Either branch accepts.
  // The !empty() guard matters for the label: refine_relocalization returns true WITHOUT touching its
  // argument when there are no refine voxelmaps, which would log "operator+refine" for a pose nothing
  // refined. That log is the operator's only evidence of what GLIM did to their placement.
  // src can be empty when the confirm arrives before the first submap closes — there is simply
  // nothing to fit yet, so the operator's pose is taken verbatim (which is the intended fallback
  // anyway).
  if (params.reloc_refine && !reloc_refine_voxelmaps.empty() && !src.empty()) {
    Eigen::Isometry3d refined = T_override;
    if (refine_relocalization(src, refined)) {
      correction = refined;
      method = "operator+refine";
    } else {
      logger->info("continuation: operator override kept VERBATIM (the refine did not pass its trust gates; the operator's pose is authoritative)");
    }
  }

  correction.linear() = Eigen::Quaterniond(correction.linear()).normalized().toRotationMatrix();

  logger->warn("continuation: ACCEPTING an operator relocalization override at attempt {} — the inlier gate is bypassed", attempt);
  // Report it as an accepted candidate too, so a viewer animating ~/reloc_candidate lands the scan on
  // the same pose it is about to see on ~/map_offset instead of freezing on the last rejected guess.
  GlobalMappingCallbacks::on_relocalization_candidate(correction, attempt, true);
  accept_relocalization(src, correction, attempt, method);
}

std::pair<Eigen::Isometry3d, double>
GlobalMapping::vgicp_fit(const std::shared_ptr<gtsam_points::PointCloudCPU>& src, const Eigen::Isometry3d& T_init, int max_iterations) const {
  gtsam::Values values;
  values.insert(gtsam::Key(0), gtsam::Pose3(T_init.matrix()));

  gtsam::NonlinearFactorGraph graph;

#ifdef GTSAM_POINTS_USE_CUDA
  // GPU fit: pose the (fixed) map-frame target voxelmaps against the source variable Key(0), one factor
  // per resolution level, sharing the round-robin CUDA stream pool used elsewhere in this file. The GPU
  // factor exposes the same inlier_fraction() the yaw-sweep gates on, so the caller is oblivious to the
  // path taken. src carries covariances (added by the caller); clone them up to the GPU once.
  if (params.enable_gpu && !reloc_refine_voxelmaps_gpu.empty()) {
    auto src_gpu = gtsam_points::PointCloudGPU::clone(*src);
    std::vector<const gtsam_points::IntegratedVGICPFactorGPU*> gpu_factors;  // owned by `graph`
    for (const auto& voxelmap : reloc_refine_voxelmaps_gpu) {
      const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
      const auto& stream = stream_buffer.first;
      const auto& buffer = stream_buffer.second;
      auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(gtsam::Pose3(), gtsam::Key(0), voxelmap, src_gpu, stream, buffer);
      graph.add(f);
      gpu_factors.push_back(f.get());
    }

    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setMaxIterations(max_iterations);
    lm_params.setAbsoluteErrorTol(1e-3);
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
    values = optimizer.optimize();

    const Eigen::Isometry3d refined(values.at<gtsam::Pose3>(gtsam::Key(0)).matrix());
    const double inlier_rate = gpu_factors.empty() ? 0.0 : gpu_factors.front()->inlier_fraction();
    return {refined, inlier_rate};
  }
#endif

  std::vector<const gtsam_points::IntegratedVGICPFactor*> factors;  // owned by `graph`
  for (const auto& voxelmap : reloc_refine_voxelmaps) {
    auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), gtsam::Key(0), voxelmap, src);
    f->set_num_threads(params.num_threads);
    graph.add(f);
    factors.push_back(f.get());
  }

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(max_iterations);
  lm_params.setAbsoluteErrorTol(1e-3);
  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
  values = optimizer.optimize();

  const Eigen::Isometry3d refined(values.at<gtsam::Pose3>(gtsam::Key(0)).matrix());
  const double inlier_rate = factors.empty() ? 0.0 : factors.front()->inlier_fraction();
  return {refined, inlier_rate};
}

bool GlobalMapping::relocalize_yaw_sweep(const std::vector<Eigen::Vector4d>& src_points, Eigen::Isometry3d& out_T, double* out_best_inlier) {
  if (out_best_inlier) {
    *out_best_inlier = -1.0;
  }
  if (reloc_refine_voxelmaps.empty() || submaps.empty() || pending_submaps.empty()) {
    return false;
  }

  auto src = std::make_shared<gtsam_points::PointCloudCPU>(src_points);
  auto ds = gtsam_points::voxelgrid_sampling(src, params.reloc_voxel_resolution, params.num_threads);
  if (!ds || ds->size() < 100) {
    return false;
  }
  ds->add_covs(gtsam_points::estimate_covariances(ds->points, ds->size(), 10, params.num_threads));

  // Anchor: the first new submap sits at (roughly) the same physical place as the map's first submap
  // (same start, same route), so base = M0 * P0^-1 maps the new-session odom frame onto the map with
  // matched headings. The reboot heading is unknown, so we twist the anchor by a global yaw about M0's
  // position and let VGICP settle each hypothesis; the best-inlier fit that lands within the spawn
  // radius wins.
  const Eigen::Isometry3d M0 = submaps.front()->T_world_origin;
  const Eigen::Isometry3d P0 = pending_submaps.front()->T_world_origin;
  const Eigen::Isometry3d base = M0 * P0.inverse();
  const Eigen::Vector3d c = M0.translation();

  const double step = std::max(1.0, params.reloc_yaw_step_deg);
  const int num_hyp = static_cast<int>(std::ceil(360.0 / step));
  const int attempt = static_cast<int>(pending_submaps.size());

  // Drive a live progress bar (relayed to the website by the robot, RViz-visible on ~/reloc_progress):
  // the yaw sweep is the dominant, multi-second cost, so report each fitted hypothesis.
  GlobalMappingCallbacks::on_relocalization_progress(0, num_hyp, attempt, "yaw-sweep");

  double best_inlier = -1.0;
  bool interrupted = false;
  int fitted = 0;
  Eigen::Isometry3d best_T = Eigen::Isometry3d::Identity();
  for (int k = 0; k < num_hyp; k++) {
    // Bail out the moment an operator override lands. The mailbox is otherwise only polled between
    // attempts, and with unbounded retries an attempt's cost grows with the accumulated source cloud
    // — so late in a long wait, "confirm" could sit behind a sweep for longer than the robot's ack
    // timeout. The override supersedes whatever this sweep would have concluded anyway.
    if (RelocOverride::pending()) {
      logger->info("continuation: yaw sweep interrupted at hypothesis {}/{} — an operator override is waiting", k, num_hyp);
      interrupted = true;
      break;
    }
    fitted = k + 1;
    const double theta = k * step * M_PI / 180.0;
    Eigen::Isometry3d twist = Eigen::Isometry3d::Identity();
    twist.linear() = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    twist.translation() = c - twist.linear() * c;  // yaw about the anchor point c
    const Eigen::Isometry3d T_init = twist * base;

    const auto [T_ref, inlier] = vgicp_fit(ds, T_init, 20);

    // Sanity gate: the first new submap must land within the spawn radius of the map's first submap.
    const Eigen::Vector3d landed = (T_ref * P0).translation();
    if ((landed - c).norm() > params.reloc_search_radius_m) {
      GlobalMappingCallbacks::on_relocalization_progress(k + 1, num_hyp, attempt, "yaw-sweep");
      continue;
    }
    if (inlier > best_inlier) {
      best_inlier = inlier;
      best_T = T_ref;
    }
    GlobalMappingCallbacks::on_relocalization_progress(k + 1, num_hyp, attempt, "yaw-sweep");
  }

  logger->info(
    "continuation: yaw-sweep best inlier_rate={:.2f} over {}/{} hypotheses{} (gate {:.2f})",
    best_inlier,
    fitted,
    num_hyp,
    interrupted ? " [interrupted by an operator override]" : "",
    params.reloc_refine_min_inlier_rate);
  // Always hand back the best hypothesis found (out_best_inlier < 0 means none landed within the
  // spawn radius), so the caller can broadcast even a rejected attempt as a visualization candidate.
  out_T = best_T;
  if (out_best_inlier) {
    *out_best_inlier = best_inlier;
  }
  // A truncated sweep must never claim success, even if an early hypothesis happened to clear the
  // gate: the operator's override supersedes it, and the caller consumes that override immediately
  // after this returns. Returning false here keeps that guarantee local instead of relying on the
  // caller's take() ordering to mask a partial result.
  if (interrupted || best_inlier < params.reloc_refine_min_inlier_rate) {
    return false;
  }
  return true;
}

bool GlobalMapping::refine_relocalization(const std::vector<Eigen::Vector4d>& src_points, Eigen::Isometry3d& T_map_odom_coarse) {
  if (reloc_refine_voxelmaps.empty()) {
    logger->warn("continuation: refine skipped (no refine voxelmaps)");
    return true;
  }

  auto src = std::make_shared<gtsam_points::PointCloudCPU>(src_points);
  auto downsampled = gtsam_points::voxelgrid_sampling(src, params.reloc_voxel_resolution, params.num_threads);
  if (!downsampled || downsampled->size() < 100) {
    logger->warn("continuation: refine rejected (accumulated cloud too small after downsampling)");
    return false;
  }
  downsampled->add_covs(gtsam_points::estimate_covariances(downsampled->points, downsampled->size(), 10, params.num_threads));

  // Indeterminate spinner for the single fine-fit (fpfh method only; the yaw sweep refines inline).
  GlobalMappingCallbacks::on_relocalization_progress(0, 0, static_cast<int>(pending_submaps.size()), "refine");

  // VGICP-fit the source (odom frame) to the full-map voxelmaps from the coarse FPFH pose.
  const auto [refined, inlier_rate] = vgicp_fit(downsampled, T_map_odom_coarse, 30);
  const Eigen::Isometry3d delta = T_map_odom_coarse.inverse() * refined;
  const double delta_t = delta.translation().norm();
  const double delta_r = Eigen::AngleAxisd(delta.linear()).angle();

  logger->info("continuation: refine moved {:.2f}m / {:.1f}deg from the coarse pose, inlier_rate={:.2f}", delta_t, delta_r * 180.0 / M_PI, inlier_rate);

  if (delta_t > params.reloc_refine_max_correction || delta_r > 15.0 * M_PI / 180.0) {
    logger->warn("continuation: refine rejected (correction exceeds the trust gate: {:.2f}m / {:.1f}deg)", delta_t, delta_r * 180.0 / M_PI);
    return false;
  }
  if (inlier_rate < params.reloc_refine_min_inlier_rate) {
    logger->warn("continuation: refine rejected (inlier rate {:.2f} below gate {:.2f})", inlier_rate, params.reloc_refine_min_inlier_rate);
    return false;
  }

  T_map_odom_coarse = refined;
  return true;
}

void GlobalMapping::insert_submap(const SubMap::Ptr& submap) {
  ensure_prior_map_loaded();

  // Continuation: buffer new submaps (out of the graph) until relocalization resolves, retrying
  // with the accumulated cloud each time one arrives. On success (or after giving up) the buffer
  // is flushed in order: the first pending submap gets the absolute (relocalized) placement, the
  // rest chain through odometry as usual. The IMU queue is untouched while buffering, so the
  // IMU factors between the flushed submaps are still created normally.
  if (continuation_mode && !relocalized && !reloc_abandoned && !reloc_report_only_done) {
    pending_submaps.push_back(submap);
    try_relocalize_pending();
    settle_relocalization_outcome();
    return;
  }

  insert_submap_internal(submap);
}

void GlobalMapping::settle_relocalization_outcome() {
  // Everything that must follow a relocalization attempt resolving, one way or the other. Extracted
  // from insert_submap because an operator override can now resolve a continuation from OUTSIDE the
  // submap path (apply_pending_reloc_override), and that path silently skipped the flush below —
  // stranding every buffered submap, which save() then appended out of order into the graph.
  if (reloc_abandoned) {
    // Could not align to the loaded map: fall back to fresh mapping, then flush the buffered
    // submaps as the fresh session's first submaps.
    reset_to_fresh_mapping();
  } else if (reloc_report_only_done) {
    // Sim mode: the alignment was computed + reported but is intentionally NOT applied. Drop the
    // loaded prior from the active graph and map fresh in the odom frame (this is not a failure).
    reset_to_fresh_mapping(/*relocalization_failed=*/false);
  }
  if (relocalized || !continuation_mode) {
    std::vector<SubMap::Ptr> to_flush;
    to_flush.swap(pending_submaps);
    for (const auto& pending : to_flush) {
      insert_submap_internal(pending);
    }
  }
}

void GlobalMapping::reset_to_fresh_mapping(bool relocalization_failed) {
  if (relocalization_failed) {
    logger->error("========================================================================");
    logger->error("continuation: RELOCALIZATION FAILED — could not align the new session to the loaded map.");
    logger->error("continuation: falling back to FRESH MAPPING. The loaded prior map is discarded for this run");
    logger->error("continuation: (the original on disk is untouched); a brand-new map is built and saved into the");
    logger->error("continuation: continuation output folder on shutdown.");
    logger->error("========================================================================");
  } else {
    logger->info(
      "continuation: [sim/report-only] alignment reported; building the map fresh in the odom frame "
      "(the loaded prior map was used only as the alignment test target and is left untouched).");
  }

  // Drop the loaded map from the active graph and start over with an empty optimizer.
  submaps.clear();
  subsampled_submaps.clear();
  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  gtsam::ISAM2Params isam2_params;
  if (params.use_isam2_dogleg) {
    gtsam::ISAM2DoglegParams dogleg_params;
    isam2_params.setOptimizationParams(dogleg_params);
  }
  isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);
  if (params.enable_optimization) {
    isam2.reset(new gtsam_points::ISAM2Ext(isam2_params));
  } else {
    isam2.reset(new gtsam_points::ISAM2ExtDummy(isam2_params));
  }

  // Continuation is over for graph purposes; subsequent submaps insert as a normal fresh session
  // (first = X(0) with a damping anchor, the rest chain). continuation_save_redirect stays true so
  // the fresh map is still written into the continuation output folder.
  continuation_mode = false;
  relocalized = false;
  reloc_abandoned = false;
  reloc_report_only_done = false;
  continuation_start_id = -1;
  // There is no longer a prior map to align onto, so a late operator confirm (or a latched override
  // replayed on reconnect) must not re-anchor this now-fresh session.
  RelocOverride::clear();
  reloc_refine_voxelmaps.clear();
  aligner.reset();
}

void GlobalMapping::insert_submap_internal(const SubMap::Ptr& submap) {
  logger->debug("insert_submap id={} |frame|={}", submap->id, submap->frame->size());

  const int current = submaps.size();
  const int last = current - 1;

  // First submap of a new session appended onto a loaded prior map: there is no continuous
  // odometry across the GLIM restart, so it gets the absolute relocalized placement (T_map_odom)
  // instead of chaining from the (loaded) previous submap.
  const bool session_start = continuation_mode && current == continuation_start_id;

  insert_submap(current, submap);

  gtsam::Pose3 current_T_world_submap = gtsam::Pose3::Identity();
  gtsam::Pose3 last_T_world_submap = gtsam::Pose3::Identity();

  if (session_start) {
    // Absolute placement via relocalization; the prior-map matching-cost factors below tie it in.
    current_T_world_submap = gtsam::Pose3((T_map_odom * submap->T_world_origin).matrix());
  } else if (current != 0) {
    if (isam2->valueExists(X(last))) {
      last_T_world_submap = isam2->calculateEstimate<gtsam::Pose3>(X(last));
    } else {
      last_T_world_submap = new_values->at<gtsam::Pose3>(X(last));
    }

    const Eigen::Isometry3d T_origin0_endpointR0 = submaps[last]->T_origin_endpoint_R;
    const Eigen::Isometry3d T_origin1_endpointL1 = submaps[current]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_endpointR0_endpointL1 = submaps[last]->odom_frames.back()->T_world_sensor().inverse() * submaps[current]->odom_frames.front()->T_world_sensor();
    const Eigen::Isometry3d T_origin0_origin1 = T_origin0_endpointR0 * T_endpointR0_endpointL1 * T_origin1_endpointL1.inverse();

    current_T_world_submap = last_T_world_submap * gtsam::Pose3(T_origin0_origin1.matrix());
  } else {
    current_T_world_submap = gtsam::Pose3(submap->T_world_origin.matrix());
  }

  new_values->insert(X(current), current_T_world_submap);
  submap->T_world_origin = Eigen::Isometry3d(current_T_world_submap.matrix());

  Callbacks::on_insert_submap(submap);

  submap->drop_frame_points();

  if (current == 0) {
    new_factors->emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params.init_pose_damping_scale);
  } else if (session_start) {
    // No between-factor across the restart gap (the odometry chain is broken). Connect to the prior
    // map purely via spatial overlap. If relocalization failed the submap is a free-floating gauge,
    // so anchor it with a damping prior to keep the optimization well-posed.
    new_factors->add(*create_matching_cost_factors(current));
    if (!relocalized) {
      new_factors->emplace_shared<gtsam_points::LinearDampingFactor>(X(current), 6, params.init_pose_damping_scale);
    }
  } else {
    new_factors->add(*create_between_factors(current));
    new_factors->add(*create_matching_cost_factors(current));
  }

  if (params.enable_imu) {
    logger->debug("create IMU factor");
    if (submap->odom_frames.front()->frame_id != FrameID::IMU) {
      logger->warn("odom frames are not estimated in the IMU frame while global mapping requires IMU estimation");
    }

    // Local velocities
    const gtsam::imuBias::ConstantBias imu_biasL(submap->frames.front()->imu_bias);
    const gtsam::imuBias::ConstantBias imu_biasR(submap->frames.back()->imu_bias);

    const Eigen::Vector3d v_origin_imuL = submap->T_world_origin.linear().inverse() * submap->frames.front()->v_world_imu;
    const Eigen::Vector3d v_origin_imuR = submap->T_world_origin.linear().inverse() * submap->frames.back()->v_world_imu;

    const auto prior_noise3 = gtsam::noiseModel::Isotropic::Precision(3, 1e6);
    const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);

    // At a session start (first new submap onto a loaded map) there is no IMU data spanning the
    // restart gap and no continuous L-endpoint from the previous (loaded) session, so treat it like
    // the very first submap: only the R endpoint is created and no cross-boundary ImuFactor is added.
    if (current > 0 && !session_start) {
      new_values->insert(E(current * 2), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_L).matrix()));
      new_values->insert(V(current * 2), (submap->T_world_origin.linear() * v_origin_imuL).eval());
      new_values->insert(B(current * 2), imu_biasL);

      new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current), E(current * 2), gtsam::Pose3(submap->T_origin_endpoint_L.matrix()), prior_noise6);
      new_factors->emplace_shared<gtsam_points::RotateVector3Factor>(X(current), V(current * 2), v_origin_imuL, prior_noise3);
      new_factors->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current * 2), imu_biasL, prior_noise6);
      new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(current * 2), B(current * 2 + 1), gtsam::imuBias::ConstantBias(), prior_noise6);
    }

    new_values->insert(E(current * 2 + 1), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_R).matrix()));
    new_values->insert(V(current * 2 + 1), (submap->T_world_origin.linear() * v_origin_imuR).eval());
    new_values->insert(B(current * 2 + 1), imu_biasR);

    new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current), E(current * 2 + 1), gtsam::Pose3(submap->T_origin_endpoint_R.matrix()), prior_noise6);
    new_factors->emplace_shared<gtsam_points::RotateVector3Factor>(X(current), V(current * 2 + 1), v_origin_imuR, prior_noise3);
    new_factors->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current * 2 + 1), imu_biasR, prior_noise6);

    if (current != 0 && !session_start) {
      const double stampL = submaps[last]->frames.back()->stamp;
      const double stampR = submaps[current]->frames.front()->stamp;

      int num_integrated;
      const int imu_read_cursor = imu_integration->integrate_imu(stampL, stampR, imu_biasL, &num_integrated);
      imu_integration->erase_imu_data(imu_read_cursor);

      if (num_integrated < 2) {
        logger->warn("insufficient IMU data between submaps (global_mapping)!!");
        new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(V(last * 2 + 1), V(current * 2), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Precision(3, 1.0));
      } else {
        new_factors
          ->emplace_shared<gtsam::ImuFactor>(E(last * 2 + 1), V(last * 2 + 1), E(current * 2), V(current * 2), B(last * 2 + 1), imu_integration->integrated_measurements());
      }
    }
  }

  logger->debug("|new_factors|={} |new_values|={}", new_factors->size(), new_values->size());

  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  auto result = update_isam2(*new_factors, *new_values);
  Callbacks::on_smoother_update_result(*isam2, result);

  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

void GlobalMapping::insert_submap(int current, const SubMap::Ptr& submap) {
  submap->voxelmaps.clear();

  // Adaptively determine the voxel resolution based on the median distance
  const int max_scan_count = 256;
  const double dist_median = gtsam_points::median_distance(submap->frame, max_scan_count);
  const double p = std::max(0.0, std::min(1.0, (dist_median - params.submap_voxel_resolution_dmin) / (params.submap_voxel_resolution_dmax - params.submap_voxel_resolution_dmin)));
  const double base_resolution = params.submap_voxel_resolution + p * (params.submap_voxel_resolution_max - params.submap_voxel_resolution);

  // Create frame and voxelmaps
  gtsam_points::PointCloud::ConstPtr subsampled_submap;
  if (params.randomsampling_rate > 0.99) {
    subsampled_submap = submap->frame;
  } else {
    subsampled_submap = gtsam_points::random_sampling(submap->frame, params.randomsampling_rate, mt);
  }

#ifdef GTSAM_POINTS_USE_CUDA
  if (params.enable_gpu && !submap->frame->points_gpu) {
    submap->frame = gtsam_points::PointCloudGPU::clone(*submap->frame);
  }

  if (params.enable_gpu) {
    if (params.randomsampling_rate > 0.99) {
      subsampled_submap = submap->frame;
    } else {
      subsampled_submap = gtsam_points::PointCloudGPU::clone(*subsampled_submap);
    }

    for (int i = 0; i < params.submap_voxelmap_levels; i++) {
      const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, i);
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution);
      voxelmap->insert(*submap->frame);
      submap->voxelmaps.push_back(voxelmap);
    }
  }
#endif

  if (submap->voxelmaps.empty()) {
    for (int i = 0; i < params.submap_voxelmap_levels; i++) {
      const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, i);
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
      voxelmap->insert(*subsampled_submap);
      submap->voxelmaps.push_back(voxelmap);
    }
  }

  submaps.push_back(submap);
  subsampled_submaps.push_back(subsampled_submap);
}

void GlobalMapping::find_overlapping_submaps(double min_overlap) {
  if (submaps.empty()) {
    return;
  }

  // Between factors are Vector2i actually. A bad use of Vector3i
  std::unordered_set<Eigen::Vector3i, gtsam_points::Vector3iHash> existing_factors;
  for (const auto& factor : isam2->getFactorsUnsafe()) {
    if (factor == nullptr) {
      continue;
    }
    if (factor->keys().size() != 2) {
      continue;
    }

    gtsam::Symbol sym1(factor->keys()[0]);
    gtsam::Symbol sym2(factor->keys()[1]);
    if (sym1.chr() != 'x' || sym2.chr() != 'x') {
      continue;
    }

    existing_factors.emplace(sym1.index(), sym2.index(), 0);
  }

  double squared_max_implicit_loop_distance = params.max_implicit_loop_distance * params.max_implicit_loop_distance;
  for (int i = 0; i < submaps.size(); i++) {
    for (int j = i + 1; j < submaps.size(); j++) {
      if (existing_factors.count(Eigen::Vector3i(i, j, 0))) {
        continue;
      }

      const Eigen::Isometry3d delta = submaps[i]->T_world_origin.inverse() * submaps[j]->T_world_origin;
      const double squared_dist = delta.translation().squaredNorm();
      if (squared_dist > squared_max_implicit_loop_distance) {
        continue;
      }

      const double overlap = gtsam_points::overlap_auto(submaps[i]->voxelmaps.back(), subsampled_submaps[j], delta);
      if (overlap < min_overlap) {
        continue;
      }

      if (false) {
      }
#ifdef GTSAM_POINTS_USE_CUDA
      else if (std::dynamic_pointer_cast<gtsam_points::GaussianVoxelMapGPU>(submaps[i]->voxelmaps.back()) && subsampled_submaps[j]->points_gpu) {
        const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
        const auto& stream = stream_buffer.first;
        const auto& buffer = stream_buffer.second;
        for (const auto& voxelmap : submaps[i]->voxelmaps) {
          new_factors->emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(i), X(j), voxelmap, subsampled_submaps[j], stream, buffer);
        }
      }
#endif
      else {
        for (const auto& voxelmap : submaps[i]->voxelmaps) {
          new_factors->emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(i), X(j), voxelmap, subsampled_submaps[j]);
        }
      }
    }
  }

  logger->info("new overlapping {} submap pairs found", new_factors->size());

  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  auto result = update_isam2(*new_factors, *new_values);
  Callbacks::on_smoother_update_result(*isam2, result);

  new_factors->resize(0);
  new_values->clear();

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

void GlobalMapping::optimize() {
  if (isam2->empty()) {
    return;
  }

  logger->debug("|new_factors|={} |new_values|={}", new_factors->size(), new_values->size());

  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  auto result = update_isam2(*new_factors, *new_values);

  new_factors.reset(new gtsam::NonlinearFactorGraph);
  new_values.reset(new gtsam::Values);

  Callbacks::on_smoother_update_result(*isam2, result);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMapping::create_between_factors(int current) const {
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0 || !params.enable_between_factors) {
    return factors;
  }

  const int last = current - 1;
  const gtsam::Pose3 init_delta = gtsam::Pose3((submaps[last]->T_world_origin.inverse() * submaps[current]->T_world_origin).matrix());

  if (params.between_registration_type == "NONE") {
    factors->add(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), init_delta, gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
    return factors;
  }

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3::Identity());
  values.insert(X(1), init_delta);

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3::Identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(X(0), X(1), submaps[last]->frame, submaps[current]->frame);
  factor->set_max_correspondence_distance(0.5);
  factor->set_num_threads(2);
  graph.add(factor);

  logger->debug("--- LM optimization ---");
  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setlambdaInitial(1e-12);
  lm_params.setMaxIterations(10);
  lm_params.callback = [this](const auto& status, const auto& values) { logger->debug(status.to_string()); };

#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
  arena->execute([&] {
#endif
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
    values = optimizer.optimize();

#ifdef GTSAM_USE_TBB
  });
#endif

  const gtsam::Pose3 estimated_delta = values.at<gtsam::Pose3>(X(1));
  const auto linearized = factor->linearize(values);
  const gtsam::Matrix6 H = linearized->hessianBlockDiagonal()[X(1)] + 1e6 * gtsam::Matrix6::Identity();

  factors->add(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), estimated_delta, gtsam::noiseModel::Gaussian::Information(H)));
  return factors;
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMapping::create_matching_cost_factors(int current) const {
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0) {
    return factors;
  }

  const auto& current_submap = submaps.back();

  double previous_overlap = 0.0;
  double squared_max_implicit_loop_distance = params.max_implicit_loop_distance * params.max_implicit_loop_distance;

  for (int i = 0; i < current; i++) {
    const double squared_dist = (submaps[i]->T_world_origin.translation() - current_submap->T_world_origin.translation()).squaredNorm();
    if (squared_dist > squared_max_implicit_loop_distance) {
      continue;
    }

    const Eigen::Isometry3d delta = submaps[i]->T_world_origin.inverse() * current_submap->T_world_origin;
    const double overlap = gtsam_points::overlap_auto(submaps[i]->voxelmaps.back(), current_submap->frame, delta);

    previous_overlap = i == current - 1 ? overlap : previous_overlap;
    if (overlap < params.min_implicit_loop_overlap) {
      continue;
    }

    if (params.registration_error_factor_type == "VGICP") {
      for (const auto& voxelmap : submaps[i]->voxelmaps) {
        factors->emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(i), X(current), voxelmap, subsampled_submaps[current]);
      }
    }
#ifdef GTSAM_POINTS_USE_CUDA
    else if (params.registration_error_factor_type == "VGICP_GPU") {
      const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
      const auto& stream = stream_buffer.first;
      const auto& buffer = stream_buffer.second;
      for (const auto& voxelmap : submaps[i]->voxelmaps) {
        factors->emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(i), X(current), voxelmap, subsampled_submaps[current], stream, buffer);
      }
    }
#endif
    else {
      logger->warn("unknown registration error type ({})", params.registration_error_factor_type);
    }
  }

  if (previous_overlap < std::max(0.25, params.min_implicit_loop_overlap)) {
    logger->warn("previous submap has only a small overlap with the current submap ({})", previous_overlap);
    logger->warn("create a between factor to prevent the submap from being isolated");
    const int last = current - 1;
    const gtsam::Pose3 init_delta = gtsam::Pose3((submaps[last]->T_world_origin.inverse() * submaps[current]->T_world_origin).matrix());
    factors->add(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), init_delta, gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
  }

  return factors;
}

void GlobalMapping::update_submaps() {
  for (int i = 0; i < submaps.size(); i++) {
    submaps[i]->T_world_origin = Eigen::Isometry3d(isam2->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
  }
}

gtsam_points::ISAM2ResultExt GlobalMapping::update_isam2(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values) {
  gtsam_points::ISAM2ResultExt result;

  gtsam::Key indeterminant_nearby_key = 0;
  try {
#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
    arena->execute([&] {
#endif
      result = isam2->update(new_factors, new_values);
#ifdef GTSAM_USE_TBB
    });
#endif
  } catch (const gtsam::IndeterminantLinearSystemException& e) {
    logger->error("an indeterminant linear system exception was caught during global map optimization!!");
    logger->error(e.what());
    indeterminant_nearby_key = e.nearbyVariable();
  } catch (const std::exception& e) {
    logger->error("an exception was caught during global map optimization!!");
    logger->error(e.what());
  }

  if (indeterminant_nearby_key != 0) {
    const gtsam::Symbol symbol(indeterminant_nearby_key);
    if (symbol.chr() == 'v' || symbol.chr() == 'b' || symbol.chr() == 'e') {
      indeterminant_nearby_key = X(symbol.index() / 2);
    }
    logger->warn("insert a damping factor at {} to prevent corruption", std::string(gtsam::Symbol(indeterminant_nearby_key)));

    gtsam::Values values = isam2->getLinearizationPoint();
    gtsam::NonlinearFactorGraph factors = isam2->getFactorsUnsafe();
    factors.emplace_shared<gtsam_points::LinearDampingFactor>(indeterminant_nearby_key, 6, 1e3);

    gtsam::ISAM2Params isam2_params;
    if (params.use_isam2_dogleg) {
      gtsam::ISAM2DoglegParams dogleg_params;
      isam2_params.setOptimizationParams(dogleg_params);
    }
    isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
    isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);

    if (params.enable_optimization) {
      isam2.reset(new gtsam_points::ISAM2Ext(isam2_params));
    } else {
      isam2.reset(new gtsam_points::ISAM2ExtDummy(isam2_params));
    }

    logger->warn("reset isam2");
    return update_isam2(factors, values);
  }

  return result;
}

void GlobalMapping::save(const std::string& path) {
  namespace fs = boost::filesystem;

  // A run can end while submaps are still buffered awaiting relocalization; flush them with the
  // current (best-effort) placement rather than silently dropping them from the saved map.
  if (!pending_submaps.empty()) {
    logger->warn("continuation: {} submaps still buffered awaiting relocalization at save; flushing with the current placement", pending_submaps.size());
    reloc_abandoned = true;
    for (const auto& pending : pending_submaps) {
      insert_submap_internal(pending);
    }
    pending_submaps.clear();
  }

  // In continuation mode the loaded prior map must never be clobbered: grow it into a distinct
  // sibling folder. Precedence: explicit save_map_path > requested path (if it does not resolve to
  // the loaded map) > "<loaded>_continued".
  std::string out_path = path;
  if (continuation_save_redirect && !params.continue_from_map_path.empty()) {
    boost::system::error_code ec;
    const auto same_dir = [&](const std::string& a, const std::string& b) {
      return fs::exists(a, ec) && fs::exists(b, ec) && fs::equivalent(a, b, ec);
    };

    if (!params.save_map_path.empty()) {
      out_path = params.save_map_path;
    } else if (path.empty() || same_dir(path, params.continue_from_map_path)) {
      out_path = params.continue_from_map_path;
      while (!out_path.empty() && (out_path.back() == '/' || out_path.back() == '\\')) {
        out_path.pop_back();
      }
      out_path += "_continued";
    }

    if (same_dir(out_path, params.continue_from_map_path)) {
      logger->error("continuation: refusing to overwrite the loaded prior map at {}; aborting save", params.continue_from_map_path);
      return;
    }
    logger->info("continuation: saving the grown map to {} (loaded prior map at {} left untouched)", out_path, params.continue_from_map_path);
  }

  optimize();

  fs::create_directories(out_path);

  gtsam::NonlinearFactorGraph serializable_factors;
  std::unordered_map<std::string, gtsam::NonlinearFactor::shared_ptr> matching_cost_factors;

  for (const auto& factor : isam2->getFactorsUnsafe()) {
    bool serializable = !dynamic_cast<gtsam_points::IntegratedMatchingCostFactor*>(factor.get())
#ifdef GTSAM_POINTS_USE_CUDA
                        && !dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(factor.get())
#endif
      ;

    if (serializable) {
      serializable_factors.push_back(factor);
    } else {
      const gtsam::Symbol symbol0(factor->keys()[0]);
      const gtsam::Symbol symbol1(factor->keys()[1]);
      const std::string key = std::to_string(symbol0.index()) + "_" + std::to_string(symbol1.index());

      matching_cost_factors[key] = factor;
    }
  }

  logger->info("serializing factor graph to {}/graph.bin", out_path);
  serializeToBinaryFile(serializable_factors, out_path + "/graph.bin");
  serializeToBinaryFile(isam2->calculateEstimate(), out_path + "/values.bin");

  std::ofstream ofs(out_path + "/graph.txt");
  ofs << "num_submaps: " << submaps.size() << std::endl;
  ofs << "num_all_frames: " << std::accumulate(submaps.begin(), submaps.end(), 0, [](int sum, const SubMap::ConstPtr& submap) { return sum + submap->frames.size(); }) << std::endl;

  ofs << "num_matching_cost_factors: " << matching_cost_factors.size() << std::endl;
  for (const auto& factor : matching_cost_factors) {
    std::string type;

    if (dynamic_cast<gtsam_points::IntegratedGICPFactor*>(factor.second.get())) {
      type = "gicp";
    } else if (dynamic_cast<gtsam_points::IntegratedVGICPFactor*>(factor.second.get())) {
      type = "vgicp";
    }
#ifdef GTSAM_POINTS_USE_CUDA
    else if (dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(factor.second.get())) {
      type = "vgicp_gpu";
    }
#endif

    gtsam::Symbol symbol0(factor.second->keys()[0]);
    gtsam::Symbol symbol1(factor.second->keys()[1]);
    ofs << "matching_cost " << type << " " << symbol0.index() << " " << symbol1.index() << std::endl;
  }

  std::ofstream odom_lidar_ofs(out_path + "/odom_lidar.txt");
  std::ofstream traj_lidar_ofs(out_path + "/traj_lidar.txt");

  std::ofstream odom_imu_ofs(out_path + "/odom_imu.txt");
  std::ofstream traj_imu_ofs(out_path + "/traj_imu.txt");

  const auto write_tum_frame = [](std::ofstream& ofs, const double stamp, const Eigen::Isometry3d& pose) {
    const Eigen::Quaterniond quat(pose.linear());
    const Eigen::Vector3d trans(pose.translation());
    ofs << boost::format("%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f") % stamp % trans.x() % trans.y() % trans.z() % quat.x() % quat.y() % quat.z() % quat.w() << std::endl;
  };

  for (int i = 0; i < submaps.size(); i++) {
    for (const auto& frame : submaps[i]->odom_frames) {
      write_tum_frame(odom_lidar_ofs, frame->stamp, frame->T_world_lidar);
      write_tum_frame(odom_imu_ofs, frame->stamp, frame->T_world_imu);
    }

    const Eigen::Isometry3d T_world_endpoint_L = submaps[i]->T_world_origin * submaps[i]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_odom_lidar0 = submaps[i]->frames.front()->T_world_lidar;
    const Eigen::Isometry3d T_odom_imu0 = submaps[i]->frames.front()->T_world_imu;

    for (const auto& frame : submaps[i]->frames) {
      const Eigen::Isometry3d T_world_imu = T_world_endpoint_L * T_odom_imu0.inverse() * frame->T_world_imu;
      const Eigen::Isometry3d T_world_lidar = T_world_imu * frame->T_lidar_imu.inverse();

      write_tum_frame(traj_imu_ofs, frame->stamp, T_world_imu);
      write_tum_frame(traj_lidar_ofs, frame->stamp, T_world_lidar);
    }

    submaps[i]->save((boost::format("%s/%06d") % out_path % i).str());
  }

  logger->info("saving config");
  GlobalConfig::instance()->dump(out_path + "/config");
}


gtsam_points::PointCloud::Ptr GlobalMapping::export_points() {
  auto merged = std::make_shared<gtsam_points::PointCloudCPU>();

  size_t total_points = 0;
  for (const auto& submap : submaps) {
    if (!submap || !submap->frame) {
      continue;
    }
    total_points += submap->frame->size();
  }

  std::vector<Eigen::Vector4d> points;
  points.reserve(total_points);

  bool export_intensities = true;
  for (const auto& submap : submaps) {
    if (!submap || !submap->frame || !submap->frame->has_intensities()) {
      export_intensities = false;
      break;
    }
  }

  std::vector<double> intensities;
  if (export_intensities) {
    intensities.reserve(total_points);
  }

  for (const auto& submap : submaps) {
    if (!submap || !submap->frame) {
      continue;
    }

    for (int i = 0; i < submap->frame->size(); i++) {
      const Eigen::Vector4d point = submap->T_world_origin * submap->frame->points[i];
      points.push_back(point);

      if (export_intensities) {
        intensities.push_back(submap->frame->intensities[i]);
      }
    }
  }

  if (!points.empty()) {
    merged->add_points(points);
    if (export_intensities) {
      merged->add_intensities(intensities);
    }
  }

  return merged;
}

bool GlobalMapping::load(const std::string& path) {
  std::ifstream ifs(path + "/graph.txt");
  if (!ifs) {
    logger->error("failed to open {}/graph.txt", path);
    return false;
  }

  const int start_from_frame_id = submaps.size();

  std::string token;
  int num_submaps, num_all_frames, num_matching_cost_factors;

  ifs >> token >> num_submaps;
  ifs >> token >> num_all_frames;
  ifs >> token >> num_matching_cost_factors;

  std::vector<std::tuple<std::string, int, int>> matching_cost_factors(num_matching_cost_factors);
  for (int i = 0; i < num_matching_cost_factors; i++) {
    auto& factor = matching_cost_factors[i];
    ifs >> token >> std::get<0>(factor) >> std::get<1>(factor) >> std::get<2>(factor);
  }

  logger->info("Load submaps (session_id={})", session_id);
  submaps.reserve(submaps.size() + num_submaps);
  subsampled_submaps.reserve(submaps.size() + num_submaps);
  for (int i = 0; i < num_submaps; i++) {
    auto submap = SubMap::load((boost::format("%s/%06d") % path % i).str());
    if (!submap) {
      return false;
    }
    submap->id += start_from_frame_id;
    submap->session_id = session_id;

    // Adaptively determine the voxel resolution based on the median distance
    const int max_scan_count = 256;
    const double dist_median = gtsam_points::median_distance(submap->frame, max_scan_count);
    const double p =
      std::max(0.0, std::min(1.0, (dist_median - params.submap_voxel_resolution_dmin) / (params.submap_voxel_resolution_dmax - params.submap_voxel_resolution_dmin)));
    const double base_resolution = params.submap_voxel_resolution + p * (params.submap_voxel_resolution_max - params.submap_voxel_resolution);

    gtsam_points::PointCloud::Ptr subsampled_submap;
    if (params.randomsampling_rate > 0.99) {
      subsampled_submap = submap->frame;
    } else {
      subsampled_submap = gtsam_points::random_sampling(submap->frame, params.randomsampling_rate, mt);
    }

    submaps.push_back(submap);
    submaps.back()->voxelmaps.clear();
    subsampled_submaps.push_back(subsampled_submap);

    if (params.enable_gpu) {
#ifdef GTSAM_POINTS_USE_CUDA
      subsampled_submaps.back() = gtsam_points::PointCloudGPU::clone(*subsampled_submaps.back());

      for (int j = 0; j < params.submap_voxelmap_levels; j++) {
        const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, j);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution);
        voxelmap->insert(*subsampled_submaps.back());
        submaps.back()->voxelmaps.push_back(voxelmap);
      }
#else
      logger->warn("GPU is enabled for global_mapping but gtsam_points was built without CUDA!!");
#endif
    } else {
      for (int j = 0; j < params.submap_voxelmap_levels; j++) {
        const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, j);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
        voxelmap->insert(*subsampled_submaps.back());
        submaps.back()->voxelmaps.push_back(voxelmap);
      }
    }

    Callbacks::on_insert_submap(submap);
  }

  gtsam::Values values, loaded_values;
  gtsam::NonlinearFactorGraph graph, loaded_graph;
  bool needs_recover = false;

  logger->info("deserializing factor graph");
  try {
    gtsam::deserializeFromBinaryFile(path + "/graph.bin", loaded_graph);
  } catch (boost::archive::archive_exception e) {
    logger->error("failed to deserialize factor graph!!");
    logger->error(e.what());
  } catch (std::exception& e) {
    logger->error("failed to deserialize factor graph!!");
    logger->error(e.what());
    needs_recover = true;
  }

  logger->info("deserializing values");
  try {
    gtsam::deserializeFromBinaryFile(path + "/values.bin", loaded_values);
  } catch (boost::archive::archive_exception e) {
    logger->error("failed to deserialize values!!");
    logger->error(e.what());
  } catch (std::exception& e) {
    logger->error("failed to deserialize values!!");
    logger->error(e.what());
    needs_recover = true;
  }

  // remap keys in graph and values if dump previously loaded
  if (start_from_frame_id > 0) {
    std::map<gtsam::Key, gtsam::Key> rekey_mapping;
    for (int i = 0; i < num_submaps; i++) {
      rekey_mapping[X(i)] = X(i + start_from_frame_id);
      rekey_mapping[E(i * 2)] = E((i + start_from_frame_id) * 2);
      rekey_mapping[E(i * 2 + 1)] = E((i + start_from_frame_id) * 2 + 1);
      rekey_mapping[B(i * 2)] = B((i + start_from_frame_id) * 2);
      rekey_mapping[B(i * 2 + 1)] = B((i + start_from_frame_id) * 2 + 1);
      rekey_mapping[V(i * 2)] = V((i + start_from_frame_id) * 2);
      rekey_mapping[V(i * 2 + 1)] = V((i + start_from_frame_id) * 2 + 1);
    }

    logger->info("removing translation prior factors");
    auto remove_loc = std::remove_if(loaded_graph.begin(), loaded_graph.end(), [](const auto& factor) {
      return dynamic_cast<gtsam::PoseTranslationPrior<gtsam::Pose3>*>(factor.get()) != nullptr;
    });
    logger->info("removed {} prior factors", std::distance(remove_loc, loaded_graph.end()));
    loaded_graph.erase(remove_loc, loaded_graph.end());

    logger->info("removing damping factors");
    remove_loc =
      std::remove_if(loaded_graph.begin(), loaded_graph.end(), [](const auto& factor) { return dynamic_cast<gtsam_points::LinearDampingFactor*>(factor.get()) != nullptr; });
    logger->info("removed {} prior factors", std::distance(remove_loc, loaded_graph.end()));
    loaded_graph.erase(remove_loc, loaded_graph.end());

    logger->info("removing prior factors");
    remove_loc =
      std::remove_if(loaded_graph.begin(), loaded_graph.end(), [](const auto& factor) { return dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factor.get()) != nullptr; });
    logger->info("removed {} prior factors", std::distance(remove_loc, loaded_graph.end()));
    loaded_graph.erase(remove_loc, loaded_graph.end());

    // rekey graph
    graph = loaded_graph;
    logger->info("rekeying factors");
    graph = graph.rekey(rekey_mapping);

    // rekey values
    for (auto it = loaded_values.begin(); it != loaded_values.end(); ++it) {
      auto matched_key = rekey_mapping.find(it->key);
      if (matched_key != rekey_mapping.end()) {
        values.insert(matched_key->second, it->value);
      } else {
        logger->warn("No remapping found for Value with key {}, keeping it as is", gtsam::Symbol(it->key).string());
        values.insert(it->key, it->value);
      }
    }
  } else {
    graph = loaded_graph;
    values = loaded_values;
  }

  logger->info("creating matching cost factors");
  for (const auto& factor : matching_cost_factors) {
    const auto type = std::get<0>(factor);
    const auto first = std::get<1>(factor) + start_from_frame_id;
    const auto second = std::get<2>(factor) + start_from_frame_id;

    if (type == "vgicp" || type == "vgicp_gpu") {
      if (params.enable_gpu) {
#ifdef GTSAM_POINTS_USE_CUDA
        const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
        const auto& stream = stream_buffer.first;
        const auto& buffer = stream_buffer.second;

        for (const auto& voxelmap : submaps[first]->voxelmaps) {
          graph.emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(first), X(second), voxelmap, subsampled_submaps[second], stream, buffer);
        }
#else
        logger->warn("GPU is enabled but gtsam_points was built without CUDA!!");
#endif
      } else {
        for (const auto& voxelmap : submaps[first]->voxelmaps) {
          graph.emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(first), X(second), voxelmap, subsampled_submaps[second]);
        }
      }
    } else {
      logger->warn("unsupported matching cost factor type ({})", type);
    }
  }

  // Pin the loaded submaps to their saved poses so the certified prior map stays put and acts as
  // drift-free truth (new-session submaps register against it rather than dragging it around). We
  // stripped the map's own damping/translation/pose priors above (they encode the old gauge); here we
  // re-anchor each loaded submap to the pose it was saved at with a strong 6-DOF prior.
  if (start_from_frame_id > 0 && params.freeze_loaded_map) {
    const auto freeze_noise = gtsam::noiseModel::Isotropic::Precision(6, params.freeze_prior_precision);
    int num_pinned = 0;
    for (int i = 0; i < num_submaps; i++) {
      const gtsam::Key key = X(i + start_from_frame_id);
      if (!values.exists(key)) {
        continue;
      }
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(key, values.at<gtsam::Pose3>(key), freeze_noise);
      num_pinned++;
    }
    logger->info("continuation: pinned {} loaded submaps to their saved poses (precision {:.0e}) -> prior map is frozen truth", num_pinned, params.freeze_prior_precision);
  }

  const size_t num_factors_before = graph.size();
  const auto remove_loc = std::remove_if(graph.begin(), graph.end(), [](const auto& factor) { return factor == nullptr; });
  graph.erase(remove_loc, graph.end());
  if (graph.size() != num_factors_before) {
    logger->warn("removed {} invalid factors", num_factors_before - graph.size());
    needs_recover = true;
  }

  if (needs_recover) {
    logger->warn("recovering factor graph");
    const auto recovered = recover_graph(graph, values, start_from_frame_id);
    logger->warn("add {} factors and {} values", recovered.first.size(), recovered.second.size());

    graph.add(recovered.first);
    values.insert_or_assign(recovered.second);
  }

  if (start_from_frame_id <= 0) {
    logger->info("optimize");
    Callbacks::on_smoother_update(*isam2, graph, values);
    auto result = update_isam2(graph, values);
    Callbacks::on_smoother_update_result(*isam2, result);

    update_submaps();
    Callbacks::on_update_submaps(submaps);
  } else {
    logger->info("skip optimization");
    this->new_factors->add(graph);
    this->new_values->insert(values);
  }

  logger->info("done");
  session_id++;

  return true;
}

void GlobalMapping::recover_graph() {
  const auto recovered = recover_graph(isam2->getFactorsUnsafe(), isam2->calculateEstimate(), 0);
  update_isam2(recovered.first, recovered.second);
}

// Recover the graph by adding missing values and factors
std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> GlobalMapping::recover_graph(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, int start_from_frame_id)
  const {
  logger->info("recovering graph");
  bool enable_imu = false;
  for (const auto& value : values) {
    const char chr = gtsam::Symbol(value.key).chr();
    enable_imu |= (chr == 'e' || chr == 'v' || chr == 'b');
  }
  for (const auto& factor : graph) {
    enable_imu |= dynamic_cast<gtsam::ImuFactor*>(factor.get()) != nullptr;
  }

  logger->info("enable_imu={}", enable_imu);

  logger->info("creating connectivity map");
  bool prior_exists = false;
  std::unordered_map<gtsam::Key, std::set<gtsam::Key>> connectivity_map;
  for (const auto& factor : graph) {
    if (!factor) {
      continue;
    }

    for (const auto key : factor->keys()) {
      for (const auto key2 : factor->keys()) {
        connectivity_map[key].insert(key2);
      }
    }

    if (factor->keys().size() == 1 && factor->keys()[0] == X(0)) {
      prior_exists |= dynamic_cast<gtsam_points::LinearDampingFactor*>(factor.get()) != nullptr;
    }
  }

  logger->info("fixing missing values and factors");
  const auto prior_noise3 = gtsam::noiseModel::Isotropic::Precision(3, 1e6);
  const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  if (!prior_exists) {
    logger->warn("X0 prior is missing");
    new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params.init_pose_damping_scale);
  }

  for (int i = start_from_frame_id; i < submaps.size(); i++) {
    if (!values.exists(X(i))) {
      logger->warn("X{} is missing", i);
      new_values.insert(X(i), gtsam::Pose3(submaps[i]->T_world_origin.matrix()));
    }

    if (connectivity_map[X(i)].count(X(i + 1)) == 0 && i != submaps.size() - 1) {
      logger->warn("X{} -> X{} is missing", i, i + 1);

      const Eigen::Isometry3d delta = submaps[i]->origin_odom_frame()->T_world_sensor().inverse() * submaps[i + 1]->origin_odom_frame()->T_world_sensor();
      new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i), X(i + 1), gtsam::Pose3(delta.matrix()), prior_noise6);
    }

    if (!enable_imu) {
      continue;
    }

    const auto submap = submaps[i];
    const gtsam::imuBias::ConstantBias imu_biasL(submap->frames.front()->imu_bias);
    const gtsam::imuBias::ConstantBias imu_biasR(submap->frames.back()->imu_bias);
    const Eigen::Vector3d v_origin_imuL = submap->T_world_origin.linear().inverse() * submap->frames.front()->v_world_imu;
    const Eigen::Vector3d v_origin_imuR = submap->T_world_origin.linear().inverse() * submap->frames.back()->v_world_imu;

    if (i != 0) {
      if (!values.exists(E(i * 2))) {
        logger->warn("E{} is missing", i * 2);
        new_values.insert(E(i * 2), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_L).matrix()));
      }
      if (!values.exists(V(i * 2))) {
        logger->warn("V{} is missing", i * 2);
        new_values.insert(V(i * 2), (submap->T_world_origin.linear() * v_origin_imuL).eval());
      }
      if (!values.exists(B(i * 2))) {
        logger->warn("B{} is missing", i * 2);
        new_values.insert(B(i * 2), imu_biasL);
      }

      if (connectivity_map[X(i)].count(E(i * 2)) == 0) {
        logger->warn("X{} -> E{} is missing", i, i * 2);
        new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i), E(i * 2), gtsam::Pose3(submap->T_origin_endpoint_L.matrix()), prior_noise6);
      }
      if (connectivity_map[X(i)].count(V(i * 2)) == 0) {
        logger->warn("X{} -> V{} is missing", i, i * 2);
        new_factors.emplace_shared<gtsam_points::RotateVector3Factor>(X(i), V(i * 2), v_origin_imuL, prior_noise3);
      }
      if (connectivity_map[B(i * 2)].count(B(i * 2)) == 0) {
        logger->warn("B{} -> B{} is missing", i * 2, i * 2);
        new_factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(i * 2), imu_biasL, prior_noise6);
      }

      if (connectivity_map[B(i * 2)].count(B(i * 2 + 1)) == 0) {
        logger->warn("B{} -> B{} is missing", i * 2, i * 2 + 1);
        new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(i * 2), B(i * 2 + 1), gtsam::imuBias::ConstantBias(), prior_noise6);
      }
    }

    if (!values.exists(E(i * 2 + 1))) {
      logger->warn("E{} is missing", i * 2 + 1);
      new_values.insert(E(i * 2 + 1), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_R).matrix()));
    }
    if (!values.exists(V(i * 2 + 1))) {
      logger->warn("V{} is missing", i * 2 + 1);
      new_values.insert(V(i * 2 + 1), (submap->T_world_origin.linear() * v_origin_imuR).eval());
    }
    if (!values.exists(B(i * 2 + 1))) {
      logger->warn("B{} is missing", i * 2 + 1);
      new_values.insert(B(i * 2 + 1), imu_biasR);
    }

    if (connectivity_map[X(i)].count(E(i * 2 + 1)) == 0) {
      logger->warn("X{} -> E{} is missing", i, i * 2 + 1);
      new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i), E(i * 2 + 1), gtsam::Pose3(submap->T_origin_endpoint_R.matrix()), prior_noise6);
    }
    if (connectivity_map[X(i)].count(V(i * 2 + 1)) == 0) {
      logger->warn("X{} -> V{} is missing", i, i * 2 + 1);
      new_factors.emplace_shared<gtsam_points::RotateVector3Factor>(X(i), V(i * 2 + 1), v_origin_imuR, prior_noise3);
    }
    if (connectivity_map[B(i * 2 + 1)].count(B(i * 2 + 1)) == 0) {
      logger->warn("B{} -> B{} is missing", i * 2 + 1, i * 2 + 1);
      new_factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(i * 2 + 1), imu_biasR, prior_noise6);
    }
  }

  logger->info("recovering done");

  return {new_factors, new_values};
}

}  // namespace glim
