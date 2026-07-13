#include <glim/odometry/map_alignment.hpp>

#include <cstdio>
#include <random>
#include <filesystem>

#include <spdlog/spdlog.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/ann/kdtreex.hpp>
#include <gtsam_points/features/normal_estimation.hpp>
#include <gtsam_points/features/fpfh_estimation.hpp>
#include <gtsam_points/registration/ransac.hpp>
#include <gtsam_points/registration/graduated_non_convexity.hpp>

#include <glim/util/logging.hpp>
#include <glim/mapping/sub_map.hpp>

namespace glim {

namespace {
std::shared_ptr<spdlog::logger> or_default(std::shared_ptr<spdlog::logger> logger) {
  return logger ? logger : create_module_logger("map_align");
}
}  // namespace

MapAligner::MapAligner(const std::string& map_path, const MapAlignerParams& params, std::shared_ptr<spdlog::logger> logger) {
  auto log = or_default(logger);
  auto cloud = load_map_cloud(map_path, log);
  if (!cloud || cloud->size() == 0) {
    log->warn("map alignment target unavailable: no points loaded from '{}'", map_path);
    return;
  }
  build(cloud, params, log);
}

MapAligner::MapAligner(const std::shared_ptr<gtsam_points::PointCloudCPU>& map_cloud, const MapAlignerParams& params, std::shared_ptr<spdlog::logger> logger) {
  auto log = or_default(logger);
  if (!map_cloud || map_cloud->size() == 0) {
    log->warn("map alignment target unavailable: empty map cloud");
    return;
  }
  build(map_cloud, params, log);
}

MapAligner::~MapAligner() {}

bool MapAligner::valid() const {
  return target_ && target_tree_ && target_fpfh_tree_ && !target_fpfh_.empty();
}

size_t MapAligner::target_size() const {
  return target_ ? target_->size() : 0;
}

std::shared_ptr<gtsam_points::PointCloudCPU> MapAligner::load_map_cloud(const std::string& map_path, std::shared_ptr<spdlog::logger> logger) {
  namespace fs = std::filesystem;
  auto log = or_default(logger);

  if (map_path.empty() || !fs::is_directory(map_path)) {
    log->warn("prior map path '{}' is not a directory (expected a GlobalMapping::save dump)", map_path);
    return nullptr;
  }

  std::vector<Eigen::Vector4d> all_points;
  int num_submaps = 0;

  // Iterate submap subdirectories 000000, 000001, ... until one fails to load.
  for (int i = 0;; i++) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "/%06d", i);
    const std::string submap_path = map_path + buffer;
    if (!fs::is_directory(submap_path)) {
      break;
    }

    SubMap::Ptr submap = SubMap::load(submap_path);
    if (!submap || !submap->frame) {
      log->warn("failed to load submap {}; stopping prior-map load", submap_path);
      break;
    }

    auto transformed = gtsam_points::transform(submap->frame, submap->T_world_origin);
    all_points.insert(all_points.end(), transformed->points, transformed->points + transformed->size());
    num_submaps++;
  }

  if (num_submaps == 0 || all_points.empty()) {
    log->warn("no submaps loaded from '{}'", map_path);
    return nullptr;
  }

  log->info("loaded prior map for alignment: {} submaps, {} points", num_submaps, all_points.size());
  return std::make_shared<gtsam_points::PointCloudCPU>(all_points);
}

void MapAligner::build(const std::shared_ptr<gtsam_points::PointCloudCPU>& map_cloud, const MapAlignerParams& params, const std::shared_ptr<spdlog::logger>& logger) {
  auto tgt = gtsam_points::voxelgrid_sampling(map_cloud, params.voxel_resolution, params.num_threads);
  if (!tgt || tgt->size() < 100) {
    logger->warn("map alignment target too small ({} points after downsample); alignment disabled", tgt ? tgt->size() : 0);
    return;
  }
  tgt->add_normals(gtsam_points::estimate_normals(tgt->points, tgt->size(), 10, params.num_threads));

  target_ = tgt;
  target_tree_ = std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(target_);

  gtsam_points::FPFHEstimationParams fpfh_params;
  fpfh_params.search_radius = params.fpfh_radius;
  fpfh_params.num_threads = params.num_threads;
  target_fpfh_ = gtsam_points::estimate_fpfh(*target_, *target_tree_, fpfh_params);
  target_fpfh_tree_ = std::make_shared<gtsam_points::KdTreeX<33>>(target_fpfh_.data(), target_fpfh_.size());

  logger->info("map alignment target ready: {} feature points (voxel {:.2f} m, fpfh r={:.2f} m, dof={})", target_->size(), params.voxel_resolution, params.fpfh_radius, params.dof);
}

MapAlignmentResult MapAligner::align(const std::vector<Eigen::Vector4d>& source_points, const MapAlignerParams& params) const {
  MapAlignmentResult out;
  if (!valid()) {
    return out;
  }

  auto src = std::make_shared<gtsam_points::PointCloudCPU>(source_points);
  src = gtsam_points::voxelgrid_sampling(src, params.voxel_resolution, params.num_threads);
  if (!src || src->size() < 100) {
    return out;
  }
  src->add_normals(gtsam_points::estimate_normals(src->points, src->size(), 10, params.num_threads));
  gtsam_points::KdTree2<gtsam_points::PointCloud> src_tree(src);

  gtsam_points::FPFHEstimationParams fpfh_params;
  fpfh_params.search_radius = params.fpfh_radius;
  fpfh_params.num_threads = params.num_threads;
  const auto src_fpfh = gtsam_points::estimate_fpfh(*src, src_tree, fpfh_params);
  gtsam_points::KdTreeX<33> src_fpfh_tree(src_fpfh.data(), src_fpfh.size());

  // Gate: the spawn is assumed within `search_radius_m` of the map origin, at any yaw.
  auto passes = [&](const gtsam_points::RegistrationResult& r) {
    return r.inlier_rate >= params.min_inlier_rate && r.T_target_source.translation().norm() <= params.search_radius_m;
  };

  std::mt19937 rng(12345u);
  gtsam_points::RegistrationResult result;
  bool ok = false;

  if (params.registration != "GNC") {
    gtsam_points::RANSACParams rp;
    rp.dof = params.dof;
    rp.num_threads = params.num_threads;
    rp.seed = rng();
    result = gtsam_points::estimate_pose_ransac(*target_, *src, target_fpfh_.data(), src_fpfh.data(), *target_tree_, *target_fpfh_tree_, rp);
    ok = passes(result);
  }
  if (!ok) {  // GNC (Fast Global Registration), primary or RANSAC fallback
    gtsam_points::GNCParams gp;
    gp.dof = params.dof;
    gp.num_threads = params.num_threads;
    gp.seed = rng();
    const auto gnc = gtsam_points::estimate_pose_gnc(*target_, *src, target_fpfh_.data(), src_fpfh.data(), *target_tree_, *target_fpfh_tree_, src_fpfh_tree, gp);
    if (passes(gnc)) {
      result = gnc;
      ok = true;
    }
  }

  out.success = ok;
  out.T_target_source = result.T_target_source;
  out.inlier_rate = result.inlier_rate;
  return out;
}

}  // namespace glim
