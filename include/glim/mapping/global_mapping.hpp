#pragma once

#include <any>
#include <memory>
#include <random>
#include <glim/mapping/global_mapping_base.hpp>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class ISAM2Ext;
class StreamTempBufferRoundRobin;
class GaussianVoxelMapCPU;
class GaussianVoxelMapGPU;
class PointCloudCPU;
struct ISAM2ResultExt;
}  // namespace gtsam_points

namespace glim {

class IMUIntegration;
class MapAligner;

/**
 * @brief Global mapping parameters
 */
struct GlobalMappingParams {
public:
  GlobalMappingParams();
  ~GlobalMappingParams();

public:
  bool enable_gpu;
  bool enable_imu;
  bool enable_optimization;
  bool enable_between_factors;
  std::string between_registration_type;

  // --- Session continuation ("as if GLIM was never turned off") ---
  // If non-empty, load this saved-map directory at startup and keep mapping onto it: the loaded
  // submaps come back as optimizable variables, the new session is relocalized onto them (FPFH +
  // RANSAC/GNC on the first new submap), and normal overlap-based loop closure ties old and new
  // poses together. The loaded map is NEVER overwritten -- save() redirects to a sibling folder.
  std::string continue_from_map_path;   ///< Prior map dir to load and extend ("" disables continuation)
  std::string save_map_path;            ///< Explicit output dir for the grown map ("" => "<loaded>_continued")

  // Loaded-map rigidity. When true (default), every loaded submap is pinned to its saved pose with a
  // strong pose prior, so the certified prior map stays put and acts as drift-free truth: new-session
  // submaps register AGAINST it (their live pose is corrected toward the fixed map) instead of the map
  // drifting to co-optimize with the new session. When false, the loaded submaps are fully free
  // variables and can shift under new loop closures (the original "True SLAM-continue" behavior).
  bool freeze_loaded_map;               ///< Pin loaded submaps to their saved poses (default true)
  double freeze_prior_precision;        ///< Isotropic precision of the per-submap pin prior (1e10 ~= rigid, 1e6 ~= reasonably fixed)

  // Whether to APPLY the continuation relocalization (re-anchor the new session onto the prior map).
  // Derived from glim_ros/publish_tf in config_ros: true on the real robot (GLIM owns the TF and the
  // re-anchor is real), false in "sim mode" where an external source (Isaac Sim) owns the TF. When
  // false, the relocalization is still COMPUTED and REPORTED (on ~/map_offset + logs) so it can be
  // tested byte-for-byte against different spawn positions, but is NOT applied — the map keeps
  // piggybacking the simulator's ground-truth odom frame.
  bool apply_relocalization;

  // Relocalization (new-session -> prior-map) registration + acceptance gates.
  double reloc_voxel_resolution;        ///< Voxel downsample for map + source before FPFH
  double reloc_fpfh_radius;             ///< FPFH feature search radius [m]
  int reloc_dof;                        ///< 4 (XYZ+yaw) or 6 (full SE3) for the global registration
  double reloc_search_radius_m;         ///< Reject a solution whose translation exceeds this (the spawn radius)
  double reloc_start_area_radius_m;     ///< If >0, build the FPFH target from ONLY the map points within this radius of the map start (submap 0). Kills global self-similarity; use when the robot always reboots near where the prior run began. <=0 => whole-map (global) target.
  double reloc_min_inlier_rate;         ///< Reject a coarse (FPFH) solution below this RANSAC/GNC inlier rate
  std::string reloc_registration;       ///< "RANSAC" (default; falls back to GNC) or "GNC"
  int num_threads;                      ///< Threads for the relocalization registration

  // Relocalization method. "yaw_sweep" (default): exploit the near-start prior directly — anchor the
  // first new submap onto the map's first submap and sweep yaw hypotheses, VGICP-refining each and
  // keeping the best. No FPFH feature dependence; ideal when the robot always reboots near the prior
  // start at an unknown heading. "fpfh": global FPFH+RANSAC/GNC coarse fix then VGICP refine.
  std::string reloc_method;             ///< "yaw_sweep" (default) or "fpfh"
  double reloc_yaw_step_deg;            ///< Yaw hypothesis spacing for the sweep [deg] (smaller = safer, more candidates)

  // Relocalization robustness: new submaps are buffered (kept out of the graph) until a confident
  // fix, retrying with the ACCUMULATED cloud as each submap arrives (a moving start only grows the
  // source geometry). A coarse FPFH fix is then fine-refined with VGICP against the prior map and
  // gated on the refine result, which is far more discriminative than the raw FPFH inlier rate.
  int reloc_max_submaps;                ///< Give up (identity fallback + damping anchor) after this many buffered submaps / failed attempts
  bool reloc_refine;                    ///< Fine-refine the coarse FPFH pose with VGICP before accepting it
  double reloc_refine_max_correction;   ///< Reject a refinement that moves more than this from the coarse pose [m]
  double reloc_refine_min_inlier_rate;  ///< Reject a refinement whose VGICP inlier fraction is below this

  std::string registration_error_factor_type;
  double submap_voxel_resolution;
  double submap_voxel_resolution_max;
  double submap_voxel_resolution_dmin;
  double submap_voxel_resolution_dmax;
  int submap_voxelmap_levels;
  double submap_voxelmap_scaling_factor;

  double randomsampling_rate;
  double max_implicit_loop_distance;
  double min_implicit_loop_overlap;

  bool use_isam2_dogleg;
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;

  double init_pose_damping_scale;
};

/**
 * @brief Global mapping
 */
class GlobalMapping : public GlobalMappingBase {
public:
  GlobalMapping(const GlobalMappingParams& params = GlobalMappingParams());
  virtual ~GlobalMapping();

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual void insert_submap(const SubMap::Ptr& submap) override;

  virtual void find_overlapping_submaps(double min_overlap) override;
  virtual void optimize() override;

  /// @brief Consume a waiting operator relocalization override and accept it immediately (bypassing
  ///        the inlier gate), without waiting for the next submap to drive an attempt. No-op unless
  ///        a continuation is in progress and unresolved.
  virtual void apply_pending_reloc_override() override;

  virtual void save(const std::string& path) override;
  virtual gtsam_points::PointCloud::Ptr export_points() override;

  /**
   * @brief Load a mapping result from a dumped directory
   * @param path Input dump path
   */
  bool load(const std::string& path) override;

private:
  void insert_submap(int current, const SubMap::Ptr& submap);

  /// @brief Actual graph insertion of a submap (the pre-continuation insert_submap body).
  ///        In continuation mode this is only reached once relocalization has succeeded or been
  ///        abandoned; the public insert_submap buffers submaps until then.
  void insert_submap_internal(const SubMap::Ptr& submap);

  /// @brief Perform the deferred prior-map load (continuation mode) on the first runtime data call,
  ///        so the loaded submaps' on_insert_submap callbacks reach an already-subscribed viewer.
  void ensure_prior_map_loaded();

  /// @brief Fallback when the new session cannot be aligned to the loaded map: discard the loaded map
  ///        from the active graph (the original on disk is untouched) and restart as a fresh mapping
  ///        session. save() still redirects the fresh map into the continuation output folder.
  void reset_to_fresh_mapping(bool relocalization_failed = true);

  /// @brief FPFH+RANSAC/GNC-register the ACCUMULATED pending-submap cloud onto the prior map, then
  ///        (optionally) VGICP-fine-refine and gate the result. On success sets T_map_odom and
  ///        `relocalized`; after reloc_max_submaps failures sets `reloc_abandoned`.
  void try_relocalize_pending();

  /// @brief Accumulate all buffered pending submaps into one odom-frame source cloud (decimated per
  ///        submap). Empty if nothing is buffered yet.
  std::vector<Eigen::Vector4d> accumulate_pending_source() const;

  /// @brief Post-attempt bookkeeping: fall back to fresh mapping if the attempt abandoned/report-only'd,
  ///        then flush the buffered submaps into the graph once the continuation has resolved. MUST be
  ///        called by every path that can resolve a continuation — the submap path and the operator
  ///        override alike — or the buffered submaps are stranded.
  void settle_relocalization_outcome();

  /// @brief Commit an accepted alignment: report it (map_offset / progress "done" / preview) and, on the
  ///        real robot, re-anchor the session onto the prior map. Shared by the autonomous success path
  ///        and the operator override so both emit identical downstream state. `method` is log-only.
  void accept_relocalization(const std::vector<Eigen::Vector4d>& src, const Eigen::Isometry3d& correction, int attempt, const std::string& method);

  /// @brief Accept an operator-supplied alignment (see RelocOverride) UNCONDITIONALLY — the inlier gate
  ///        is bypassed. Optionally VGICP-refines from their pose, keeping the operator's pose verbatim
  ///        if the refine misses its trust gates. Never fails, never abandons.
  void apply_operator_override(const std::vector<Eigen::Vector4d>& src, const Eigen::Isometry3d& T_override, int attempt);

  /// @brief VGICP fine-refinement of the coarse FPFH pose against the prior-map refine voxelmaps.
  ///        Updates T_map_odom_coarse in place; false = rejected (gates: max correction, inlier rate).
  bool refine_relocalization(const std::vector<Eigen::Vector4d>& src_points, Eigen::Isometry3d& T_map_odom_coarse);

  /// @brief Yaw-sweep relocalizer: anchor the first pending submap onto the map's first submap, sweep
  ///        yaw hypotheses about that anchor, VGICP-refine each against the full-map voxelmaps, and
  ///        keep the highest-inlier fit that lands within the spawn radius. Sets out_T; false = none passed.
  bool relocalize_yaw_sweep(const std::vector<Eigen::Vector4d>& src_points, Eigen::Isometry3d& out_T, double* out_best_inlier = nullptr);

  /// @brief VGICP-fit `src` (odom frame) to the full-map voxelmaps from initial guess `T_init`.
  ///        Returns the refined pose and its finest-level inlier fraction.
  std::pair<Eigen::Isometry3d, double> vgicp_fit(const std::shared_ptr<gtsam_points::PointCloudCPU>& src, const Eigen::Isometry3d& T_init, int max_iterations) const;

  std::shared_ptr<gtsam::NonlinearFactorGraph> create_between_factors(int current) const;
  std::shared_ptr<gtsam::NonlinearFactorGraph> create_matching_cost_factors(int current) const;

  void update_submaps();
  gtsam_points::ISAM2ResultExt update_isam2(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values);

  void recover_graph() override;
  std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> recover_graph(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, int start_from_frame_id) const;

private:
  using Params = GlobalMappingParams;
  Params params;

  std::mt19937 mt;
  int session_id;

  // --- Session continuation state (all touched only on the global-mapping thread) ---
  bool prior_map_load_done = false;               ///< The deferred prior-map load has run
  bool continuation_mode = false;                 ///< A prior map was loaded and is being extended
  int continuation_start_id = -1;                 ///< id of the first NEW submap (= number of loaded submaps)
  bool relocalized = false;                        ///< The new session was successfully aligned onto the prior map
  bool reloc_abandoned = false;                    ///< Too many failed attempts (consumed by the fresh-mapping fallback)
  bool reloc_report_only_done = false;             ///< Sim mode: alignment computed + reported, not applied (consumed by the fresh-mapping transition)
  bool continuation_save_redirect = false;         ///< A continuation was attempted; save() still redirects to the continue folder even after a fresh-mapping fallback
  Eigen::Isometry3d T_map_odom = Eigen::Isometry3d::Identity();  ///< prior-map <- new-session odom/world
  std::shared_ptr<MapAligner> aligner;            ///< FPFH global-alignment target built from the loaded map
  std::vector<SubMap::Ptr> pending_submaps;       ///< New submaps buffered (out of the graph) until relocalization resolves
  std::vector<std::shared_ptr<gtsam_points::GaussianVoxelMapCPU>> reloc_refine_voxelmaps;  ///< Map-frame VGICP targets for the fine refinement (from the same cloud as the FPFH target)
  std::vector<std::shared_ptr<gtsam_points::GaussianVoxelMapGPU>> reloc_refine_voxelmaps_gpu;  ///< GPU mirror of reloc_refine_voxelmaps (built + used only when params.enable_gpu on a CUDA build)

  std::unique_ptr<IMUIntegration> imu_integration;
  std::any stream_buffer_roundrobin;

  std::vector<SubMap::Ptr> submaps;
  std::vector<gtsam_points::PointCloud::ConstPtr> subsampled_submaps;

  std::unique_ptr<gtsam::Values> new_values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> new_factors;

  std::unique_ptr<gtsam_points::ISAM2Ext> isam2;

  std::shared_ptr<void> tbb_task_arena;
};
}  // namespace glim
