#pragma once

#include <string>

#include <glim/util/callback_slot.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>

#ifdef GLIM_USE_OPENCV
namespace cv {
class Mat;
}
#endif

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class ISAM2Ext;
class ISAM2ResultExt;
class LevenbergMarquardtOptimizationStatus;
}  // namespace gtsam_points

namespace glim {

/**
 * @brief Sub mapping-related callbacks
 *
 */
struct SubMappingCallbacks {
#ifdef GLIM_USE_OPENCV
  /**
   * @brief Image input callback
   * @param stamp  Timestamp
   * @param image  Image
   */
  static CallbackSlot<void(const double stamp, const cv::Mat& image)> on_insert_image;
#endif

  /**
   * @brief IMU input callback
   * @param stamp         Timestamp
   * @param linear_acc    Linear acceleration
   * @param angular_vel   Angular velocity
   */
  static CallbackSlot<void(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel)> on_insert_imu;

  /**
   * @brief Odometry estimation result input callback
   * @param frame  Marginalized odometry estimation frame
   */
  static CallbackSlot<void(const EstimationFrame::ConstPtr& frame)> on_insert_frame;

  /**
   * @brief SubMap keyframe creation callback
   * @param id        SubMap ID
   * @param keyframe  New keyframe
   */
  static CallbackSlot<void(int id, const EstimationFrame::ConstPtr& keyframe)> on_new_keyframe;

  /**
   * @brief SubMap optimization callback (just before optimization)
   * @param graph   Factor graph
   * @param values  Values to be optimized
   */
  static CallbackSlot<void(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values)> on_optimize_submap;

  /**
   * @brief Optimization status callback
   * @param status  Optimization status
   * @param values  Current estimate
   */
  static CallbackSlot<void(const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values)> on_optimization_status;

  /**
   * @brief SubMap creation callback
   * @param submap  New SubMap
   */
  static CallbackSlot<void(const SubMap::ConstPtr& submap)> on_new_submap;
};

/**
 * @brief Global mapping-related callbacks
 *
 */
struct GlobalMappingCallbacks {
#ifdef GLIM_USE_OPENCV
  /**
   * @brief Image input callback
   * @param stamp  Timestamp
   * @param image  Image
   */
  static CallbackSlot<void(const double stamp, const cv::Mat& image)> on_insert_image;
#endif

  /**
   * @brief IMU input callback
   * @param stamp        Timestamp
   * @param linear_acc   Linear acceleration
   * @param angular_vel  Angular velocity
   */
  static CallbackSlot<void(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel)> on_insert_imu;

  /**
   * @brief SubMap input callback
   * @param submap  SubMap
   * @note  submap->T_world_origin is updated in the global mapping thread
   *        Accessing it from another thread is not thread-safe
   */
  static CallbackSlot<void(const SubMap::ConstPtr& submap)> on_insert_submap;

  /**
   * @brief SubMap states update callback
   * @param submaps  Updated submaps
   * @note  submap->T_world_origin are updated in the global mapping thread
   *        Accessing them from another thread is not thread-safe
   */
  static CallbackSlot<void(const std::vector<SubMap::Ptr>& submaps)> on_update_submaps;

  /**
   * @brief Session-continuation relocalization callback
   * @param T_map_odom  Initial map offset: the rigid transform placing this session's odometry/world
   *                    origin into the loaded prior-map frame (prior_map <- new_session_odom). Fired
   *                    once, on the global-mapping thread, when the new session locks onto the prior map.
   */
  static CallbackSlot<void(const Eigen::Isometry3d& T_map_odom)> on_relocalized;

  /**
   * @brief Session-continuation relocalization preview callback
   * @param src_points_odom  The accumulated new-session source cloud used for the match, in the
   *                         session/odom frame. Transform by T_map_odom to place it in the prior-map
   *                         (viewer world) frame.
   * @param T_map_odom       The accepted prior_map <- new_session_odom alignment (same as on_relocalized).
   *
   * Fired alongside on_relocalized (both sim report-only and real-robot apply modes) so a viewer can
   * overlay the matched start cloud on the loaded prior map for visual confirmation of the alignment.
   */
  static CallbackSlot<void(const std::vector<Eigen::Vector4d>& src_points_odom, const Eigen::Isometry3d& T_map_odom)> on_relocalization_preview;

  /**
   * @brief Session-continuation relocalization progress callback
   * @param done     Work units finished so far within the current phase (e.g. yaw hypotheses fitted).
   * @param total    Total work units in the current phase (0 = indeterminate/spinner).
   * @param attempt  1-based relocalization attempt (grows as more submaps are buffered; see reloc_max_submaps).
   * @param phase    Short human-readable phase label ("yaw-sweep", "refine", "fpfh", "done", ...).
   *
   * Fired repeatedly on the global-mapping thread while relocalization runs, so a viewer / the website
   * can show a live progress bar during the (multi-second) initial alignment. Reporting only — carries
   * no pose. Fires in both apply and report-only modes, mirroring on_relocalized.
   */
  static CallbackSlot<void(int done, int total, int attempt, const std::string& phase)> on_relocalization_progress;

  /**
   * @brief Session-continuation per-attempt relocalization CANDIDATE callback
   * @param T_map_odom  The attempt's best-fit prior_map <- new_session_odom hypothesis (the same
   *                    frame as on_relocalized), whether or not it passed the inlier gate.
   * @param attempt     1-based relocalization attempt.
   * @param accepted    true if this hypothesis passed the gate (== the one on_relocalized will report),
   *                    false if it was rejected and another attempt will follow.
   *
   * Fired once per attempt on the global-mapping thread, INCLUDING rejected attempts, so a viewer /
   * the website can watch the relocalizer step through its alignment choices in real time rather than
   * only seeing the final accepted pose. Reporting only; never re-anchors anything.
   */
  static CallbackSlot<void(const Eigen::Isometry3d& T_map_odom, int attempt, bool accepted)> on_relocalization_candidate;

  /**
   * @brief Global optimization callback (just before optimization)
   * @param isam2        iSAM2 Optimizer
   * @param new_factors  New factors to be inserted into the factor graph
   * @param new_values   New values to be inserted into the factor graph
   */
  static CallbackSlot<void(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values)> on_smoother_update;

  /**
   * @brief Global optimization result callback (just after optimization)
   * @param isam2   iSAM2 optimizer
   * @param result  iSAM2 result
   */
  static CallbackSlot<void(gtsam_points::ISAM2Ext& isam2, const gtsam_points::ISAM2ResultExt& result)> on_smoother_update_result;

  /**
   * @brief Request the global mapping module to perform optimization
   * @note  This is a special inverse-direction callback slot
   */
  static CallbackSlot<void()> request_to_optimize;

  /**
   * @brief Request the global mapping module to detect and recover from a graph corruption
   * @note  This is a special inverse-direction callback slot
   */
  static CallbackSlot<void()> request_to_recover;

  /**
   * @brief Request the global mapping module to find new overlapping submaps
   * @param min_overlap  Minimum overlap rate between submaps
   * @note  This is a special inverse-direction callback slot
   */
  static CallbackSlot<void(double)> request_to_find_overlapping_submaps;
};
}  // namespace glim