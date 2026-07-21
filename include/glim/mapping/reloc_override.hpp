#pragma once

#include <Eigen/Geometry>

namespace glim {

/**
 * @brief Operator relocalization override: a one-slot, thread-safe mailbox.
 *
 * Session-continuation relocalization is otherwise fully autonomous — it sweeps/RANSACs for a pose
 * and gates it on an inlier rate (see GlobalMapping::try_relocalize_pending). When the geometry is
 * ambiguous the gate can reject hypotheses that a human looking at the overlay can plainly see are
 * correct, or close enough to nudge. This mailbox is how that human answer gets in.
 *
 * The pose is the SAME quantity GlobalMappingCallbacks::on_relocalized reports and the ROS
 * ~/map_offset topic publishes: T_map_odom, i.e. prior_map <- new_session_odom. So a consumer can
 * take a candidate straight off ~/reloc_candidate / ~/map_offset, adjust it, and hand it back
 * unconverted.
 *
 * An override is ACCEPTED UNCONDITIONALLY — it bypasses the inlier gate entirely; that is its whole
 * purpose. It is only refined (bounded VGICP, seeded from the operator pose, kept only if it
 * improves the inlier rate), never rejected.
 *
 * Threading: set() is called from the ROS subscription thread, take() from the global-mapping thread
 * (which is the only thread allowed near the factor graph). take() consumes, so an override applies
 * once; a stale latched message replayed on reconnect cannot re-anchor a session that already moved on.
 *
 * Latency note: take() is polled at the top of each relocalization attempt, and attempts are driven
 * by submap arrivals. An override therefore lands on the NEXT submap (typically a few seconds), not
 * instantly.
 */
struct RelocOverride {
  /// @brief Post an operator-supplied alignment. Overwrites any un-consumed previous override.
  /// @param T_map_odom  prior_map <- new_session_odom (same frame as on_relocalized / ~/map_offset).
  static void set(const Eigen::Isometry3d& T_map_odom);

  /// @brief Consume a pending override, if any.
  /// @return true if one was waiting (written to `out`); false leaves `out` untouched.
  static bool take(Eigen::Isometry3d& out);

  /// @brief Whether an override is currently waiting to be consumed (does not consume).
  static bool pending();

  /// @brief Drop any pending override (e.g. when a session resets to fresh mapping).
  static void clear();
};

}  // namespace glim
