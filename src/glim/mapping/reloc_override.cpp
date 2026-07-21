#include <glim/mapping/reloc_override.hpp>

#include <mutex>

namespace glim {

namespace {
// Storage lives here (not as an inline variable in the header) so libglim owns exactly one slot even
// though the setter is called from libglim_ros and the taker from libglim.
std::mutex mutex;
bool has_override = false;
Eigen::Isometry3d pending_T = Eigen::Isometry3d::Identity();
}  // namespace

void RelocOverride::set(const Eigen::Isometry3d& T_map_odom) {
  std::lock_guard<std::mutex> lock(mutex);
  pending_T = T_map_odom;
  has_override = true;
}

bool RelocOverride::take(Eigen::Isometry3d& out) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!has_override) {
    return false;
  }
  out = pending_T;
  has_override = false;
  return true;
}

bool RelocOverride::pending() {
  std::lock_guard<std::mutex> lock(mutex);
  return has_override;
}

void RelocOverride::clear() {
  std::lock_guard<std::mutex> lock(mutex);
  has_override = false;
}

}  // namespace glim
