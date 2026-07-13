#include <glim/odometry/odometry_estimation_external.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  return new glim::OdometryEstimationExternal();
}
