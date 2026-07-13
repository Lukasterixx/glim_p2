#include <glim/odometry/odometry_estimation_localizer.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  glim::OdometryEstimationLocalizerParams params;
  return new glim::OdometryEstimationLocalizer(params);
}
