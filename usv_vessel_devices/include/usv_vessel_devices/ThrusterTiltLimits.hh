#ifndef USV_VESSEL_DEVICES_THRUSTER_TILT_LIMITS_HH_
#define USV_VESSEL_DEVICES_THRUSTER_TILT_LIMITS_HH_

#include <cmath>

namespace usv_vessel_devices
{
inline constexpr double kMaxThrusterTiltAngle = 1.3089969389957472;  // 75 degrees

inline bool ValidTiltLimit(double upper)
{
  return std::isfinite(upper) && upper > 0.0 && upper <= kMaxThrusterTiltAngle;
}

inline bool ValidTiltTarget(double target, double upper)
{
  return ValidTiltLimit(upper) && std::isfinite(target) && target >= 0.0 && target <= upper;
}
}  // namespace usv_vessel_devices
#endif
