#ifndef USV_VESSEL_DEVICES_PROPULSION_PHYSICS_HH_
#define USV_VESSEL_DEVICES_PROPULSION_PHYSICS_HH_

#include <algorithm>
#include <cmath>

namespace usv_vessel_devices
{
// Area of a circular actuator disk below a locally horizontal free surface.
// verticalRadius is the projection of its radius onto the world vertical.
inline double SubmergedDiskFraction(double depth, double verticalRadius)
{
  if (!std::isfinite(depth) || !std::isfinite(verticalRadius) || verticalRadius < 0.0)
    return 0.0;
  if (verticalRadius < 1e-9)
    return depth > 0.0 ? 1.0 : depth < 0.0 ? 0.0 : 0.5;
  const double x = std::clamp(depth / verticalRadius, -1.0, 1.0);
  return 0.5 + (std::asin(x) + x * std::sqrt(std::max(0.0, 1.0 - x * x))) / M_PI;
}

inline double PropellerSpeed(double thrust, double density, double coefficient, double diameter)
{
  const double scale = density * coefficient * std::pow(diameter, 4);
  if (!std::isfinite(thrust) || !std::isfinite(scale) || scale <= 0.0) return 0.0;
  return std::copysign(std::sqrt(std::abs(thrust) / scale), thrust);
}
}  // namespace usv_vessel_devices
#endif
