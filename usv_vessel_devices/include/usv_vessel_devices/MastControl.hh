#ifndef USV_VESSEL_DEVICES_MAST_CONTROL_HH_
#define USV_VESSEL_DEVICES_MAST_CONTROL_HH_

#include <algorithm>
#include <cmath>

namespace usv_vessel_devices
{
inline bool ValidTarget(double target, double upper)
{
  return std::isfinite(target) && target >= 0.0 && target <= upper;
}

// Bounded velocity servo, with a one-step bound to avoid overshooting at low RTF.
inline double MastVelocity(double position, double target, double speed,
  double tolerance, double dt)
{
  if (!std::isfinite(position) || dt <= 0.0 || !std::isfinite(dt))
    return 0.0;
  const double error = target - position;
  // Keep correcting well inside the reported arrival tolerance. A full-width
  // dead band lets the heavier, enlarged assembly drift out again while the
  // velocity integral is still adapting to its gravity load.
  if (std::abs(error) <= tolerance * 0.1)
    return 0.0;
  const double bound = std::min(speed, std::abs(error) / dt);
  return std::clamp(4.0 * error, -bound, bound);
}

// Effort servo tracks bounded velocity and holds against mast gravity at rest.
// Integral is retained at a stationary target (unlike a zero-torque dead band).
inline double MastEffort(double desiredVelocity, double actualVelocity,
  double dt, double &integral)
{
  if (dt <= 0.0 || !std::isfinite(dt) || !std::isfinite(actualVelocity))
    return 0.0;
  const double error = desiredVelocity - actualVelocity;
  integral = std::clamp(integral + 80.0 * error * dt, -100.0, 100.0);
  return std::clamp(40.0 * error + integral, -100.0, 100.0);
}
}  // namespace usv_vessel_devices
#endif
