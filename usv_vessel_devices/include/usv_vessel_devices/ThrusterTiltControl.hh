#ifndef USV_VESSEL_DEVICES_THRUSTER_TILT_CONTROL_HH_
#define USV_VESSEL_DEVICES_THRUSTER_TILT_CONTROL_HH_

#include <cmath>

namespace usv_vessel_devices
{
// A nonzero target inhibits thrust immediately, before the mechanism moves.
inline bool TiltThrustEnabled(bool ready, double position, double velocity,
  double target, double tolerance, bool wasEnabled = false)
{
  return ready && std::isfinite(position) && std::isfinite(velocity) &&
    target == 0.0 && std::abs(position) <= tolerance &&
    (wasEnabled || std::abs(velocity) <= 0.001);
}

}  // namespace usv_vessel_devices
#endif
