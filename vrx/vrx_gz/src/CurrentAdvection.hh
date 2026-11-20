/*
 * Copyright (C) 2026
 * Licensed under the Apache License, Version 2.0.
 * You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
 * Distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.
 */
#ifndef VRX_CURRENTADVECTION_HH_
#define VRX_CURRENTADVECTION_HH_

#include <array>
#include <cmath>

namespace vrx
{
/// \brief Integrate texture displacement, independently of wave propagation.
/// Velocities are UV repetitions per simulation second. Callers synchronize
/// access, advance by simulation time only and reset phase on rewind.
class CurrentAdvection
{
  /// \brief Ignore invalid input without corrupting the last valid command.
  public: bool SetVelocity(double _u, double _v)
  {
    if (!std::isfinite(_u) || !std::isfinite(_v))
      return false;
    this->velocity = {_u, _v};
    return true;
  }

  /// \brief Advance continuous phase; changing velocity never resets it.
  public: void Advance(double _dt)
  {
    if (!std::isfinite(_dt) || _dt <= 0.0)
      return;
    const double du = this->velocity[0] * _dt;
    const double dv = this->velocity[1] * _dt;
    if (!std::isfinite(du) || !std::isfinite(dv))
      return;
    this->offset[0] = Wrap(this->offset[0] + du);
    this->offset[1] = Wrap(this->offset[1] + dv);
  }

  /// \brief Reset phase on rewind; retain the latest physical current command.
  public: void ResetPhase()
  {
    this->offset = {0.0, 0.0};
  }

  public: const std::array<double, 2> &Offset() const
  {
    return this->offset;
  }

  /// \brief Keep uniforms bounded to avoid precision loss in long simulations.
  private: static double Wrap(double _value)
  {
    const double wrapped = std::fmod(_value, 1.0);
    return wrapped < 0.0 ? wrapped + 1.0 : wrapped;
  }

  private: std::array<double, 2> velocity{0.0, 0.0};
  private: std::array<double, 2> offset{0.0, 0.0};
};
}
#endif
