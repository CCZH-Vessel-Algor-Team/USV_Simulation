/* Copyright 2026. Licensed under the Apache License, Version 2.0. */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace vrx
{
/// Planar Fossen dynamics in FLU coordinates on a single carrier link.
/// Gazebo integrates the rigid body; this system supplies a
/// feedback-equivalent wrench, not pose updates. Intended for one carrier
/// link whose planar channels (surge/sway/yaw) are governed by the
/// identified model below; heave/roll/pitch stay with the physics engine
/// (buoyancy Surface, gravity).
///
/// Model-2 lineage: the M5 variant adds (2026-11):
///   * input_mode=power: per-side electrical power commands (kW) mapped to
///     thrust through the digitized X20 "Thrust propeller" static curve
///     (0-20 kW, 43 knots, linear interpolation; RMS vs chart centerline
///     1.65 N, on par with the PCHIP refit at this knot density).
///   * Reverse thrust: T(-P) = -reverse_scale * T(|P|); the manufacturer
///     chart has no reverse data, so reverse_scale is an explicit
///     assumption (default 0.5, ePropulsion-typical).
///   * Wind: the vrx::USVWind still-air model embedded as part of the
///     generalized force tau so every planar force shares the same
///     M^{-1} bookkeeping. Static mean flow only (gain/random walk of the
///     original is not implemented; deployment config uses gain 0).
///     wind_direction keeps the USVWind unit convention (degrees).
///
/// Telemetry derivatives: fields 7..9 are the PLUGIN-INTERNAL derivatives
/// (they exclude forces applied by other systems, e.g. the Hydrodynamics
/// quadratic surge damping); fields 18..20 add finite-difference actual
/// derivatives of the measured nu for diagnostics. Neither is fed back into
/// the applied wrench.
/// Quadratic surge damping is NOT implemented here by design: it is
/// supplied by the stock gz Hydrodynamics system on the same link with
/// xUabsU scaled by carrier_mass/M11 so that, after the carrier divides
/// the external force, the total surge dynamics equal the identified
/// model (see model.sdf comments for the exact numbers).
class FossenPlanar : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
  public: void Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    gz::sim::Model model(_entity);
    this->link = gz::sim::Link(model.LinkByName(_ecm,
      _sdf->Get<std::string>("link_name", "base_link").first));
    if (!this->link.Valid(_ecm))
      throw std::runtime_error("FossenPlanar: missing base link");
    this->link.EnableVelocityChecks(_ecm);
    const auto inertial = this->link.WorldInertial(_ecm);
    if (!inertial)
      throw std::runtime_error("FossenPlanar: missing link inertia");
    this->mass = inertial->MassMatrix().Mass();
    this->izz = inertial->MassMatrix().Izz();
    this->M << _sdf->Get<double>("m11", 50.05).first,
      _sdf->Get<double>("m22", 84.36).first,
      _sdf->Get<double>("m33", 17.21).first;
    this->D << _sdf->Get<double>("X_u", 151.57).first,
      _sdf->Get<double>("Y_v", 132.5).first,
      _sdf->Get<double>("N_r", 34.56).first;
    this->spacing = _sdf->Get<double>("thruster_spacing", 0.52).first;
    this->limit = _sdf->Get<double>("max_thrust", 125.0).first;
    this->timeout = _sdf->Get<double>("command_timeout", 0.5).first;

    const auto mode = _sdf->Get<std::string>("input_mode", "thrust").first;
    if (mode == "power")
      this->powerMode = true;
    else if (mode == "thrust")
      this->powerMode = false;
    else
      throw std::runtime_error("FossenPlanar: input_mode must be power|thrust");
    this->maxPower = _sdf->Get<double>("max_power", 20.0).first;
    this->reverseScale = _sdf->Get<double>("reverse_scale", 0.5).first;

    this->windEnable = _sdf->Get<bool>("wind_enable", true).first;
    // USVWind-compatible unit: degrees (0 = +X/East, CCW positive).
    this->windDirection =
      _sdf->Get<double>("wind_direction", 0.0).first * M_PI / 180.0;
    this->windVelocity = _sdf->Get<double>("wind_mean_velocity", 0.0).first;
    const auto coeffs = _sdf->Get<gz::math::Vector3d>(
      "wind_coeffs", gz::math::Vector3d(0.5, 0.5, 0.33)).first;
    this->windCoeffs = coeffs;

    if (!this->M.allFinite() || !this->D.allFinite() ||
      (this->M.array() <= 0).any() || (this->D.array() < 0).any() ||
      !std::isfinite(this->spacing) || this->spacing <= 0 ||
      !std::isfinite(this->limit) || this->limit <= 0 ||
      !std::isfinite(this->timeout) || this->timeout <= 0 ||
      !std::isfinite(this->maxPower) || this->maxPower <= 0 ||
      !std::isfinite(this->reverseScale) || this->reverseScale < 0 ||
      !std::isfinite(this->windDirection) ||
      !std::isfinite(this->windVelocity) || this->windVelocity < 0 ||
      this->mass <= 0 || this->izz <= 0)
      throw std::runtime_error("FossenPlanar: invalid dynamics parameters");
    this->windUnit = gz::math::Vector3d(
      std::cos(this->windDirection), std::sin(this->windDirection), 0.0);

    const auto ns = _sdf->Get<std::string>("namespace", "usv_1").first;
    for (std::size_t i = 0; i < 2; ++i)
    {
      const std::string channel =
        this->powerMode ? "power" : "thrust";
      const auto topic = "/" + ns + "/thrusters/" +
        (i == 0 ? "left" : "right") + "/" + channel;
      this->node.Subscribe<gz::msgs::Double>(topic,
        [this, i](const gz::msgs::Double &_msg)
        {
          std::lock_guard<std::mutex> guard(this->mutex);
          // Invalid commands fail closed rather than retaining an old value.
          double value = std::isfinite(_msg.data()) ? _msg.data() : 0.0;
          if (this->powerMode)
          {
            this->commands[i] =
              this->ThrustFromPower(value, i == 1);
          }
          else
          {
            this->commands[i] = std::clamp(value, -this->limit, this->limit);
          }
          ++this->sequence[i];
        });
    }
    this->telemetry = this->node.Advertise<gz::msgs::Double_V>(
      "/" + ns + "/fossen/state");
    gzmsg << "FossenPlanar: complete M/C/D; input=" << mode
      << ", +/-" << this->limit << " N per side, spacing "
      << this->spacing << " m, wind "
      << (this->windEnable ? "on" : "off") << "\n";
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || _info.dt.count() <= 0)
      return;
    const auto pose = this->link.WorldPose(_ecm);
    const auto linear = this->link.WorldLinearVelocity(_ecm);
    const auto angular = this->link.WorldAngularVelocity(_ecm);
    if (!pose || !linear || !angular)
      return;
    const double now = std::chrono::duration<double>(_info.simTime).count();
    const auto vBody = pose->Rot().Inverse().RotateVector(*linear);
    const auto wBody = pose->Rot().Inverse().RotateVector(*angular);
    const Eigen::Vector3d nu(vBody.X(), vBody.Y(), wBody.Z());
    // Finite-difference actual derivative of the measured nu (diagnostics
    // only; includes every applied force and is never fed back).
    Eigen::Vector3d dnuActual(0, 0, 0);
    const double step = std::chrono::duration<double>(_info.dt).count();
    if (this->hasPrev && step > 0.0)
      dnuActual = (nu - this->prevNu) / step;
    this->prevNu = nu;
    this->hasPrev = true;
    std::array<double, 2> thrust{};
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      for (std::size_t i = 0; i < 2; ++i)
      {
        if (this->sequence[i] != this->consumed[i])
        {
          this->received[i] = now;
          this->consumed[i] = this->sequence[i];
        }
        if (now - this->received[i] <= this->timeout)
          thrust[i] = this->commands[i];
      }
      this->telemetryPower = this->lastPower;
    }
    // FRD -> FLU: u unchanged, v/r negated. C retains the same algebraic
    // structure. In FLU a stronger right thruster gives positive yaw.
    Eigen::Vector3d tau(thrust[0] + thrust[1], 0.0,
      (thrust[1] - thrust[0]) * this->spacing / 2.0);
    // Still-air/wind drag, USVWind algebra, applied here as part of tau so
    // it shares the identified M^{-1} instead of the carrier mass.
    gz::math::Vector3d windForceBody(0, 0, 0);
    double windTorque = 0.0;
    if (this->windEnable)
    {
      const gz::math::Vector3d relativeWind =
        this->windUnit * this->windVelocity - *linear;
      const gz::math::Vector3d windForce(
        this->windCoeffs.X() * relativeWind.X() * std::abs(relativeWind.X()),
        this->windCoeffs.Y() * relativeWind.Y() * std::abs(relativeWind.Y()),
        0.0);
      windTorque = -2.0 * this->windCoeffs.Z() * relativeWind.X()
        * relativeWind.Y();
      windForceBody = pose->Rot().Inverse().RotateVector(windForce);
      tau[0] += windForceBody.X();
      tau[1] += windForceBody.Y();
      tau[2] += windTorque;
    }
    Eigen::Matrix3d C;
    C << 0, 0, -this->M[1] * nu[1],
      0, 0, this->M[0] * nu[0],
      this->M[1] * nu[1], -this->M[0] * nu[0], 0;
    const Eigen::Vector3d dnu = (tau - C * nu -
      this->D.cwiseProduct(nu)).cwiseQuotient(this->M);

    // World acceleration expressed in the rotating body frame is
    // [du-r*v, dv+r*u]. Gazebo already supplies rigid-body C_RB; applying
    // -C_total*nu again as an external wrench would double-count it.
    // Use actual carrier inertia to realize the desired complete derivative.
    const gz::math::Vector3d force(this->mass * (dnu[0] - nu[2] * nu[1]),
      this->mass * (dnu[1] + nu[2] * nu[0]), 0);
    const gz::math::Vector3d torque(0, 0, this->izz * dnu[2]);
    this->link.AddWorldWrench(_ecm, pose->Rot().RotateVector(force),
      pose->Rot().RotateVector(torque));

    // Ordered diagnostic values:
    //   0..12 (model-2 layout): t,x,y,yaw,u,v,r, du,dv,dr (PLUGIN-INTERNAL
    //          derivatives: they exclude forces from other systems such as
    //          the Hydrodynamics quadratic surge damping), left,right,
    //          nu^T C nu
    //   13..17 (M5): leftPower_kW, rightPower_kW, windFx_body, windFy_body,
    //          windN. Power fields echo the last commanded kW when in power
    //          mode and 0 otherwise.
    //   18..20 (M5 review fix): du,dv,dr ACTUAL finite-difference derivatives
    //          of the measured nu (all forces included). These are measured
    //          physics states and evaluated forces, not an independently
    //          integrated trajectory or a replacement odometry stream.
    gz::msgs::Double_V message;
    for (double value : {now, pose->Pos().X(), pose->Pos().Y(), pose->Rot().Yaw(),
      nu[0], nu[1], nu[2], dnu[0], dnu[1], dnu[2], thrust[0], thrust[1],
      nu.dot(C * nu)})
      message.add_data(value);
    message.add_data(this->telemetryPower[0]);
    message.add_data(this->telemetryPower[1]);
    message.add_data(windForceBody.X());
    message.add_data(windForceBody.Y());
    message.add_data(windTorque);
    message.add_data(dnuActual[0]);
    message.add_data(dnuActual[1]);
    message.add_data(dnuActual[2]);
    this->telemetry.Publish(message);
  }

  public: void Reset(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &) override
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    this->commands = {};
    this->lastPower = {};
    this->sequence = {};
    this->consumed = {};
    this->received = {{-1e9, -1e9}};
    this->hasPrev = false;
  }

  private: double ThrustFromPower(double powerKw, std::size_t side)
  {
    // X20 "Thrust propeller" static curve, digitized from the manufacturer
    // chart (M5Experiment deliverable, 2026-11). kW -> N, forward only;
    // reverse is the explicit reverse_scale assumption.
    static constexpr std::array<std::array<double, 2>, 43> kCurve{{
      {0.0, 0.0}, {0.1, 87.38}, {0.2, 127.95}, {0.5, 230.96},
      {1.0, 371.87}, {1.5, 489.56}, {2.0, 594.52}, {2.5, 689.94},
      {3.0, 780.17}, {3.5, 866.44}, {4.0, 945.62}, {4.5, 1022.46},
      {5.0, 1099.37}, {5.5, 1171.86}, {6.0, 1239.52}, {6.5, 1310.98},
      {7.0, 1377.62}, {7.5, 1441.88}, {8.0, 1505.04}, {8.5, 1565.82},
      {9.0, 1626.75}, {9.5, 1687.39}, {10.0, 1748.17}, {10.5, 1805.63},
      {11.0, 1862.35}, {11.5, 1918.82}, {12.0, 1973.75}, {12.5, 2028.68},
      {13.0, 2083.40}, {13.5, 2135.84}, {14.0, 2187.63}, {14.5, 2241.17},
      {15.0, 2291.64}, {15.5, 2340.72}, {16.0, 2389.80}, {16.5, 2438.88},
      {17.0, 2487.96}, {17.5, 2537.04}, {18.0, 2586.12}, {18.5, 2634.22},
      {19.0, 2684.28}, {19.5, 2730.11}, {20.0, 2773.67},
    }};
    const double magnitude = Interpolate(
      kCurve, std::min(std::abs(powerKw), this->maxPower));
    const double signedThrust = powerKw >= 0.0
      ? magnitude : -this->reverseScale * magnitude;
    this->lastPower[side] = powerKw;
    return std::clamp(signedThrust, -this->limit, this->limit);
  }

  private: template <std::size_t N>
  static double Interpolate(const std::array<std::array<double, 2>, N> &curve,
    double x)
  {
    if (x <= curve.front()[0])
      return curve.front()[1];
    for (std::size_t i = 1; i < N; ++i)
    {
      if (x <= curve[i][0])
      {
        const auto &a = curve[i - 1];
        const auto &b = curve[i];
        const double span = b[0] - a[0];
        if (span <= 0.0)
          return b[1];
        return a[1] + (b[1] - a[1]) * (x - a[0]) / span;
      }
    }
    return curve.back()[1];
  }

  private: gz::sim::Link link;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher telemetry;
  private: Eigen::Vector3d M;
  private: Eigen::Vector3d D;
  private: double mass{0}, izz{0}, spacing{0.52}, limit{125}, timeout{0.5};
  private: bool powerMode{false};
  private: double maxPower{20.0}, reverseScale{0.5};
  private: bool windEnable{true};
  private: double windDirection{0.0}, windVelocity{0.0};
  private: gz::math::Vector3d windCoeffs{0.5, 0.5, 0.33};
  private: gz::math::Vector3d windUnit{1, 0, 0};
  private: std::mutex mutex;
  private: std::array<double, 2> commands{};
  private: std::array<double, 2> lastPower{};
  private: std::array<double, 2> telemetryPower{};
  private: Eigen::Vector3d prevNu{Eigen::Vector3d::Zero()};
  private: bool hasPrev{false};
  private: std::array<std::uint64_t, 2> sequence{}, consumed{};
  private: std::array<double, 2> received{{-1e9, -1e9}};
};
}  // namespace vrx

GZ_ADD_PLUGIN(vrx::FossenPlanar, gz::sim::System,
  vrx::FossenPlanar::ISystemConfigure, vrx::FossenPlanar::ISystemPreUpdate,
  vrx::FossenPlanar::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(vrx::FossenPlanar, "vrx::FossenPlanar")
