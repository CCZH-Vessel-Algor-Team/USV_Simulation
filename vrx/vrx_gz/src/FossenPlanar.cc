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
/// Model-2 planar dynamics in FLU coordinates. Gazebo integrates the rigid
/// body; this system supplies a feedback-equivalent wrench, not pose updates.
/// Intended for a single gravity-free link, horizontal and without contact.
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
    if (!this->M.allFinite() || !this->D.allFinite() ||
      (this->M.array() <= 0).any() || (this->D.array() < 0).any() ||
      !std::isfinite(this->spacing) || this->spacing <= 0 ||
      !std::isfinite(this->limit) || this->limit <= 0 ||
      !std::isfinite(this->timeout) || this->timeout <= 0 ||
      this->mass <= 0 || this->izz <= 0)
      throw std::runtime_error("FossenPlanar: invalid dynamics parameters");
    const auto ns = _sdf->Get<std::string>("namespace", "usv_1").first;
    for (std::size_t i = 0; i < 2; ++i)
    {
      const auto topic = "/" + ns + "/thrusters/" +
        (i == 0 ? "left" : "right") + "/thrust";
      this->node.Subscribe<gz::msgs::Double>(topic,
        [this, i](const gz::msgs::Double &_msg)
        {
          std::lock_guard<std::mutex> guard(this->mutex);
          // Invalid commands fail closed rather than retaining an old thrust.
          this->commands[i] = std::isfinite(_msg.data()) ?
            std::clamp(_msg.data(), -this->limit, this->limit) : 0.0;
          ++this->sequence[i];
        });
    }
    this->telemetry = this->node.Advertise<gz::msgs::Double_V>(
      "/" + ns + "/fossen/state");
    gzmsg << "FossenPlanar: complete M/C/D; direct +/-" << this->limit
      << " N per side, spacing " << this->spacing << " m\n";
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
    }
    // FRD -> FLU: u unchanged, v/r negated. C retains the same algebraic
    // structure. In FLU a stronger right thruster gives positive yaw.
    const Eigen::Vector3d tau(thrust[0] + thrust[1], 0.0,
      (thrust[1] - thrust[0]) * this->spacing / 2.0);
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

    // Ordered diagnostic values: t,x,y,yaw,u,v,r,du,dv,dr,left,right,nu^T C nu.
    // These are measured physics states and evaluated derivatives, not an
    // independently integrated trajectory or a replacement odometry stream.
    gz::msgs::Double_V message;
    for (double value : {now, pose->Pos().X(), pose->Pos().Y(), pose->Rot().Yaw(),
      nu[0], nu[1], nu[2], dnu[0], dnu[1], dnu[2], thrust[0], thrust[1],
      nu.dot(C * nu)})
      message.add_data(value);
    this->telemetry.Publish(message);
  }

  public: void Reset(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &) override
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    this->commands = {};
    this->sequence = {};
    this->consumed = {};
    this->received = {{-1e9, -1e9}};
  }

  private: gz::sim::Link link;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher telemetry;
  private: Eigen::Vector3d M;
  private: Eigen::Vector3d D;
  private: double mass{0}, izz{0}, spacing{0.52}, limit{125}, timeout{0.5};
  private: std::mutex mutex;
  private: std::array<double, 2> commands{};
  private: std::array<std::uint64_t, 2> sequence{}, consumed{};
  private: std::array<double, 2> received{{-1e9, -1e9}};
};
}  // namespace vrx

GZ_ADD_PLUGIN(vrx::FossenPlanar, gz::sim::System,
  vrx::FossenPlanar::ISystemConfigure, vrx::FossenPlanar::ISystemPreUpdate,
  vrx::FossenPlanar::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(vrx::FossenPlanar, "vrx::FossenPlanar")
