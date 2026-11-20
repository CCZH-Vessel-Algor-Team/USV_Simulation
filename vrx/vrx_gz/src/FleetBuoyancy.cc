// Copyright 2026 USV simulation contributors
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <string>
#include <sdf/Box.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/World.hh>
#include "FleetHydrostatics.hh"

namespace vrx
{
// Deliberately model-local: no Volume components or forces are created on the
// ownship, legacy targets, buoys, terrain or turbine models.
class FleetBuoyancy final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    if (!model.Valid(ecm)) {
      gzerr << "FleetBuoyancy must be attached to one vessel model\n";
      return;
    }
    link_ = gz::sim::Link(model.LinkByName(ecm, sdf->Get<std::string>("link_name", "base_link").first));
    world_ = ecm.EntityByComponents(gz::sim::components::World());
    water_ = sdf->Get<double>("water_density", 1025.0).first;
    air_ = sdf->Get<double>("air_density", 1.225).first;
    level_ = sdf->Get<double>("water_level", 0.0).first;
    valid_ = link_.Valid(ecm) && std::isfinite(level_) &&
      std::isfinite(water_) && water_ > 0 && std::isfinite(air_) && air_ >= 0;
    if (!valid_) {gzerr << "Invalid FleetBuoyancy link or density\n";}
  }

  void PreUpdate(const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!valid_ || info.paused) {return;}
    const auto gravity = ecm.Component<gz::sim::components::Gravity>(world_);
    const auto inertial = ecm.Component<gz::sim::components::Inertial>(link_.Entity());
    if (!gravity || !inertial) {return;}
    const auto pose = gz::sim::worldPose(link_.Entity(), ecm);
    const auto com = pose.Pos() + pose.Rot().RotateVector(inertial->Data().Pose().Pos());
    gz::math::Vector3d force, torque;
    for (auto entity : ecm.ChildrenByComponents(link_.Entity(), gz::sim::components::Collision())) {
      const auto collision = ecm.Component<gz::sim::components::CollisionElement>(entity);
      if (!collision || collision->Data().Geom()->Type() != sdf::GeometryType::BOX) {continue;}
      const auto wrench = GradedBoxWrench(collision->Data().Geom()->BoxShape()->Shape(),
        gz::sim::worldPose(entity, ecm), com, gravity->Data(), level_, water_, air_);
      force += wrench.first;
      torque += wrench.second;
    }
    if (force.IsFinite() && torque.IsFinite()) {link_.AddWorldWrench(ecm, force, torque);}
  }

private:
  gz::sim::Link link_{gz::sim::kNullEntity};
  gz::sim::Entity world_{gz::sim::kNullEntity};
  double water_{1025.0}, air_{1.225}, level_{0.0};
  bool valid_{false};
};
}  // namespace vrx
GZ_ADD_PLUGIN(vrx::FleetBuoyancy, gz::sim::System,
  vrx::FleetBuoyancy::ISystemConfigure, vrx::FleetBuoyancy::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(vrx::FleetBuoyancy, "vrx::FleetBuoyancy")
