#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <sdf/Element.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

namespace usv_4d_radar_gz
{

class ReciprocatingTargetsPlugin final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  struct TargetConfig
  {
    std::string name;
    gz::sim::Entity entity{gz::sim::kNullEntity};
    gz::math::Vector3d center{0, 0, 0};
    gz::math::Vector3d axis{1, 0, 0};
    double amplitude{1.0};
    double frequencyHz{0.1};
    double phaseRad{0.0};
    gz::math::Quaterniond orientation{gz::math::Quaterniond::Identity};
  };

  void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    if (!_sdf)
    {
      return;
    }

    if (_sdf->HasElement("target"))
    {
      sdf::ElementConstPtr targetElem = _sdf->FindElement("target");
      while (targetElem)
      {
        TargetConfig cfg;

        if (targetElem->HasElement("name"))
        {
          cfg.name = targetElem->Get<std::string>("name");
        }
        if (targetElem->HasElement("center"))
        {
          cfg.center = targetElem->Get<gz::math::Vector3d>("center");
        }
        if (targetElem->HasElement("axis"))
        {
          cfg.axis = targetElem->Get<gz::math::Vector3d>("axis");
        }
        if (targetElem->HasElement("amplitude"))
        {
          cfg.amplitude = targetElem->Get<double>("amplitude");
        }
        if (targetElem->HasElement("frequency_hz"))
        {
          cfg.frequencyHz = targetElem->Get<double>("frequency_hz");
        }
        if (targetElem->HasElement("phase"))
        {
          cfg.phaseRad = targetElem->Get<double>("phase");
        }
        if (targetElem->HasElement("orientation_rpy"))
        {
          const gz::math::Vector3d rpy =
            targetElem->Get<gz::math::Vector3d>("orientation_rpy");
          cfg.orientation = gz::math::Quaterniond(rpy.X(), rpy.Y(), rpy.Z());
        }

        if (cfg.axis.Length() < 1e-8)
        {
          cfg.axis = gz::math::Vector3d(1, 0, 0);
        }
        cfg.axis.Normalize();

        if (!cfg.name.empty())
        {
          this->targets_.push_back(cfg);
        }

        targetElem = targetElem->GetNextElement("target");
      }
    }
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
    {
      return;
    }

    const double t =
      std::chrono::duration_cast<std::chrono::duration<double>>(_info.simTime).count();

    for (auto &target : this->targets_)
    {
      if (target.entity == gz::sim::kNullEntity)
      {
        target.entity = this->FindEntityByName(_ecm, target.name);
        if (target.entity == gz::sim::kNullEntity)
        {
          continue;
        }
      }

      const double omega = 2.0 * M_PI * target.frequencyHz;
      const double d = target.amplitude * std::sin(omega * t + target.phaseRad);
      const gz::math::Vector3d p = target.center + target.axis * d;
      const gz::math::Pose3d pose(p, target.orientation);

      _ecm.SetComponentData<gz::sim::components::Pose>(target.entity, pose);

      const double vd = target.amplitude * omega * std::cos(omega * t + target.phaseRad);
      const gz::math::Vector3d vWorld = target.axis * vd;
      const gz::math::Vector3d vLocal = target.orientation.Inverse().RotateVector(vWorld);
      _ecm.SetComponentData<gz::sim::components::LinearVelocity>(
        target.entity, vLocal);
    }
  }

private:
  gz::sim::Entity FindEntityByName(
    const gz::sim::EntityComponentManager &_ecm,
    const std::string &_name) const
  {
    gz::sim::Entity result = gz::sim::kNullEntity;
    _ecm.Each<gz::sim::components::Name>(
      [&](const gz::sim::Entity &_entity, const gz::sim::components::Name *_nameComp)
      {
        if (_nameComp->Data() == _name)
        {
          result = _entity;
          return false;
        }
        return true;
      });
    return result;
  }

  std::vector<TargetConfig> targets_;
};

}  // namespace usv_4d_radar_gz

GZ_ADD_PLUGIN(
  usv_4d_radar_gz::ReciprocatingTargetsPlugin,
  ::gz::sim::System,
  ::gz::sim::ISystemConfigure,
  ::gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  usv_4d_radar_gz::ReciprocatingTargetsPlugin,
  "usv_4d_radar_gz::ReciprocatingTargetsPlugin")
