#include "usv_4d_radar_gz/4d_radar_plugin.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>

#include <sdf/Element.hh>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/AxisAlignedBox.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>

namespace usv_4d_radar_gz
{
namespace
{

double DegToRad(double _deg)
{
  return _deg * 3.14159265358979323846 / 180.0;
}

bool IntersectRayAabbLocal(
  const gz::math::Vector3d &_rayOriginLocal,
  const gz::math::Vector3d &_rayDirLocal,
  const gz::math::AxisAlignedBox &_aabb,
  double &_outT)
{
  const double kEps = 1e-8;
  double tMin = 0.0;
  double tMax = std::numeric_limits<double>::max();

  const gz::math::Vector3d minPt = _aabb.Min();
  const gz::math::Vector3d maxPt = _aabb.Max();

  for (int i = 0; i < 3; ++i)
  {
    const double origin = _rayOriginLocal[i];
    const double dir = _rayDirLocal[i];

    if (std::abs(dir) < kEps)
    {
      if (origin < minPt[i] || origin > maxPt[i])
      {
        return false;
      }
      continue;
    }

    double t1 = (minPt[i] - origin) / dir;
    double t2 = (maxPt[i] - origin) / dir;
    if (t1 > t2)
    {
      std::swap(t1, t2);
    }

    tMin = std::max(tMin, t1);
    tMax = std::min(tMax, t2);
    if (tMin > tMax)
    {
      return false;
    }
  }

  _outT = tMin;
  return true;
}

bool IntersectRaySphereWorld(
  const gz::math::Vector3d &_rayOrigin,
  const gz::math::Vector3d &_rayDir,
  const gz::math::Vector3d &_sphereCenter,
  double _radius,
  double &_outT)
{
  const gz::math::Vector3d oc = _rayOrigin - _sphereCenter;
  const double b = 2.0 * oc.Dot(_rayDir);
  const double c = oc.Dot(oc) - _radius * _radius;
  const double discriminant = b * b - 4.0 * c;
  if (discriminant < 0.0)
  {
    return false;
  }

  const double sqrtDisc = std::sqrt(discriminant);
  const double t1 = (-b - sqrtDisc) * 0.5;
  const double t2 = (-b + sqrtDisc) * 0.5;

  if (t1 >= 0.0)
  {
    _outT = t1;
    return true;
  }
  if (t2 >= 0.0)
  {
    _outT = t2;
    return true;
  }
  return false;
}

class EcmAabbRayCaster final : public IRayCaster
{
public:
  std::optional<RayHit> Cast(
    const RaySpec &_ray,
    const gz::sim::EntityComponentManager &_ecm,
    gz::sim::Entity _selfEntity,
    gz::sim::Entity _ignoreTopLevelModel) const override
  {
    std::optional<RayHit> bestHit;
    double minRange = _ray.maxRange;

    _ecm.Each<gz::sim::components::Collision>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::Collision *)
      {
        if (_entity == _selfEntity)
        {
          return true;
        }

        if (_ignoreTopLevelModel != gz::sim::kNullEntity)
        {
          const gz::sim::Entity hitTopModel =
            gz::sim::topLevelModel(_entity, _ecm);
          if (hitTopModel == _ignoreTopLevelModel)
          {
            return true;
          }
        }

        const gz::math::Pose3d worldPose = gz::sim::worldPose(_entity, _ecm);

        gz::math::Vector3d hitWorld;
        double t = 0.0;
        const auto *aabbComp = _ecm.Component<gz::sim::components::AxisAlignedBox>(_entity);
        if (aabbComp)
        {
          const gz::math::Vector3d originLocal =
            worldPose.Rot().Inverse().RotateVector(_ray.origin - worldPose.Pos());
          const gz::math::Vector3d dirLocal =
            worldPose.Rot().Inverse().RotateVector(_ray.direction);

          double tLocal = 0.0;
          if (!IntersectRayAabbLocal(originLocal, dirLocal, aabbComp->Data(), tLocal))
          {
            return true;
          }
          if (tLocal < 0.0)
          {
            return true;
          }

          const gz::math::Vector3d hitLocal = originLocal + dirLocal * tLocal;
          hitWorld = worldPose.Pos() + worldPose.Rot().RotateVector(hitLocal);
          t = tLocal;
        }
        else
        {
          constexpr double kFallbackRadius = 2.0;
          if (!IntersectRaySphereWorld(
              _ray.origin, _ray.direction, worldPose.Pos(), kFallbackRadius, t))
          {
            return true;
          }

          hitWorld = _ray.origin + _ray.direction * t;
        }

        const double range = hitWorld.Distance(_ray.origin);

        if (range < minRange && range <= _ray.maxRange)
        {
          minRange = range;
          bestHit = RayHit{_entity, hitWorld, range};
        }

        return true;
      });

    return bestHit;
  }
};

builtin_interfaces::msg::Time ToRosTime(const int64_t _simTimeNs)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(_simTimeNs / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(_simTimeNs % 1000000000LL);
  return t;
}

/// URDF 常见 link 名为「命名空间/短名」（如 usv_1/mmwave_front_link），而 gz-sim Name 组件多为短名。
std::string BasenameLink(const std::string &_full)
{
  const auto pos = _full.find_last_of('/');
  if (pos == std::string::npos || pos + 1 >= _full.size())
  {
    return _full;
  }
  return _full.substr(pos + 1);
}

std::string SanitizeTopicToNodeName(const std::string &_topic)
{
  std::string out = "gz_mmwave";
  for (char c : _topic)
  {
    if (c == '/')
    {
      continue;
    }
    if (std::isalnum(static_cast<unsigned char>(c)) || c == '_')
    {
      out.push_back(c);
    }
    else
    {
      out.push_back('_');
    }
  }
  if (out.size() > 240U)
  {
    out.resize(240U);
  }
  return out;
}

gz::sim::Entity FindEntityByName(
  const gz::sim::EntityComponentManager &_ecm,
  const std::string &_name)
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

gz::math::Vector3d ReadLinearVelocity(
  const gz::sim::EntityComponentManager &_ecm,
  gz::sim::Entity _entity)
{
  if (_entity == gz::sim::kNullEntity)
  {
    return gz::math::Vector3d::Zero;
  }

  const auto *velComp = _ecm.Component<gz::sim::components::LinearVelocity>(_entity);
  if (!velComp)
  {
    return gz::math::Vector3d::Zero;
  }

  const gz::math::Pose3d poseWorld = gz::sim::worldPose(_entity, _ecm);
  return poseWorld.Rot().RotateVector(velComp->Data());
}

gz::math::Vector3d ReadAngularVelocity(
  const gz::sim::EntityComponentManager &_ecm,
  gz::sim::Entity _entity)
{
  if (_entity == gz::sim::kNullEntity)
  {
    return gz::math::Vector3d::Zero;
  }

  const auto *velComp = _ecm.Component<gz::sim::components::AngularVelocity>(_entity);
  if (!velComp)
  {
    return gz::math::Vector3d::Zero;
  }

  const gz::math::Pose3d poseWorld = gz::sim::worldPose(_entity, _ecm);
  return poseWorld.Rot().RotateVector(velComp->Data());
}

std::string ReadEntityName(
  const gz::sim::EntityComponentManager &_ecm,
  const gz::sim::Entity _entity)
{
  if (_entity == gz::sim::kNullEntity)
  {
    return std::string();
  }

  const auto *nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
  if (!nameComp)
  {
    return std::string();
  }

  return nameComp->Data();
}

}  // namespace

sensor_msgs::msg::PointCloud2 RadarPointCloudBuilder::Build(
  const std::vector<RadarPoint> &_points,
  const std::string &_frameId,
  int64_t _simTimeNs)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = _frameId;
  cloud.header.stamp = ToRosTime(_simTimeNs);

  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(_points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    5,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "doppler_velocity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "rcs", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(_points.size());

  sensor_msgs::PointCloud2Iterator<float> iterX(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iterY(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iterZ(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iterDoppler(cloud, "doppler_velocity");
  sensor_msgs::PointCloud2Iterator<float> iterRcs(cloud, "rcs");

  for (const auto &pt : _points)
  {
    *iterX = pt.x;
    *iterY = pt.y;
    *iterZ = pt.z;
    *iterDoppler = pt.dopplerVelocity;
    *iterRcs = pt.rcs;

    ++iterX;
    ++iterY;
    ++iterZ;
    ++iterDoppler;
    ++iterRcs;
  }

  return cloud;
}

double RadarMath::RadialDopplerVelocity(
  const gz::math::Vector3d &_sensorWorldPos,
  const gz::math::Vector3d &_pointWorldPos,
  const gz::math::Vector3d &_targetLinearVelWorld,
  const gz::math::Vector3d &_egoLinearVelWorld,
  const gz::math::Vector3d &_egoAngularVelWorld,
  const gz::math::Vector3d &_sensorOffsetWorld)
{
  gz::math::Vector3d los = _pointWorldPos - _sensorWorldPos;
  const double dist = los.Length();
  if (dist < 1e-6)
  {
    return 0.0;
  }

  los /= dist;

  // v_sensor = v_ego + omega_ego x r_sensor
  const gz::math::Vector3d sensorVel =
    _egoLinearVelWorld + _egoAngularVelWorld.Cross(_sensorOffsetWorld);

  // Radial Doppler from relative velocity projection on LOS.
  // Positive means target moving away from radar.
  const gz::math::Vector3d relVel = _targetLinearVelWorld - sensorVel;
  return relVel.Dot(los);
}

float RadarMath::EstimateRcs(
  const std::string &_entityName,
  double _range,
  double _baseRcs,
  double _distanceDecay)
{
  double materialScale = 1.0;

  std::string nameLower = _entityName;
  std::transform(nameLower.begin(), nameLower.end(), nameLower.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (nameLower.find("metal") != std::string::npos ||
      nameLower.find("ship") != std::string::npos ||
      nameLower.find("hull") != std::string::npos)
  {
    materialScale = 1.6;
  }
  else if (nameLower.find("water") != std::string::npos)
  {
    materialScale = 0.5;
  }

  const double distanceGain = std::exp(-_distanceDecay * std::max(0.0, _range));
  return static_cast<float>(_baseRcs * materialScale * distanceGain);
}

FourDRadarPlugin::FourDRadarPlugin()
{
  rayCaster_ = std::make_unique<EcmAabbRayCaster>();
}

FourDRadarPlugin::~FourDRadarPlugin()
{
  // 禁止在此调用 rclcpp::shutdown()：同一 gz sim 进程内还有 ros_gz_bridge 等节点，shutdown 会拆掉全局 context。
}

void FourDRadarPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  pluginEntity_ = _entity;
  sensorEntity_ = _entity;
  egoModelEntity_ = gz::sim::topLevelModel(_entity, _ecm);
  if (egoModelEntity_ == gz::sim::kNullEntity)
  {
    egoModelEntity_ = _entity;
  }
  LoadParameters(_sdf);

  if (!rclcpp::ok())
  {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  rosNode_ = std::make_shared<rclcpp::Node>(SanitizeTopicToNodeName(topic_));
  cloudPub_ = rosNode_->create_publisher<sensor_msgs::msg::PointCloud2>(
    topic_,
    rclcpp::SensorDataQoS());
}

void FourDRadarPlugin::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || !cloudPub_)
  {
    return;
  }

  const int64_t simTimeNs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();

  const double publishPeriod = 1.0 / std::max(1e-3, updateRateHz_);
  if (lastPubTimeNs_ != 0)
  {
    const double dt = static_cast<double>(simTimeNs - lastPubTimeNs_) / 1e9;
    if (dt < publishPeriod)
    {
      return;
    }
  }

  ResolveEntities(_ecm);

  const gz::sim::Entity activeSensorEntity =
    (sensorEntity_ == gz::sim::kNullEntity) ? pluginEntity_ : sensorEntity_;
  const gz::math::Pose3d sensorPoseWorld = gz::sim::worldPose(activeSensorEntity, _ecm);
  const gz::math::Pose3d egoPoseWorld =
    (egoLinkEntity_ == gz::sim::kNullEntity) ? sensorPoseWorld :
    gz::sim::worldPose(egoLinkEntity_, _ecm);

  const gz::math::Vector3d egoLinearVel = ReadLinearVelocity(_ecm, egoLinkEntity_);
  const gz::math::Vector3d egoAngularVel = ReadAngularVelocity(_ecm, egoLinkEntity_);

  const std::vector<RaySpec> rays = BuildScanPattern(sensorPoseWorld);
  std::vector<RadarPoint> points;
  points.reserve(rays.size());

  static thread_local std::mt19937 rng(std::random_device{}());
  const double effMaxRange = EffectiveMaxRange();
  std::uniform_real_distribution<double> clutterGate(0.0, 1.0);
  std::normal_distribution<double> clutterNoise(0.0, seaClutterAmplitude_);

  for (size_t i = 0; i < rays.size(); ++i)
  {
    const auto &ray = rays[i];

    // 海杂波占据该角向格点：触发时仅输出杂波点，不再叠加同一条射线上的几何回波。
    const bool clutterOccupiesCell =
      enableSeaClutter_ && seaClutterProbability_ > 0.0 &&
      clutterGate(rng) <= seaClutterProbability_;

    if (clutterOccupiesCell)
    {
      const double clutterRange = std::min(effMaxRange, 0.2 * effMaxRange);
      gz::math::Vector3d p = ray.origin + ray.direction * clutterRange;
      p.Z() += clutterNoise(rng);

      RadarPoint clutterPt;
      clutterPt.x = static_cast<float>(p.X());
      clutterPt.y = static_cast<float>(p.Y());
      clutterPt.z = static_cast<float>(p.Z());
      clutterPt.dopplerVelocity = static_cast<float>(clutterNoise(rng));
      clutterPt.rcs = static_cast<float>(baseRcs_ * 0.15);
      points.push_back(clutterPt);
      continue;
    }

    const std::optional<RayHit> hit =
      rayCaster_->Cast(ray, _ecm, activeSensorEntity, egoModelEntity_);
    if (!hit.has_value())
    {
      continue;
    }

    if (hit->range < minRange_ || hit->range > effMaxRange)
    {
      continue;
    }

    RayHit h = hit.value();
    if (enableRangeMeasurementError_ || enableAzimuthMeasurementError_)
    {
      h = ApplyMeasurementErrors(h, sensorPoseWorld, rng);
      if (h.range < minRange_ || h.range > effMaxRange)
      {
        continue;
      }
    }

    const std::optional<RadarPoint> radarPoint =
      BuildRadarPoint(h, sensorPoseWorld, egoLinearVel, egoAngularVel, _ecm);
    if (radarPoint.has_value())
    {
      points.push_back(radarPoint.value());
    }
  }

  sensor_msgs::msg::PointCloud2 cloud =
    RadarPointCloudBuilder::Build(points, frameId_, simTimeNs);
  cloudPub_->publish(cloud);
  rclcpp::spin_some(rosNode_);

  lastPubTimeNs_ = simTimeNs;

  (void)egoPoseWorld;
}

void FourDRadarPlugin::LoadParameters(const std::shared_ptr<const sdf::Element> &_sdf)
{
  if (!_sdf)
  {
    return;
  }

  if (_sdf->HasElement("topic"))
  {
    topic_ = _sdf->Get<std::string>("topic");
  }
  if (_sdf->HasElement("frame_id"))
  {
    frameId_ = _sdf->Get<std::string>("frame_id");
  }
  if (_sdf->HasElement("ego_link_name"))
  {
    egoLinkName_ = _sdf->Get<std::string>("ego_link_name");
  }
  if (_sdf->HasElement("sensor_link_name"))
  {
    sensorLinkName_ = _sdf->Get<std::string>("sensor_link_name");
  }

  if (_sdf->HasElement("horizontal_fov_deg"))
  {
    horizontalFovRad_ = DegToRad(_sdf->Get<double>("horizontal_fov_deg"));
  }
  if (_sdf->HasElement("vertical_fov_deg"))
  {
    verticalFovRad_ = DegToRad(_sdf->Get<double>("vertical_fov_deg"));
  }
  if (_sdf->HasElement("azimuth_resolution_deg"))
  {
    azimuthResolutionRad_ = DegToRad(_sdf->Get<double>("azimuth_resolution_deg"));
  }
  if (_sdf->HasElement("elevation_resolution_deg"))
  {
    elevationResolutionRad_ = DegToRad(_sdf->Get<double>("elevation_resolution_deg"));
  }

  if (_sdf->HasElement("max_range"))
  {
    maxRange_ = _sdf->Get<double>("max_range");
  }
  if (_sdf->HasElement("min_range"))
  {
    minRange_ = _sdf->Get<double>("min_range");
  }
  if (_sdf->HasElement("update_rate_hz"))
  {
    updateRateHz_ = _sdf->Get<double>("update_rate_hz");
  }

  if (_sdf->HasElement("base_rcs"))
  {
    baseRcs_ = _sdf->Get<double>("base_rcs");
  }
  if (_sdf->HasElement("rcs_distance_decay"))
  {
    rcsDistanceDecay_ = _sdf->Get<double>("rcs_distance_decay");
  }

  if (_sdf->HasElement("enable_sea_clutter"))
  {
    enableSeaClutter_ = _sdf->Get<bool>("enable_sea_clutter");
  }
  if (_sdf->HasElement("sea_clutter_probability"))
  {
    seaClutterProbability_ = _sdf->Get<double>("sea_clutter_probability");
  }
  if (_sdf->HasElement("sea_clutter_amplitude"))
  {
    seaClutterAmplitude_ = _sdf->Get<double>("sea_clutter_amplitude");
  }

  if (_sdf->HasElement("perception_range_limit_m"))
  {
    perceptionRangeLimitM_ = _sdf->Get<double>("perception_range_limit_m");
  }
  if (_sdf->HasElement("enable_range_measurement_error"))
  {
    enableRangeMeasurementError_ = _sdf->Get<bool>("enable_range_measurement_error");
  }
  if (_sdf->HasElement("enable_azimuth_measurement_error"))
  {
    enableAzimuthMeasurementError_ = _sdf->Get<bool>("enable_azimuth_measurement_error");
  }
  if (_sdf->HasElement("output_in_sensor_frame"))
  {
    outputInSensorFrame_ = _sdf->Get<bool>("output_in_sensor_frame");
  }
  else if (_sdf->HasElement("output_points_in_sensor_frame"))
  {
    // 与旧版 xacro / 文档别名兼容
    outputInSensorFrame_ = _sdf->Get<bool>("output_points_in_sensor_frame");
  }
  if (_sdf->HasElement("range_error_at_reference_m"))
  {
    rangeErrorAtReferenceM_ = _sdf->Get<double>("range_error_at_reference_m");
  }
  if (_sdf->HasElement("range_error_reference_m"))
  {
    rangeErrorReferenceM_ = _sdf->Get<double>("range_error_reference_m");
  }
  if (_sdf->HasElement("azimuth_error_std_deg"))
  {
    azimuthErrorStdRad_ = DegToRad(_sdf->Get<double>("azimuth_error_std_deg"));
  }
}

void FourDRadarPlugin::ResolveEntities(const gz::sim::EntityComponentManager &_ecm)
{
  gz::sim::Model model(egoModelEntity_);
  if (model.Valid(_ecm))
  {
    if (!sensorLinkName_.empty())
    {
      const std::string shortSensor = BasenameLink(sensorLinkName_);
      const gz::sim::Entity sensorEnt = model.LinkByName(_ecm, shortSensor);
      gz::sim::Link sensorLink(sensorEnt);
      if (sensorLink.Valid(_ecm))
      {
        sensorEntity_ = sensorEnt;
      }
      else
      {
        const gz::sim::Entity e = FindEntityByName(_ecm, sensorLinkName_);
        if (e != gz::sim::kNullEntity)
        {
          sensorEntity_ = e;
        }
        else
        {
          const gz::sim::Entity e2 = FindEntityByName(_ecm, shortSensor);
          if (e2 != gz::sim::kNullEntity)
          {
            sensorEntity_ = e2;
          }
        }
      }
    }

    if (egoLinkEntity_ == gz::sim::kNullEntity && !egoLinkName_.empty())
    {
      const std::string shortEgo = BasenameLink(egoLinkName_);
      const gz::sim::Entity egoEnt = model.LinkByName(_ecm, shortEgo);
      gz::sim::Link egoLink(egoEnt);
      if (egoLink.Valid(_ecm))
      {
        egoLinkEntity_ = egoEnt;
      }
      else
      {
        egoLinkEntity_ = FindEntityByName(_ecm, egoLinkName_);
        if (egoLinkEntity_ == gz::sim::kNullEntity)
        {
          egoLinkEntity_ = FindEntityByName(_ecm, shortEgo);
        }
      }
    }
  }
  else
  {
    if (!sensorLinkName_.empty())
    {
      const gz::sim::Entity e = FindEntityByName(_ecm, sensorLinkName_);
      if (e != gz::sim::kNullEntity)
      {
        sensorEntity_ = e;
      }
    }
    if (egoLinkEntity_ == gz::sim::kNullEntity)
    {
      egoLinkEntity_ = FindEntityByName(_ecm, egoLinkName_);
    }
  }
}

double FourDRadarPlugin::EffectiveMaxRange() const noexcept
{
  return std::min(maxRange_, perceptionRangeLimitM_);
}

RayHit FourDRadarPlugin::ApplyMeasurementErrors(
  const RayHit &_hit,
  const gz::math::Pose3d &_sensorPoseWorld,
  std::mt19937 &_rng) const
{
  if (!enableRangeMeasurementError_ && !enableAzimuthMeasurementError_)
  {
    return _hit;
  }

  RayHit out = _hit;
  const gz::math::Vector3d origin = _sensorPoseWorld.Pos();
  const gz::math::Vector3d rel = _hit.point - origin;
  const double r = rel.Length();
  if (r < 1e-9)
  {
    return out;
  }

  gz::math::Vector3d dirLocal =
    _sensorPoseWorld.Rot().Inverse().RotateVector(rel / r);

  double el = std::asin(std::clamp(dirLocal.Z(), -1.0, 1.0));
  double az = std::atan2(dirLocal.Y(), dirLocal.X());

  if (enableAzimuthMeasurementError_)
  {
    std::normal_distribution<double> dAz(0.0, azimuthErrorStdRad_);
    az += dAz(_rng);
  }

  double rUses = r;
  if (enableRangeMeasurementError_)
  {
    const double ref = std::max(1e-3, rangeErrorReferenceM_);
    const double scale = std::min(r, ref) / ref;
    const double sigma = rangeErrorAtReferenceM_ * scale;
    std::normal_distribution<double> dr(0.0, std::max(1e-9, sigma));
    rUses = r + dr(_rng);
    const double rMax = EffectiveMaxRange();
    rUses = std::clamp(rUses, minRange_, rMax);
  }

  const double cosEl = std::cos(el);
  gz::math::Vector3d dirLocalNew(
    cosEl * std::cos(az),
    cosEl * std::sin(az),
    std::sin(el));
  dirLocalNew.Normalize();
  const gz::math::Vector3d dirWorld =
    _sensorPoseWorld.Rot().RotateVector(dirLocalNew);

  out.point = origin + dirWorld * rUses;
  out.range = rUses;
  return out;
}

std::vector<RaySpec> FourDRadarPlugin::BuildScanPattern(
  const gz::math::Pose3d &_sensorPoseWorld) const
{
  std::vector<RaySpec> rays;

  const int azBins = static_cast<int>(
    std::max(1.0, std::floor(horizontalFovRad_ / std::max(1e-4, azimuthResolutionRad_))));
  const int elBins = static_cast<int>(
    std::max(1.0, std::floor(verticalFovRad_ / std::max(1e-4, elevationResolutionRad_))));

  rays.reserve(static_cast<size_t>(azBins * elBins));

  for (int ei = 0; ei < elBins; ++ei)
  {
    const double el = -0.5 * verticalFovRad_ +
      (ei + 0.5) * (verticalFovRad_ / static_cast<double>(elBins));

    for (int ai = 0; ai < azBins; ++ai)
    {
      const double az = -0.5 * horizontalFovRad_ +
        (ai + 0.5) * (horizontalFovRad_ / static_cast<double>(azBins));

      const gz::math::Vector3d dirSensor(
        std::cos(el) * std::cos(az),
        std::cos(el) * std::sin(az),
        std::sin(el));
      const gz::math::Vector3d dirWorld =
        _sensorPoseWorld.Rot().RotateVector(dirSensor).Normalized();

      RaySpec ray;
      ray.origin = _sensorPoseWorld.Pos();
      ray.direction = dirWorld;
      ray.maxRange = EffectiveMaxRange();
      rays.push_back(ray);
    }
  }

  return rays;
}

std::optional<RadarPoint> FourDRadarPlugin::BuildRadarPoint(
  const RayHit &_hit,
  const gz::math::Pose3d &_sensorPoseWorld,
  const gz::math::Vector3d &_egoLinearVelWorld,
  const gz::math::Vector3d &_egoAngularVelWorld,
  const gz::sim::EntityComponentManager &_ecm) const
{
  const gz::math::Vector3d targetVel = ReadLinearVelocity(_ecm, _hit.entity);
  const gz::math::Pose3d egoPose =
    (egoLinkEntity_ == gz::sim::kNullEntity) ? _sensorPoseWorld :
    gz::sim::worldPose(egoLinkEntity_, _ecm);
  const gz::math::Vector3d sensorOffset = _sensorPoseWorld.Pos() - egoPose.Pos();

  RadarPoint p;
  gz::math::Vector3d outPoint = _hit.point;
  if (outputInSensorFrame_)
  {
    outPoint = _sensorPoseWorld.Rot().Inverse().RotateVector(_hit.point - _sensorPoseWorld.Pos());
  }
  p.x = static_cast<float>(outPoint.X());
  p.y = static_cast<float>(outPoint.Y());
  p.z = static_cast<float>(outPoint.Z());

  p.dopplerVelocity = static_cast<float>(RadarMath::RadialDopplerVelocity(
    _sensorPoseWorld.Pos(),
    _hit.point,
    targetVel,
    _egoLinearVelWorld,
    _egoAngularVelWorld,
    sensorOffset));

  const std::string entityName = ReadEntityName(_ecm, _hit.entity);
  p.rcs = RadarMath::EstimateRcs(entityName, _hit.range, baseRcs_, rcsDistanceDecay_);

  return p;
}

}  // namespace usv_4d_radar_gz

GZ_ADD_PLUGIN(
  usv_4d_radar_gz::FourDRadarPlugin,
  ::gz::sim::System,
  ::gz::sim::ISystemConfigure,
  ::gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  usv_4d_radar_gz::FourDRadarPlugin,
  "usv_4d_radar_gz::FourDRadarPlugin")
