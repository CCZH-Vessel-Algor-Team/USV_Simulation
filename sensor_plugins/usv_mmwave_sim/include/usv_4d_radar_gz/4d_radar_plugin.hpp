#ifndef USV_4D_RADAR_GZ_4D_RADAR_PLUGIN_HPP_
#define USV_4D_RADAR_GZ_4D_RADAR_PLUGIN_HPP_

#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>

namespace usv_4d_radar_gz
{

struct RadarPoint
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float dopplerVelocity{0.0f};
  float rcs{0.0f};
};

struct RaySpec
{
  gz::math::Vector3d origin;
  gz::math::Vector3d direction;
  double maxRange{0.0};
};

struct RayHit
{
  gz::sim::Entity entity{gz::sim::kNullEntity};
  gz::math::Vector3d point;
  double range{0.0};
};

class IRayCaster
{
public:
  virtual ~IRayCaster() = default;

  virtual std::optional<RayHit> Cast(
    const RaySpec &_ray,
    const gz::sim::EntityComponentManager &_ecm,
    gz::sim::Entity _selfEntity,
    gz::sim::Entity _ignoreTopLevelModel) const = 0;
};

class RadarPointCloudBuilder
{
public:
  static sensor_msgs::msg::PointCloud2 Build(
    const std::vector<RadarPoint> &_points,
    const std::string &_frameId,
    int64_t _simTimeNs);
};

class RadarMath
{
public:
  static double RadialDopplerVelocity(
    const gz::math::Vector3d &_sensorWorldPos,
    const gz::math::Vector3d &_pointWorldPos,
    const gz::math::Vector3d &_targetLinearVelWorld,
    const gz::math::Vector3d &_egoLinearVelWorld,
    const gz::math::Vector3d &_egoAngularVelWorld,
    const gz::math::Vector3d &_sensorOffsetWorld);

  static float EstimateRcs(
    const std::string &_entityName,
    double _range,
    double _baseRcs,
    double _distanceDecay);
};

class FourDRadarPlugin final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  FourDRadarPlugin();
  ~FourDRadarPlugin() override;

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) override;

private:
  void LoadParameters(const std::shared_ptr<const sdf::Element> &_sdf);

  void ResolveEntities(const gz::sim::EntityComponentManager &_ecm);

  std::vector<RaySpec> BuildScanPattern(
    const gz::math::Pose3d &_sensorPoseWorld) const;

  std::optional<RadarPoint> BuildRadarPoint(
    const RayHit &_hit,
    const gz::math::Pose3d &_sensorPoseWorld,
    const gz::math::Vector3d &_egoLinearVelWorld,
    const gz::math::Vector3d &_egoAngularVelWorld,
    const gz::sim::EntityComponentManager &_ecm) const;

  double EffectiveMaxRange() const noexcept;

  RayHit ApplyMeasurementErrors(
    const RayHit &_hit,
    const gz::math::Pose3d &_sensorPoseWorld,
    std::mt19937 &_rng) const;

  std::unique_ptr<IRayCaster> rayCaster_;

  rclcpp::Node::SharedPtr rosNode_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;

  gz::sim::Entity pluginEntity_{gz::sim::kNullEntity};
  gz::sim::Entity sensorEntity_{gz::sim::kNullEntity};
  gz::sim::Entity egoModelEntity_{gz::sim::kNullEntity};
  gz::sim::Entity egoLinkEntity_{gz::sim::kNullEntity};

  std::string topic_{"/radar/points_4d"};
  std::string frameId_{"radar_link"};
  std::string egoLinkName_{"base_link"};
  std::string sensorLinkName_;

  double horizontalFovRad_{2.0};
  double verticalFovRad_{0.35};
  double azimuthResolutionRad_{0.01};
  double elevationResolutionRad_{0.01};
  double maxRange_{250.0};
  double minRange_{0.5};
  double updateRateHz_{20.0};

  double baseRcs_{10.0};
  double rcsDistanceDecay_{0.015};

  bool enableSeaClutter_{false};
  double seaClutterProbability_{0.01};
  double seaClutterAmplitude_{0.2};

  /// 感知距离上限（米）；有效最大作用距离为 min(max_range, 本值)。
  double perceptionRangeLimitM_{300.0};
  bool enableRangeMeasurementError_{false};
  bool enableAzimuthMeasurementError_{false};
  /// 为 true 时，x/y/z 以雷达坐标系输出；为 false 时按世界坐标输出。
  bool outputInSensorFrame_{false};
  /// 在 range_error_reference_m 距离处，距离测量高斯噪声的 1σ（米）；近处按 r/参考距离比例缩小。
  double rangeErrorAtReferenceM_{0.66};
  double rangeErrorReferenceM_{300.0};
  /// 方位角测量高斯噪声 1σ（弧度），由 azimuth_error_std_deg 加载。
  double azimuthErrorStdRad_{0.008726646259971648};  // 0.5°

  int64_t lastPubTimeNs_{0};
};

}  // namespace usv_4d_radar_gz

#endif  // USV_4D_RADAR_GZ_4D_RADAR_PLUGIN_HPP_
