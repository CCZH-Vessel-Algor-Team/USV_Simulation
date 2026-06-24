/*
 * Maritime Radar Gazebo Plugin
 * 独立 .so 库，可直接在 SDF 中引用
 */

#ifndef GZ_MARITIME_RADAR_PLUGIN_HH_
#define GZ_MARITIME_RADAR_PLUGIN_HH_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/float_v.pb.h>

namespace gz_maritime_radar
{

/// \brief 海事雷达仿真插件
/// 
/// SDF 参数:
/// - <joint_name>: 旋转关节名称 (必需)
/// - <lidar_topic>: LiDAR 输入话题
/// - <radar_topic>: 雷达输出话题
/// - <angular_resolution>: 角度分辨率 (rad), 默认 0.0251327 (~1.44°)
/// - <linear_resolution>: 距离分辨率 (m), 默认 0.75
/// - <min_range>: 最小距离 (m), 默认 1.0
/// - <max_range>: 最大距离 (m), 默认 1500.0
/// - <min_elevation_angle>: 最小仰角 (rad), 默认 -0.02 (~-1.15°)
///                          用于过滤水面杂波，低于此角度的射线将被忽略
///
class MaritimeRadarPlugin :
      public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate,
      public gz::sim::ISystemPostUpdate
{
public:
  MaritimeRadarPlugin();
  ~MaritimeRadarPlugin() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  void OnLidarScan(const gz::msgs::LaserScan &_msg);
  void PublishSpoke();
  void ClearBin(std::size_t _index);

  // 配置参数
  double minRange_{1.0};
  double maxRange_{500.0};
  double linearResolution_{1};
  double angularResolution_{0.0251327};
  double minElevationAngle_{-0.02};  // 最小仰角，用于过滤水面杂波 (rad)

  // 运行状态
  std::size_t numBeams_{0};
  std::size_t currentSpokeIndex_{0};
  std::vector<std::vector<double>> radarBins_;
  std::size_t numSpokes_{0};
  std::size_t numRangeBins_{0};
  bool lidarInitialized_{false};

  // 话题
  std::string lidarTopic_;
  std::string radarTopic_;

  // Gazebo 实体
  gz::sim::Entity modelEntity_{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity_{gz::sim::kNullEntity};

  // 传输
  gz::transport::Node node_;
  gz::transport::Node::Publisher radarPub_;
  std::mutex mutex_;
};

}  // namespace gz_maritime_radar

#endif