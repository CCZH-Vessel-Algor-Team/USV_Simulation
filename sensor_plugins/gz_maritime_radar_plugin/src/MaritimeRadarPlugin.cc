/*
 * Maritime Radar Gazebo Plugin Implementation
 */

#include "MaritimeRadarPlugin.hh"

#include <cmath>
#include <algorithm>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/common/Console.hh>

using namespace gz_maritime_radar;

// 注册插件
GZ_ADD_PLUGIN(MaritimeRadarPlugin,
              gz::sim::System,
              MaritimeRadarPlugin::ISystemConfigure,
              MaritimeRadarPlugin::ISystemPreUpdate,
              MaritimeRadarPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(MaritimeRadarPlugin, "gz::sim::systems::MaritimeRadarPlugin")
GZ_ADD_PLUGIN_ALIAS(MaritimeRadarPlugin, "maritime_radar")

MaritimeRadarPlugin::MaritimeRadarPlugin() = default;

void MaritimeRadarPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->modelEntity_ = _entity;
  auto model = gz::sim::Model(_entity);
  auto modelName = model.Name(_ecm);

  gzmsg << "MaritimeRadarPlugin: Configuring for model [" << modelName << "]" << std::endl;

  // 读取参数
  if (_sdf->HasElement("angular_resolution"))
    this->angularResolution_ = _sdf->Get<double>("angular_resolution");

  if (_sdf->HasElement("linear_resolution"))
    this->linearResolution_ = _sdf->Get<double>("linear_resolution");

  if (_sdf->HasElement("min_range"))
    this->minRange_ = _sdf->Get<double>("min_range");

  if (_sdf->HasElement("max_range"))
    this->maxRange_ = _sdf->Get<double>("max_range");

  // 最小仰角过滤（用于消除水面杂波）
  // 默认 -0.02 rad ≈ -1.15°，略低于水平线，过滤大部分水面回波
  if (_sdf->HasElement("min_elevation_angle"))
    this->minElevationAngle_ = _sdf->Get<double>("min_elevation_angle");

  // 关节名称 (必需)
  std::string jointName;
  if (_sdf->HasElement("joint_name"))
  {
    jointName = _sdf->Get<std::string>("joint_name");
  }
  else
  {
    gzerr << "MaritimeRadarPlugin: <joint_name> required!" << std::endl;
    return;
  }

  // 查找关节
  this->jointEntity_ = model.JointByName(_ecm, jointName);
  if (this->jointEntity_ == gz::sim::kNullEntity)
  {
    gzerr << "MaritimeRadarPlugin: Joint [" << jointName << "] not found!" << std::endl;
    return;
  }

  // 话题配置
  if (_sdf->HasElement("lidar_topic"))
    this->lidarTopic_ = _sdf->Get<std::string>("lidar_topic");
  else
    this->lidarTopic_ = "/" + modelName + "/lidar";

  if (_sdf->HasElement("radar_topic"))
    this->radarTopic_ = _sdf->Get<std::string>("radar_topic");
  else
    this->radarTopic_ = "/" + modelName + "/radar/spokes";

  // 初始化雷达数据结构
  this->numSpokes_ = static_cast<std::size_t>(std::ceil(2.0 * M_PI / this->angularResolution_));
  this->numRangeBins_ = static_cast<std::size_t>(
      std::ceil((this->maxRange_ - this->minRange_) / this->linearResolution_));

  this->radarBins_.resize(this->numSpokes_);
  for (auto &bin : this->radarBins_)
  {
    bin.resize(this->numRangeBins_, 0.0);
  }

  // 创建发布者
  this->radarPub_ = this->node_.Advertise<gz::msgs::Float_V>(this->radarTopic_);

  gzmsg << "MaritimeRadarPlugin: Configured successfully" << std::endl;
  gzmsg << "  LiDAR topic: " << this->lidarTopic_ << std::endl;
  gzmsg << "  Radar topic: " << this->radarTopic_ << std::endl;
  gzmsg << "  Spokes: " << this->numSpokes_ << ", Range bins: " << this->numRangeBins_ << std::endl;
}

void MaritimeRadarPlugin::PreUpdate(
    const gz::sim::UpdateInfo &/*_info*/,
    gz::sim::EntityComponentManager &_ecm)
{
  // 确保关节位置组件存在
  if (this->jointEntity_ != gz::sim::kNullEntity)
  {
    if (!_ecm.Component<gz::sim::components::JointPosition>(this->jointEntity_))
    {
      _ecm.CreateComponent(this->jointEntity_, gz::sim::components::JointPosition());
    }
  }
}

void MaritimeRadarPlugin::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // 初始化 LiDAR 订阅
  if (!this->lidarInitialized_)
  {
    if (this->node_.Subscribe(this->lidarTopic_, &MaritimeRadarPlugin::OnLidarScan, this))
    {
      this->lidarInitialized_ = true;
      gzmsg << "MaritimeRadarPlugin: Subscribed to LiDAR topic" << std::endl;
    }
    return;
  }

  // 获取关节位置
  auto jointPosComp = _ecm.Component<gz::sim::components::JointPosition>(this->jointEntity_);
  if (!jointPosComp || jointPosComp->Data().empty())
    return;

  double jointAngle = jointPosComp->Data()[0];

  // 标准化到 [0, 2π)
  while (jointAngle < 0)
    jointAngle += 2.0 * M_PI;
  while (jointAngle >= 2.0 * M_PI)
    jointAngle -= 2.0 * M_PI;

  // 将逆时针角度转换为顺时针惯例（海事雷达标准）
  // Gazebo: CCW 为正 → 海事雷达: CW 为正
  double radarAngle = 2.0 * M_PI - jointAngle;
  if (radarAngle >= 2.0 * M_PI)
    radarAngle -= 2.0 * M_PI;

  // 计算当前 spoke 索引（使用转换后的角度）
  std::size_t spokeIndex = static_cast<std::size_t>(radarAngle / this->angularResolution_);
  spokeIndex = std::min(spokeIndex, this->numSpokes_ - 1);

  // 如果进入新的 spoke，发布上一个
  if (spokeIndex != this->currentSpokeIndex_)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (this->numBeams_ > 0)
    {
      this->PublishSpoke();
    }

    this->currentSpokeIndex_ = spokeIndex;
    this->ClearBin(spokeIndex);
    this->numBeams_ = 0;
  }
}

void MaritimeRadarPlugin::OnLidarScan(const gz::msgs::LaserScan &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  // 累积 LiDAR 数据到当前 spoke
  for (int i = 0; i < _msg.ranges_size(); ++i)
  {
    double range = _msg.ranges(i);

    if (range < this->minRange_ || range > this->maxRange_ ||
        std::isinf(range) || std::isnan(range))
      continue;

    // 计算垂直角度
    double verticalAngle = _msg.vertical_angle_min() +
                           i * _msg.vertical_angle_step();

    // 过滤水面杂波：忽略低于最小仰角的射线
    // 负角度表示向下，水面回波主要来自向下的射线
    if (verticalAngle < this->minElevationAngle_)
      continue;

    // 投影到水平面 (2D)
    double projectedRange = std::cos(verticalAngle) * range;

    if (projectedRange < this->minRange_ || projectedRange > this->maxRange_)
      continue;

    // 计算距离 bin 索引
    std::size_t rangeBin = static_cast<std::size_t>(
        (projectedRange - this->minRange_) / this->linearResolution_);

    if (rangeBin < this->numRangeBins_)
    {
      this->radarBins_[this->currentSpokeIndex_][rangeBin] += 1.0;
    }
  }

  this->numBeams_++;
}

void MaritimeRadarPlugin::PublishSpoke()
{
  gz::msgs::Float_V msg;

  // 元数据 - spokeIndex 已经是顺时针惯例下的索引
  double spokeAngle = this->currentSpokeIndex_ * this->angularResolution_;
  msg.add_data(static_cast<float>(spokeAngle));
  msg.add_data(static_cast<float>(this->angularResolution_));
  msg.add_data(static_cast<float>(this->linearResolution_));

  // 强度数据 (转换为 dB)
  const auto &bin = this->radarBins_[this->currentSpokeIndex_];
  for (std::size_t i = 0; i < this->numRangeBins_; ++i)
  {
    double intensity = bin[i] / static_cast<double>(this->numBeams_);
    double dB = (intensity > 0.0) ? 10.0 * std::log10(intensity) : -100.0;
    dB = std::max(-100.0, std::min(0.0, dB));
    msg.add_data(static_cast<float>(dB));
  }

  this->radarPub_.Publish(msg);
}

void MaritimeRadarPlugin::ClearBin(std::size_t _index)
{
  if (_index < this->radarBins_.size())
  {
    std::fill(this->radarBins_[_index].begin(),
              this->radarBins_[_index].end(), 0.0);
  }
}