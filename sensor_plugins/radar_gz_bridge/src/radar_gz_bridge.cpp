/*
 * Radar Gazebo to ROS2 Bridge
 * 
 * 将 Gazebo gz.msgs.Float_V 雷达数据转换为 marine_sensor_msgs/RadarSector
 */

#include <memory>
#include <string>
#include <chrono>
#include <cmath>

#include <gz/msgs/float_v.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <marine_sensor_msgs/msg/radar_sector.hpp>
#include <marine_sensor_msgs/msg/radar_echo.hpp>

using namespace std::chrono_literals;

class RadarGzBridge : public rclcpp::Node
{
public:
  RadarGzBridge() : Node("radar_gz_bridge")
  {
    // 声明参数
    this->declare_parameter<std::string>("gz_topic", "/blueboat/radar/spokes");
    this->declare_parameter<std::string>("ros_topic", "/sensors/radar/nav/sector");
    this->declare_parameter<std::string>("frame_id", "nav_radar_link");
    this->declare_parameter<double>("range_min", 5.0);
    this->declare_parameter<double>("range_max", 500.0);
    this->declare_parameter<double>("rotation_period", 1.25);  // 24 RPM ≈ 2.5s
    this->declare_parameter<bool>("log_publish_statistics", false);

    // 获取参数
    this->get_parameter("gz_topic", gz_topic_);
    this->get_parameter("ros_topic", ros_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("range_min", range_min_);
    this->get_parameter("range_max", range_max_);
    this->get_parameter("rotation_period", rotation_period_);
    bool log_publish_statistics = false;
    this->get_parameter("log_publish_statistics", log_publish_statistics);

    RCLCPP_INFO(this->get_logger(), "=== Radar Gazebo Bridge ===");
    RCLCPP_INFO(this->get_logger(), "  GZ topic:  %s", gz_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  ROS topic: %s", ros_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Frame ID:  %s", frame_id_.c_str());

    // 创建 ROS2 发布者
    radar_pub_ = this->create_publisher<marine_sensor_msgs::msg::RadarSector>(
      ros_topic_, rclcpp::SensorDataQoS());

    // 订阅 Gazebo 话题
    if (!gz_node_.Subscribe(gz_topic_, &RadarGzBridge::OnGzMsg, this))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to GZ topic: %s", gz_topic_.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Subscribed to GZ topic successfully");
    }

    if (log_publish_statistics)
    {
      stats_timer_ = this->create_wall_timer(
        5s, std::bind(&RadarGzBridge::PrintStats, this));
    }
  }

private:
  /// \brief Gazebo 消息回调
  /// 消息格式: [angle, angular_res, linear_res, dB_0, dB_1, ...]
  void OnGzMsg(const gz::msgs::Float_V &_msg)
  {
    if (_msg.data_size() < 4)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Invalid message size: %d", _msg.data_size());
      return;
    }

    // 解析元数据（与 gz 海事雷达 Float_V 约定一致: [angle, angular_res, linear_res, dB...]）
    float angle = _msg.data(0);           // 当前角度 (rad)
    float angular_res = _msg.data(1);     // 角度分辨率 (rad)
    [[maybe_unused]] const float linear_res = _msg.data(2);  // 距离分辨率 (m)；RadarSector 无对应字段，保留供后续扩展
    int num_bins = _msg.data_size() - 3;  // 距离 bin 数量

    // 构建 RadarSector 消息
    marine_sensor_msgs::msg::RadarSector sector_msg;

    // Header
    sector_msg.header.stamp = this->now();
    sector_msg.header.frame_id = frame_id_;

    // 扇区参数
    sector_msg.angle_start = angle;
    sector_msg.angle_increment = angular_res;

    // 时间参数
    // time_increment: 射线之间的时间 (单射线时为 0)
    sector_msg.time_increment.sec = 0;
    sector_msg.time_increment.nanosec = 0;

    // scan_time: 完整旋转周期
    double scan_time_ns = rotation_period_ * 1e9;
    sector_msg.scan_time.sec = static_cast<int32_t>(scan_time_ns / 1e9);
    sector_msg.scan_time.nanosec = static_cast<uint32_t>(
      static_cast<int64_t>(scan_time_ns) % 1000000000);

    // 距离参数
    sector_msg.range_min = static_cast<float>(range_min_);
    sector_msg.range_max = static_cast<float>(range_max_);

    // 构建 RadarEcho (一条射线的回波数据)
    marine_sensor_msgs::msg::RadarEcho echo;
    echo.echoes.reserve(num_bins);

    for (int i = 0; i < num_bins; ++i)
    {
      float dB = _msg.data(3 + i);

      // dB 转换为归一化强度 [0, 1]
      // dB 范围: [-100, 0] -> intensity: [0, 15]
      float intensity;
      if (dB <= -100.0f)
      {
        intensity = 0.0f;
      }
      else if (dB >= 0.0f)
      {
        intensity = 15.0f;
      }
      else
      {
        intensity = (dB + 100.0f) / 100.0f;
      }

      echo.echoes.push_back(intensity);
    }

    // 添加到 intensities 数组
    sector_msg.intensities.push_back(echo);

    // 发布
    radar_pub_->publish(sector_msg);

    msg_count_++;
  }

  void PrintStats()
  {
    RCLCPP_INFO(this->get_logger(), "Published %zu radar sectors (%.1f Hz)",
                msg_count_, msg_count_ / 5.0);
    msg_count_ = 0;
  }

  // ROS2
  rclcpp::Publisher<marine_sensor_msgs::msg::RadarSector>::SharedPtr radar_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // Gazebo
  gz::transport::Node gz_node_;

  // 参数
  std::string gz_topic_;
  std::string ros_topic_;
  std::string frame_id_;
  double range_min_;
  double range_max_;
  double rotation_period_;

  // 统计
  size_t msg_count_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RadarGzBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
