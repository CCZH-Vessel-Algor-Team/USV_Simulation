// 将 gpu_ray + ros_gz_bridge 的 PointCloud2 增强为 x,y,z,doppler_velocity,rcs
#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

}  // namespace

class Mmwave4dCloudNode final : public rclcpp::Node
{
public:
  Mmwave4dCloudNode()
  : Node("mmwave_4d_cloud_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "");
    output_topic_ = declare_parameter<std::string>("output_topic", "");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    // Reliable 与 RViz/部分工具默认订阅 QoS 兼容；Best effort 订阅方仍可接收 Reliable 发布
    output_use_reliable_qos_ = declare_parameter<bool>("output_use_reliable_qos", true);

    base_rcs_ = declare_parameter<double>("base_rcs", 12.0);
    rcs_distance_decay_ = declare_parameter<double>("rcs_distance_decay", 0.01);
    perception_range_limit_m_ = declare_parameter<double>("perception_range_limit_m", 300.0);
    enable_sea_clutter_ = declare_parameter<bool>("enable_sea_clutter", false);
    sea_clutter_probability_ = declare_parameter<double>("sea_clutter_probability", 0.0);
    sea_clutter_amplitude_ = declare_parameter<double>("sea_clutter_amplitude", 0.0);
    enable_range_measurement_error_ =
      declare_parameter<bool>("enable_range_measurement_error", false);
    enable_azimuth_measurement_error_ =
      declare_parameter<bool>("enable_azimuth_measurement_error", false);
    range_error_at_reference_m_ = declare_parameter<double>("range_error_at_reference_m", 0.66);
    range_error_reference_m_ = declare_parameter<double>("range_error_reference_m", 300.0);
    azimuth_error_std_deg_ = declare_parameter<double>("azimuth_error_std_deg", 0.5);

    if (input_topic_.empty() || output_topic_.empty()) {
      RCLCPP_FATAL(get_logger(), "input_topic and output_topic must be set");
      throw std::invalid_argument("mmwave_4d_cloud_node topics");
    }
    if (odom_topic_.empty()) {
      RCLCPP_WARN(get_logger(), "odom_topic empty: doppler will be 0");
    }

    rclcpp::QoS pub_qos(rclcpp::KeepLast(10));
    if (output_use_reliable_qos_) {
      pub_qos.reliable();
    } else {
      pub_qos.best_effort();
    }
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, pub_qos);
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Mmwave4dCloudNode::onCloud, this, std::placeholders::_1));

    if (!odom_topic_.empty()) {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(10),
        std::bind(&Mmwave4dCloudNode::onOdom, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      get_logger(),
      "mmwave_4d_cloud: in=%s out=%s odom=%s output_qos=%s",
      input_topic_.c_str(), output_topic_.c_str(), odom_topic_.c_str(),
      output_use_reliable_qos_ ? "reliable" : "best_effort");
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) { last_odom_ = msg; }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const std::string sensor_frame = msg->header.frame_id;
    geometry_msgs::msg::TransformStamped world_T_sensor;
    geometry_msgs::msg::TransformStamped world_T_base;
    bool have_tf = true;
    // 使用 TimePointZero 取「最新可用」TF，避免点云 header 时间略超前于 TF 树时的
    // “extrapolation into the future”（仿真步进与桥接延迟常见）。
    try {
      world_T_sensor = tf_buffer_.lookupTransform(
        world_frame_, sensor_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "TF %s->%s: %s", world_frame_.c_str(), sensor_frame.c_str(), e.what());
      have_tf = false;
    }

    tf2::Quaternion q_ws(0, 0, 0, 1);
    tf2::Vector3 p_ws(0, 0, 0);
    if (have_tf) {
      tf2::fromMsg(world_T_sensor.transform.rotation, q_ws);
      p_ws = tf2::Vector3(
        world_T_sensor.transform.translation.x,
        world_T_sensor.transform.translation.y,
        world_T_sensor.transform.translation.z);
    }

    tf2::Vector3 v_sensor_world(0, 0, 0);
    if (last_odom_ && have_tf) {
      std::string bf = last_odom_->child_frame_id;
      bool have_base = false;
      if (!bf.empty()) {
        try {
          world_T_base = tf_buffer_.lookupTransform(
            world_frame_, bf, tf2::TimePointZero, tf2::durationFromSec(0.1));
          have_base = true;
        } catch (const tf2::TransformException &e) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "TF base %s: %s", bf.c_str(), e.what());
        }
      }
      if (have_base) {
        tf2::Quaternion q_wb;
        tf2::fromMsg(world_T_base.transform.rotation, q_wb);
        const tf2::Vector3 p_wb(
          world_T_base.transform.translation.x,
          world_T_base.transform.translation.y,
          world_T_base.transform.translation.z);

        const auto &tw = last_odom_->twist.twist;
        tf2::Vector3 v_b(tw.linear.x, tw.linear.y, tw.linear.z);
        tf2::Vector3 w_b(tw.angular.x, tw.angular.y, tw.angular.z);
        tf2::Vector3 v_base_world = tf2::quatRotate(q_wb, v_b);
        tf2::Vector3 w_world = tf2::quatRotate(q_wb, w_b);
        tf2::Vector3 r_world = p_ws - p_wb;
        v_sensor_world = v_base_world + w_world.cross(r_world);
      }
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = 1;
    out.is_bigendian = msg->is_bigendian;
    out.is_dense = false;

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2Fields(
      5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "doppler_velocity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rcs", 1, sensor_msgs::msg::PointField::FLOAT32);

    std::uniform_real_distribution<double> uni01(0.0, 1.0);
    std::normal_distribution<double> n_range(0.0, 1.0);
    std::normal_distribution<double> n_az(0.0, 1.0);
    std::normal_distribution<double> n_clutter(0.0, 1.0);

    std::vector<float> xs, ys, zs, dops, rcsv;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
      for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        tf2::Vector3 p_s(*ix, *iy, *iz);
        double range = p_s.length();
        if (range < 1e-6 || range > perception_range_limit_m_) {
          continue;
        }

        bool clutter = enable_sea_clutter_ && sea_clutter_probability_ > 0.0 &&
          uni01(rng_) <= sea_clutter_probability_;
        if (clutter) {
          const double cr = std::min(perception_range_limit_m_, 0.2 * perception_range_limit_m_);
          const double u = uni01(rng_);
          const double v = uni01(rng_);
          const double el = (u - 0.5) * 0.5;
          const double az = (v - 0.5) * 1.2;
          const double ce = std::cos(el);
          p_s = tf2::Vector3(
            cr * ce * std::cos(az), cr * ce * std::sin(az), cr * std::sin(el));
          p_s.setZ(p_s.z() + static_cast<float>(n_clutter(rng_) * sea_clutter_amplitude_));
        }

        double azimuth = std::atan2(p_s.y(), p_s.x());
        double el_ratio = p_s.z() / std::max(1e-9, p_s.length());
        el_ratio = std::clamp(el_ratio, -1.0, 1.0);
        double elevation = std::asin(el_ratio);
        range = p_s.length();

        if (!clutter && enable_azimuth_measurement_error_) {
          azimuth += n_az(rng_) * (azimuth_error_std_deg_ * kPi / 180.0);
        }
        if (!clutter && enable_range_measurement_error_) {
          const double ref = std::max(1e-3, range_error_reference_m_);
          const double scale = std::min(range, ref) / ref;
          range = std::max(1e-3, range + n_range(rng_) * range_error_at_reference_m_ * scale);
          const double ce = std::cos(elevation);
          p_s = tf2::Vector3(
            range * ce * std::cos(azimuth),
            range * ce * std::sin(azimuth),
            range * std::sin(elevation));
        }

        double doppler = 0.0;
        if (have_tf) {
          const tf2::Vector3 p_w = tf2::quatRotate(q_ws, p_s) + p_ws;
          tf2::Vector3 los = p_w - p_ws;
          const double L = los.length();
          if (L > 1e-6) {
            los /= L;
            doppler = (-v_sensor_world).dot(los);
          }
        }

        const float rcs = static_cast<float>(
          base_rcs_ * std::exp(-rcs_distance_decay_ * std::max(0.0, p_s.length())));

        xs.push_back(static_cast<float>(p_s.x()));
        ys.push_back(static_cast<float>(p_s.y()));
        zs.push_back(static_cast<float>(p_s.z()));
        dops.push_back(static_cast<float>(doppler));
        rcsv.push_back(rcs);
      }
    } catch (const std::runtime_error &e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "PointCloud2 field error: %s", e.what());
      out.width = 0;
      mod.resize(0);
      pub_->publish(out);
      return;
    }

    out.width = static_cast<uint32_t>(xs.size());
    mod.resize(out.width);
    sensor_msgs::PointCloud2Iterator<float> ox(out, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(out, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(out, "z");
    sensor_msgs::PointCloud2Iterator<float> od(out, "doppler_velocity");
    sensor_msgs::PointCloud2Iterator<float> orcs(out, "rcs");
    for (size_t i = 0; i < xs.size(); ++i) {
      *ox = xs[i];
      *oy = ys[i];
      *oz = zs[i];
      *od = dops[i];
      *orcs = rcsv[i];
      ++ox;
      ++oy;
      ++oz;
      ++od;
      ++orcs;
    }

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_topic_;
  std::string world_frame_;
  bool output_use_reliable_qos_{true};
  double base_rcs_{12.0};
  double rcs_distance_decay_{0.01};
  double perception_range_limit_m_{300.0};
  bool enable_sea_clutter_{false};
  double sea_clutter_probability_{0.0};
  double sea_clutter_amplitude_{0.0};
  bool enable_range_measurement_error_{false};
  bool enable_azimuth_measurement_error_{false};
  double range_error_at_reference_m_{0.66};
  double range_error_reference_m_{300.0};
  double azimuth_error_std_deg_{0.5};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mt19937 rng_{std::random_device{}()};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mmwave4dCloudNode>());
  rclcpp::shutdown();
  return 0;
}
