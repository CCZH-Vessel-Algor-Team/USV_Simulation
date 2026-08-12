#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <usv_interfaces/msg/hull_draft.hpp>

namespace env_panel
{

class HullDraftNode final : public rclcpp::Node
{
public:
  HullDraftNode()
  : Node("hull_draft_publisher")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/usv_1/odom");
    draft_topic_ = declare_parameter<std::string>("draft_topic", "~/draft");
    fluid_level_ = declare_parameter<double>("fluid_level", 0.0);
    hull_radius_ = declare_parameter<double>("hull_radius", 0.213);
    front_x_ = declare_parameter<double>("front_x", 0.6);
    aft_x_ = declare_parameter<double>("aft_x", -1.4);
    port_y_ = declare_parameter<double>("port_y", 1.03);
    starboard_y_ = declare_parameter<double>("starboard_y", -1.03);

    if (hull_radius_ <= 0.0) {
      throw std::invalid_argument("hull_radius must be positive");
    }

    draft_pub_ = create_publisher<usv_interfaces::msg::HullDraft>(draft_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HullDraftNode::onOdometry, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Publishing static-water draft from %s to %s",
      odom_topic_.c_str(), draft_topic_.c_str());
  }

private:
  struct Point
  {
    double x;
    double y;
  };

  double pointDraft(const Point & point, const nav_msgs::msg::Odometry & odom) const
  {
    const auto & q = odom.pose.pose.orientation;

    // This is the Z row of the body-to-world rotation used by Surface.cc.
    const double r20 = 2.0 * (q.x * q.z - q.w * q.y);
    const double r21 = 2.0 * (q.y * q.z + q.w * q.x);
    const double point_world_z = odom.pose.pose.position.z + r20 * point.x + r21 * point.y;
    return std::clamp(fluid_level_ - point_world_z, 0.0, hull_radius_);
  }

  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const double port_front = pointDraft({front_x_, port_y_}, *msg);
    const double port_aft = pointDraft({aft_x_, port_y_}, *msg);
    const double starboard_front = pointDraft({front_x_, starboard_y_}, *msg);
    const double starboard_aft = pointDraft({aft_x_, starboard_y_}, *msg);

    const double fore = 0.5 * (port_front + starboard_front);
    const double aft = 0.5 * (port_aft + starboard_aft);
    const double port = 0.5 * (port_front + port_aft);
    const double starboard = 0.5 * (starboard_front + starboard_aft);
    const double average = 0.25 * (port_front + port_aft + starboard_front + starboard_aft);
    const double trim = std::atan2(fore - aft, front_x_ - aft_x_);
    const double heel = std::atan2(port - starboard, port_y_ - starboard_y_);

    usv_interfaces::msg::HullDraft draft;
    draft.header = msg->header;
    draft.port_front = static_cast<float>(port_front);
    draft.port_aft = static_cast<float>(port_aft);
    draft.starboard_front = static_cast<float>(starboard_front);
    draft.starboard_aft = static_cast<float>(starboard_aft);
    draft.port = static_cast<float>(port);
    draft.starboard = static_cast<float>(starboard);
    draft.average_draft = static_cast<float>(average);
    draft.trim = static_cast<float>(trim);
    draft.heel = static_cast<float>(heel);
    draft_pub_->publish(draft);
  }

  std::string odom_topic_;
  std::string draft_topic_;
  double fluid_level_{0.0};
  double hull_radius_{0.213};
  double front_x_{0.6};
  double aft_x_{-1.4};
  double port_y_{1.03};
  double starboard_y_{-1.03};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<usv_interfaces::msg::HullDraft>::SharedPtr draft_pub_;
};

}  // namespace env_panel

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<env_panel::HullDraftNode>());
  rclcpp::shutdown();
  return 0;
}
