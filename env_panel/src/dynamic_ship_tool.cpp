#include "env_panel/dynamic_ship_tool.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

namespace env_panel
{

DynamicShipTool::DynamicShipTool()
: DynamicShipTool("添加动态船", "/dynamic_ship/spawn_point", "/dynamic_ship/spawn_pose", 'd')
{
}

DynamicShipTool::DynamicShipTool(
  const QString & name, const std::string & point_topic,
  const std::string & pose_topic, char shortcut)
: tool_name_(name), point_topic_(point_topic), pose_topic_(pose_topic)
{
  shortcut_key_ = shortcut;
}

void DynamicShipTool::onInitialize()
{
  InitialPoseTool::onInitialize();
  setName(tool_name_);
  getPropertyContainer()->subProp("Topic")->setValue(QString::fromStdString(pose_topic_));
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  point_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
    point_topic_, rclcpp::QoS(10));
  if (point_topic_ == "/dynamic_ship/spawn_point") {
    virtual_point_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
      "/dynamic_ship/virtual_clicked_point", rclcpp::QoS(10));
    virtual_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/dynamic_ship/virtual_spawn_pose", rclcpp::QoS(10));
  }
}

void DynamicShipTool::activate()
{
  PoseTool::activate();
  setStatus(context_->getFixedFrame() == "map" ?
    "单击：面板默认方向；拖拽：箭头方向；右键取消；Esc 退出。" :
    tool_name_ + "要求 Fixed Frame 为 map。");
}

int DynamicShipTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.rightDown() || context_->getFixedFrame() != "map") {
    deactivate();
    activate();
    return Render;
  }
  if (event.leftDown()) {
    press_x_ = event.x;
    press_y_ = event.y;
    angle_ = 0.0;
  }
  if (event.leftUp() && state_ == Orientation) {
    const auto [valid, point] = projection_finder_->getViewportPointProjectionOnXYPlane(
      event.panel->getRenderWindow(), event.x, event.y);
    use_default_heading_ = std::hypot(
      static_cast<double>(event.x) - press_x_, static_cast<double>(event.y) - press_y_) < 6.0;
    const auto direction = valid ? point - arrow_position_ : Ogre::Vector3::ZERO;
    if (!valid || !std::isfinite(point.x) || !std::isfinite(point.y) ||
      (!use_default_heading_ && std::hypot(direction.x, direction.y) < 1e-6))
    {
      deactivate();
      activate();
      return Render;
    }
    // PoseTool previews during movement; use the actual release point for heading.
    if (!use_default_heading_) {
      angle_ = std::atan2(direction.y, direction.x);
    }
  }
  const int flags = PoseTool::processMouseEvent(event);
  if (flags & Finished) {
    deactivate();
    activate();  // Keep the same button active for consecutive placements.
  }
  return flags & ~Finished;
}

void DynamicShipTool::onPoseSet(double x, double y, double theta)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(theta)) {
    return;
  }
  const bool virtual_ship = virtual_point_pub_ &&
    context_->property("env_panel.virtual_ship").toBool();
  if (!use_default_heading_) {
    if (virtual_ship) {
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header.frame_id = "map";
      msg.header.stamp = node_->now();
      msg.pose.pose.position.x = x;
      msg.pose.pose.position.y = y;
      msg.pose.pose.orientation.z = std::sin(theta / 2.0);
      msg.pose.pose.orientation.w = std::cos(theta / 2.0);
      virtual_pose_pub_->publish(msg);
    } else {
      InitialPoseTool::onPoseSet(x, y, theta);
    }
    return;
  }
  geometry_msgs::msg::PointStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();
  msg.point.x = x;
  msg.point.y = y;
  (virtual_ship ? virtual_point_pub_ : point_pub_)->publish(msg);
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::DynamicShipTool, rviz_common::Tool)
