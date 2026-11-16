#ifndef ENV_PANEL__DYNAMIC_SHIP_TOOL_HPP_
#define ENV_PANEL__DYNAMIC_SHIP_TOOL_HPP_

#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rviz_default_plugins/tools/pose_estimate/initial_pose_tool.hpp>

namespace env_panel
{

// Reuse RViz pose placement; only single-click dispatch differs.
class DynamicShipTool : public rviz_default_plugins::tools::InitialPoseTool
{
public:
  DynamicShipTool();
  void onInitialize() override;
  void activate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  DynamicShipTool(
    const QString & name, const std::string & point_topic,
    const std::string & pose_topic, char shortcut);
  void onPoseSet(double x, double y, double theta) override;

private:
  QString tool_name_;
  std::string point_topic_;
  std::string pose_topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr virtual_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr virtual_pose_pub_;
  int press_x_ = 0;
  int press_y_ = 0;
  bool use_default_heading_ = true;
};

}  // namespace env_panel

#endif  // ENV_PANEL__DYNAMIC_SHIP_TOOL_HPP_
