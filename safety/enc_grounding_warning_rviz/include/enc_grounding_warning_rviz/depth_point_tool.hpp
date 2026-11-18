#ifndef ENC_GROUNDING_WARNING_RVIZ__DEPTH_POINT_TOOL_HPP_
#define ENC_GROUNDING_WARNING_RVIZ__DEPTH_POINT_TOOL_HPP_

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

namespace rviz_common::properties
{
class StringProperty;
}

namespace enc_grounding_warning_rviz
{

class DepthPointTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  DepthPointTool();
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  void setTopic(const std::string & topic);

private Q_SLOTS:
  void updateTopic();

private:
  rviz_common::properties::StringProperty * topic_property_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  rviz_rendering::ViewportProjectionFinder projection_finder_;
};

}  // namespace enc_grounding_warning_rviz

#endif  // ENC_GROUNDING_WARNING_RVIZ__DEPTH_POINT_TOOL_HPP_
