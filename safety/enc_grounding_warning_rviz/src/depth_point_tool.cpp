#include "enc_grounding_warning_rviz/depth_point_tool.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace enc_grounding_warning_rviz
{

DepthPointTool::DepthPointTool()
{
  shortcut_key_ = 'w';
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/usv_1/safety/depth_point", "水深区域中心点话题", getPropertyContainer(),
    SLOT(updateTopic()), this);
}

void DepthPointTool::onInitialize()
{
  setName("添加水深点");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/PublishPoint.png"));
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  updateTopic();
}

void DepthPointTool::setTopic(const std::string & topic)
{
  topic_property_->setString(QString::fromStdString(topic));
}

void DepthPointTool::updateTopic()
{
  if (node_) {
    publisher_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
      topic_property_->getStdString(), 10);
  }
}

void DepthPointTool::activate()
{
  setStatus("单击地图设置一个水深区域后自动退出打点；右键或 Esc 取消。");
}

void DepthPointTool::deactivate() {}

int DepthPointTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.rightDown()) {
    return Finished;
  }
  if (context_->getFixedFrame() != "map") {
    setStatus("添加水深点要求 Fixed Frame 为 map。");
    return Render;
  }
  if (!event.leftUp() || !publisher_) {
    return 0;
  }
  const auto [valid, point] = projection_finder_.getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);
  if (!valid || !std::isfinite(point.x) || !std::isfinite(point.y)) {
    return 0;
  }
  geometry_msgs::msg::PointStamped message;
  message.header.frame_id = "map";
  message.header.stamp = node_->now();
  message.point.x = point.x;
  message.point.y = point.y;
  publisher_->publish(message);
  return Render | Finished;
}

}  // namespace enc_grounding_warning_rviz

PLUGINLIB_EXPORT_CLASS(enc_grounding_warning_rviz::DepthPointTool, rviz_common::Tool)
