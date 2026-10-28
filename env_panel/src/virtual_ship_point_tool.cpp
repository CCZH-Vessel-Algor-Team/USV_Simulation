#include "env_panel/virtual_ship_point_tool.hpp"

#include <rviz_common/load_resource.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace env_panel
{

VirtualShipPointTool::VirtualShipPointTool()
: rviz_default_plugins::tools::PointTool()
{
  shortcut_key_ = 'v';
}

void VirtualShipPointTool::onInitialize()
{
  PointTool::onInitialize();
  setName("虚拟目标船取点");
  setIcon(rviz_common::loadPixmap(
      "package://rviz_default_plugins/icons/classes/PublishPoint.png"));
  topic_property_->setString("/dynamic_ship/virtual_clicked_point");
  updateTopic();
}

}  // namespace env_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(env_panel::VirtualShipPointTool, rviz_common::Tool)
