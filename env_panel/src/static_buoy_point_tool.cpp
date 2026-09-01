#include "env_panel/static_buoy_point_tool.hpp"

#include <rviz_common/load_resource.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace env_panel
{

StaticBuoyPointTool::StaticBuoyPointTool()
: rviz_default_plugins::tools::PointTool()
{
  shortcut_key_ = 'b';
}

void StaticBuoyPointTool::onInitialize()
{
  PointTool::onInitialize();
  setName("Static Buoy Point");
  setIcon(rviz_common::loadPixmap(
      "package://rviz_default_plugins/icons/classes/PublishPoint.png"));
  topic_property_->setString("/dynamic_buoy/clicked_point");
  updateTopic();
}

}  // namespace env_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(env_panel::StaticBuoyPointTool, rviz_common::Tool)
