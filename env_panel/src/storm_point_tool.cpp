#include "env_panel/storm_point_tool.hpp"

#include <rviz_common/load_resource.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace env_panel
{

StormPointTool::StormPointTool()
: rviz_default_plugins::tools::PointTool()
{
  shortcut_key_ = 's';
}

void StormPointTool::onInitialize()
{
  PointTool::onInitialize();
  setName("Storm Point");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/PublishPoint.png"));
  topic_property_->setString("/storm_field/clicked_point");
  updateTopic();
}

}  // namespace env_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(env_panel::StormPointTool, rviz_common::Tool)
