#include "env_panel/storm_point_tool.hpp"

namespace env_panel
{

StormPointTool::StormPointTool()
: DynamicShipTool("添加风暴", "/storm_field/clicked_point", "/storm_field/spawn_pose", 's')
{
}

}  // namespace env_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(env_panel::StormPointTool, rviz_common::Tool)
