#ifndef ENV_PANEL__VIRTUAL_SHIP_POINT_TOOL_HPP_
#define ENV_PANEL__VIRTUAL_SHIP_POINT_TOOL_HPP_

#include <rviz_default_plugins/tools/point/point_tool.hpp>

namespace env_panel
{

class VirtualShipPointTool : public rviz_default_plugins::tools::PointTool
{
  Q_OBJECT

public:
  VirtualShipPointTool();

  void onInitialize() override;
};

}  // namespace env_panel

#endif  // ENV_PANEL__VIRTUAL_SHIP_POINT_TOOL_HPP_
