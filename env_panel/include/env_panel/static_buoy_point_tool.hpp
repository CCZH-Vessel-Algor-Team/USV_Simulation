#ifndef ENV_PANEL__STATIC_BUOY_POINT_TOOL_HPP_
#define ENV_PANEL__STATIC_BUOY_POINT_TOOL_HPP_

#include <QObject>

#include <rviz_default_plugins/tools/point/point_tool.hpp>

namespace env_panel
{

class StaticBuoyPointTool : public rviz_default_plugins::tools::PointTool
{
  Q_OBJECT

public:
  StaticBuoyPointTool();

  void onInitialize() override;
};

}  // namespace env_panel

#endif  // ENV_PANEL__STATIC_BUOY_POINT_TOOL_HPP_
