#ifndef ENV_PANEL__STORM_POINT_TOOL_HPP_
#define ENV_PANEL__STORM_POINT_TOOL_HPP_

#include <QObject>

#include <rviz_default_plugins/tools/point/point_tool.hpp>

namespace env_panel
{

class StormPointTool : public rviz_default_plugins::tools::PointTool
{
  Q_OBJECT

public:
  StormPointTool();

  void onInitialize() override;
};

}  // namespace env_panel

#endif  // ENV_PANEL__STORM_POINT_TOOL_HPP_
