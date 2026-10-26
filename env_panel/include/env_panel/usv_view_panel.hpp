#ifndef ENV_PANEL__USV_VIEW_PANEL_HPP_
#define ENV_PANEL__USV_VIEW_PANEL_HPP_

#include <QPushButton>
#include <QString>

#include <rviz_common/panel.hpp>

namespace rviz_common
{
class ViewController;
class ViewManager;
}  // namespace rviz_common

namespace env_panel
{

class UsvViewPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit UsvViewPanel(QWidget * parent = nullptr);
  ~UsvViewPanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void showGlobalTopDown();
  void showFollowTopDown();
  void showThirdPersonFollow();

private:
  void setupUi();
  rviz_common::ViewController * switchController(const QString & class_id);
  bool setViewProperty(
    rviz_common::ViewController * controller,
    const QString & name,
    const QVariant & value);
  bool setVectorProperty(
    rviz_common::ViewController * controller,
    const QString & name,
    double x, double y, double z);
  rviz_common::ViewManager * view_manager_;
  QPushButton * global_top_down_button_;
  QPushButton * follow_top_down_button_;
  QPushButton * third_person_button_;
};

}  // namespace env_panel

#endif  // ENV_PANEL__USV_VIEW_PANEL_HPP_
