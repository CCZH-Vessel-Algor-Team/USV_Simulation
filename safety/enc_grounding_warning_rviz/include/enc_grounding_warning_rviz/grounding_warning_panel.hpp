#ifndef ENC_GROUNDING_WARNING_RVIZ__GROUNDING_WARNING_PANEL_HPP_
#define ENC_GROUNDING_WARNING_RVIZ__GROUNDING_WARNING_PANEL_HPP_

#include <QtWidgets>
#undef NO_ERROR

#include <memory>
#include <string>

#include "enc_grounding_warning_msgs/msg/depth_grid.hpp"
#include "enc_grounding_warning_msgs/msg/grounding_alert.hpp"
#include "enc_grounding_warning_msgs/msg/ukc_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

class QLineEdit;
class QLabel;
class QPushButton;
class QTimer;

namespace enc_grounding_warning_rviz
{

/// RViz panel showing UKC / grounding alert / depth grid status.
class GroundingWarningPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GroundingWarningPanel(QWidget * parent = 0);
  ~GroundingWarningPanel() override;

  void onInitialize() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onApplyNamespace();

private:
  void updateSubscriptions();
  void setRiskStyle(QLabel * label, uint8_t risk);

  rclcpp::Node::SharedPtr node_;
  std::string ns_;
  QTimer * timer_;

  QLineEdit * ns_edit_;
  QPushButton * apply_button_;
  QLabel * ukc_label_;
  QLabel * alert_label_;
  QLabel * grid_label_;

  rclcpp::Subscription<enc_grounding_warning_msgs::msg::UKCState>::SharedPtr ukc_sub_;
  rclcpp::Subscription<enc_grounding_warning_msgs::msg::GroundingAlert>::SharedPtr alert_sub_;
  rclcpp::Subscription<enc_grounding_warning_msgs::msg::DepthGrid>::SharedPtr grid_sub_;
};

}  // namespace enc_grounding_warning_rviz

#endif  // ENC_GROUNDING_WARNING_RVIZ__GROUNDING_WARNING_PANEL_HPP_
