#ifndef ENC_GROUNDING_WARNING_RVIZ__DEPTH_SETTINGS_PANEL_HPP_
#define ENC_GROUNDING_WARNING_RVIZ__DEPTH_SETTINGS_PANEL_HPP_

#include <QtWidgets>
#undef NO_ERROR

#include <chrono>
#include <deque>
#include <string>

#include "enc_grounding_warning_msgs/srv/configure_depth.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

namespace enc_grounding_warning_rviz
{

class DepthSettingsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DepthSettingsPanel(QWidget * parent = nullptr);
  ~DepthSettingsPanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onApplyNamespace();
  void onRemoveRegion();

private:
  using Service = enc_grounding_warning_msgs::srv::ConfigureDepth;
  using Settings = enc_grounding_warning_msgs::msg::DepthSettings;
  enum class Operation {Read, Add, Remove};
  void updateConnections();
  void updateControls();
  void bindPointTool();
  void receivePoint(const geometry_msgs::msg::PointStamped & point);
  void processNextPoint();
  void requestSettings(Operation operation, const Settings & settings = Settings());
  void showSettings(const Settings & settings);

  rclcpp::Node::SharedPtr node_;
  std::string ns_ = "usv_1";
  QTimer * timer_ = nullptr;
  QLineEdit * ns_edit_;
  QDoubleSpinBox * radius_edit_;
  QDoubleSpinBox * depth_edit_;
  QTableWidget * regions_table_;
  QPushButton * remove_region_button_;
  QPushButton * remove_all_button_;
  QLabel * status_label_;
  Settings current_settings_;
  std::deque<enc_grounding_warning_msgs::msg::DepthShoal> pending_regions_;
  bool settings_loaded_ = false;
  bool request_pending_ = false;
  Operation pending_operation_ = Operation::Read;
  int64_t request_id_ = -1;
  uint64_t generation_ = 0;
  std::chrono::steady_clock::time_point request_deadline_;
  std::chrono::steady_clock::time_point query_after_;
  rclcpp::Client<Service>::SharedPtr client_;
  rclcpp::Subscription<Settings>::SharedPtr settings_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
};

}  // namespace enc_grounding_warning_rviz

#endif  // ENC_GROUNDING_WARNING_RVIZ__DEPTH_SETTINGS_PANEL_HPP_
