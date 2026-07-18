#ifndef ENV_PANEL__DYNAMIC_SHIP_PANEL_HPP_
#define ENV_PANEL__DYNAMIC_SHIP_PANEL_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QComboBox>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSlider>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
#include <usv_interfaces/msg/tracked_ship_list.hpp>
#include <usv_interfaces/srv/delete_dynamic_ship.hpp>
#include <usv_interfaces/srv/clear_dynamic_ships.hpp>
#include <usv_interfaces/srv/set_dynamic_ship_config.hpp>

namespace env_panel
{

class DynamicShipPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DynamicShipPanel(QWidget * parent = nullptr);
  ~DynamicShipPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onHeadingChanged(int value);
  void onSpeedChanged(int value);
  void onHalfDistChanged(int value);
  void onShapeChanged(const QString & text);
  void onDeleteClicked();
  void onClearAllClicked();
  void publishConfig();

private:
  void setupUi();
  void setupRos();
  void onNamesReceived(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  rclcpp::Client<usv_interfaces::srv::DeleteDynamicShip>::SharedPtr
      delete_client_;
  rclcpp::Client<usv_interfaces::srv::ClearDynamicShips>::SharedPtr
      clear_client_;
  rclcpp::Client<usv_interfaces::srv::SetDynamicShipConfig>::SharedPtr
      config_client_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr names_sub_;

  double heading_deg_;
  double speed_;
  double half_distance_;
  std::string shape_;

  QSlider * heading_slider_;
  QSlider * speed_slider_;
  QSlider * half_dist_slider_;
  QComboBox * shape_combo_;
  QLabel * heading_label_;
  QLabel * speed_label_;
  QLabel * half_dist_label_;
  QListWidget * ship_list_;
  QPushButton * clear_all_button_;
  QTimer * config_timer_;

  std::vector<std::string> ship_names_;
};

}  // namespace env_panel

#endif  // ENV_PANEL__DYNAMIC_SHIP_PANEL_HPP_
