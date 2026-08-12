#ifndef ENV_PANEL__STORM_FIELD_PANEL_HPP_
#define ENV_PANEL__STORM_FIELD_PANEL_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSlider>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
#include <usv_interfaces/srv/clear_storm_fields.hpp>
#include <usv_interfaces/srv/delete_storm_field.hpp>
#include <usv_interfaces/srv/set_storm_field_config.hpp>

namespace env_panel
{

class StormFieldPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit StormFieldPanel(QWidget * parent = nullptr);
  ~StormFieldPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onDriftHeadingChanged(int value);
  void onDriftSpeedChanged(int value);
  void onRadiusChanged(int value);
  void onWeatherValidityChanged(int value);
  void onWeatherGridChanged(int value);
  void onDeleteClicked();
  void onClearAllClicked();
  void publishConfig();

private:
  void setupUi();
  void onNamesReceived(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  rclcpp::Client<usv_interfaces::srv::SetStormFieldConfig>::SharedPtr config_client_;
  rclcpp::Client<usv_interfaces::srv::DeleteStormField>::SharedPtr delete_client_;
  rclcpp::Client<usv_interfaces::srv::ClearStormFields>::SharedPtr clear_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr names_sub_;

  double drift_heading_deg_;
  double drift_speed_;
  double radius_;
  double weather_validity_duration_s_;
  double weather_grid_resolution_m_;

  QSlider * drift_heading_slider_;
  QSlider * drift_speed_slider_;
  QSlider * radius_slider_;
  QSlider * weather_validity_slider_;
  QSlider * weather_grid_slider_;
  QLabel * drift_heading_label_;
  QLabel * drift_speed_label_;
  QLabel * radius_label_;
  QLabel * weather_validity_label_;
  QLabel * weather_grid_label_;
  QListWidget * storm_list_;
  QPushButton * clear_all_button_;
  QTimer * config_timer_;

  std::vector<std::string> storm_names_;
};

}  // namespace env_panel

#endif  // ENV_PANEL__STORM_FIELD_PANEL_HPP_
