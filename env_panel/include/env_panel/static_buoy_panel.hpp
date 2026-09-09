#ifndef ENV_PANEL__STATIC_BUOY_PANEL_HPP_
#define ENV_PANEL__STATIC_BUOY_PANEL_HPP_

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
#include <usv_interfaces/srv/clear_dynamic_buoys.hpp>
#include <usv_interfaces/srv/delete_dynamic_buoy.hpp>
#include <usv_interfaces/srv/set_dynamic_buoy_config.hpp>

namespace env_panel
{

class StaticBuoyPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit StaticBuoyPanel(QWidget * parent = nullptr);
  ~StaticBuoyPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onBuoyTypeChanged(const QString & text);
  void onRadiusChanged(int value);
  void onDeleteClicked();
  void onClearAllClicked();
  void publishConfig();

private:
  void setupUi();
  void onNamesReceived(const std_msgs::msg::String::SharedPtr msg);
  void updateBuoyList(const std::vector<std::string> & names);

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  rclcpp::Client<usv_interfaces::srv::DeleteDynamicBuoy>::SharedPtr delete_client_;
  rclcpp::Client<usv_interfaces::srv::ClearDynamicBuoys>::SharedPtr clear_client_;
  rclcpp::Client<usv_interfaces::srv::SetDynamicBuoyConfig>::SharedPtr config_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr names_sub_;

  std::string buoy_type_;
  double radius_m_;

  QComboBox * buoy_type_combo_;
  QSlider * radius_slider_;
  QLabel * radius_label_;
  QListWidget * buoy_list_;
  QPushButton * clear_all_button_;
  QTimer * config_timer_;

  std::vector<std::string> buoy_names_;
  std::string last_names_json_;
};

}  // namespace env_panel

#endif  // ENV_PANEL__STATIC_BUOY_PANEL_HPP_
