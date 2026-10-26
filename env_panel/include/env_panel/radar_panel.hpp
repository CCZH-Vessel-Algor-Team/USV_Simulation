#ifndef ENV_PANEL__RADAR_PANEL_HPP_
#define ENV_PANEL__RADAR_PANEL_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>

#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace env_panel
{

class RadarPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RadarPanel(QWidget * parent = nullptr);
  ~RadarPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void checkConnection();
  void refreshParameters();
  void applyAllParameters();
  void queryRadarState();
  void onRangeChanged(int index);
  void onRpmChanged(int index);
  void onGainAutoToggled(bool checked);
  void onSeaAutoToggled(bool checked);
  void onSideAutoToggled(bool checked);
  void onGainReleased();
  void onSeaReleased();
  void onSideReleased();
  void onRainReleased();
  void onInsIpEditingFinished();

private:
  void setupUi();
  void setupRos();
  void setIntegerParameter(const std::string & name, int64_t value);
  void setBooleanParameter(const std::string & name, bool value);
  void setStringParameter(const std::string & name, const std::string & value);
  void sendParameters(
    const std::vector<rcl_interfaces::msg::Parameter> & parameters,
    const QString & success_text);
  void callPowerService(bool enabled);
  void callTransmitService(bool enabled);
  void handleParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg);
  void updateParameter(
    const std::string & name,
    const rcl_interfaces::msg::ParameterValue & value);
  void updateAutoSlider(
    int64_t value, QCheckBox * automatic, QSlider * slider, QLabel * label);
  void updateConnectionStatus(bool connected);
  void updateOperationStatus(const QString & text, bool success);

  static std::vector<std::string> parameterNames();
  static rcl_interfaces::msg::Parameter makeIntegerParameter(
    const std::string & name, int64_t value);
  static rcl_interfaces::msg::Parameter makeBooleanParameter(
    const std::string & name, bool value);
  static rcl_interfaces::msg::Parameter makeStringParameter(
    const std::string & name, const std::string & value);

  const std::string target_node_{"/radar_control_node"};

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr power_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr transmit_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr query_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  QComboBox * range_combo_;
  QComboBox * rpm_combo_;
  QCheckBox * gain_auto_checkbox_;
  QSlider * gain_slider_;
  QLabel * gain_value_label_;
  QCheckBox * sea_auto_checkbox_;
  QSlider * sea_slider_;
  QLabel * sea_value_label_;
  QCheckBox * side_auto_checkbox_;
  QSlider * side_slider_;
  QLabel * side_value_label_;
  QSlider * rain_slider_;
  QLabel * rain_value_label_;
  QSpinBox * beam_spinbox_;
  QSpinBox * interference_spinbox_;
  QSpinBox * noise_spinbox_;
  QSpinBox * boost_spinbox_;
  QCheckBox * stretch_checkbox_;
  QCheckBox * ins_enable_checkbox_;
  QLineEdit * ins_ip_edit_;
  QSpinBox * ins_port_spinbox_;
  QLabel * connection_label_;
  QLabel * radar_state_label_;
  QTimer * connection_timer_;
  bool connected_;
};

}  // namespace env_panel

#endif  // ENV_PANEL__RADAR_PANEL_HPP_
