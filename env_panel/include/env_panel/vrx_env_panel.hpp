#ifndef ENV_PANEL__VRX_ENV_PANEL_HPP_
#define ENV_PANEL__VRX_ENV_PANEL_HPP_

#include <memory>

#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>

#include <geometry_msgs/msg/twist.hpp>
#include <ros_gz_interfaces/msg/param_vec.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/float64.hpp>

namespace env_panel
{

class VrxEnvPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit VrxEnvPanel(QWidget * parent = nullptr);
  ~VrxEnvPanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void onWindSpeedChanged(int value);
  void onWindDirectionChanged(int value);
  void onWaveAmplitudeChanged(int value);
  void onWavePeriodChanged(int value);
  void onCurrentXChanged(int value);
  void onCurrentYChanged(int value);
  void onPhysicsCouplingToggled(bool checked);
  void onEmergencyStopClicked();
  void onResetClicked();
  void onRampTimer();

private:
  struct EnvState
  {
    double wind_speed;
    double wind_direction_deg;
    double wave_amplitude;
    double wave_period;
    double current_x;
    double current_y;
  };

  void setupUi();
  void setupPublishers();
  void updateLabels();
  void updateCouplingPreviewLabels();

  static double clampStep(double current, double target, double max_step);

  rclcpp::Node::SharedPtr raw_node_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wind_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wind_direction_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wave_amplitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wave_period_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr current_pub_;
  rclcpp::Publisher<ros_gz_interfaces::msg::ParamVec>::SharedPtr wavefield_param_pub_;

  QSlider * wind_speed_slider_;
  QSlider * wind_direction_slider_;
  QSlider * wave_amplitude_slider_;
  QSlider * wave_period_slider_;
  QSlider * current_x_slider_;
  QSlider * current_y_slider_;

  QLabel * wind_speed_label_;
  QLabel * wind_direction_label_;
  QLabel * wave_amplitude_label_;
  QLabel * wave_period_label_;
  QLabel * current_x_label_;
  QLabel * current_y_label_;

  QLabel * coupling_amplitude_label_;
  QLabel * coupling_period_label_;
  QLabel * coupling_current_speed_label_;
  QLabel * coupling_current_direction_label_;

  QLabel * coupling_published_amplitude_label_;
  QLabel * coupling_published_period_label_;
  QLabel * coupling_published_current_speed_label_;
  QLabel * coupling_published_current_direction_label_;

  QCheckBox * physics_coupling_checkbox_;

  QPushButton * emergency_stop_button_;
  QPushButton * reset_button_;

  QTimer * ramp_timer_;

  bool physics_coupling_enabled_;

  EnvState target_values_;
  EnvState current_published_values_;
};

}  // namespace env_panel

#endif  // ENV_PANEL__VRX_ENV_PANEL_HPP_
