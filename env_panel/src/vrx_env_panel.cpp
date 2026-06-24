#include "env_panel/vrx_env_panel.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace env_panel
{

namespace
{
constexpr double kRampHz = 10.0;
constexpr double kDtSec = 1.0 / kRampHz;

constexpr double kWindSpeedRatePerSec = 1.0;
constexpr double kWindDirectionRatePerSec = 30.0;
constexpr double kWaveAmplitudeRatePerSec = 0.2;
constexpr double kWavePeriodRatePerSec = 1.0;
constexpr double kCurrentRatePerSec = 0.5;

constexpr double kWindSpeedScale = 10.0;
constexpr double kWaveScale = 100.0;
constexpr double kCurrentScale = 100.0;

constexpr double kWaveGainFromAmplitudeScale = 1.0 / 3.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

constexpr double kGravity = 9.81;
constexpr double kPi = 3.14159;
}  // namespace

VrxEnvPanel::VrxEnvPanel(QWidget * parent)
: rviz_common::Panel(parent),
  wind_speed_slider_(nullptr),
  wind_direction_slider_(nullptr),
  wave_amplitude_slider_(nullptr),
  wave_period_slider_(nullptr),
  current_x_slider_(nullptr),
  current_y_slider_(nullptr),
  wind_speed_label_(nullptr),
  wind_direction_label_(nullptr),
  wave_amplitude_label_(nullptr),
  wave_period_label_(nullptr),
  current_x_label_(nullptr),
  current_y_label_(nullptr),
  coupling_amplitude_label_(nullptr),
  coupling_period_label_(nullptr),
  coupling_current_speed_label_(nullptr),
  coupling_current_direction_label_(nullptr),
  coupling_published_amplitude_label_(nullptr),
  coupling_published_period_label_(nullptr),
  coupling_published_current_speed_label_(nullptr),
  coupling_published_current_direction_label_(nullptr),
  physics_coupling_checkbox_(nullptr),
  emergency_stop_button_(nullptr),
  reset_button_(nullptr),
  ramp_timer_(nullptr),
  physics_coupling_enabled_(false),
  target_values_{0.0, 0.0, 0.0, 5.0, 0.0, 0.0},
  current_published_values_{0.0, 0.0, 0.0, 5.0, 0.0, 0.0}
{
  setupUi();

  ramp_timer_ = new QTimer(this);
  connect(ramp_timer_, &QTimer::timeout, this, &VrxEnvPanel::onRampTimer);
  ramp_timer_->start(static_cast<int>(1000.0 / kRampHz));
}

void VrxEnvPanel::onInitialize()
{
  auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    return;
  }

  raw_node_ = ros_node_abstraction->get_raw_node();
  setupPublishers();
}

void VrxEnvPanel::setupUi()
{
  auto * main_layout = new QVBoxLayout();

  physics_coupling_checkbox_ =
    new QCheckBox("Enable Physics Coupling (真实海况联动)", this);
  physics_coupling_checkbox_->setChecked(false);
  main_layout->addWidget(physics_coupling_checkbox_);

  auto * coupling_preview_group = new QGroupBox("Physics Coupling Preview", this);
  auto * coupling_preview_layout = new QGridLayout();
  coupling_amplitude_label_ = new QLabel("0.00 m", coupling_preview_group);
  coupling_period_label_ = new QLabel("0.00 s", coupling_preview_group);
  coupling_current_speed_label_ = new QLabel("0.00 m/s", coupling_preview_group);
  coupling_current_direction_label_ = new QLabel("0 deg", coupling_preview_group);

  coupling_published_amplitude_label_ = new QLabel("0.00 m", coupling_preview_group);
  coupling_published_period_label_ = new QLabel("0.00 s", coupling_preview_group);
  coupling_published_current_speed_label_ = new QLabel("0.00 m/s", coupling_preview_group);
  coupling_published_current_direction_label_ = new QLabel("0 deg", coupling_preview_group);

  coupling_preview_layout->addWidget(new QLabel("Metric", coupling_preview_group), 0, 0);
  coupling_preview_layout->addWidget(new QLabel("Derived Target", coupling_preview_group), 0, 1);
  coupling_preview_layout->addWidget(new QLabel("Current Published", coupling_preview_group), 0, 2);

  coupling_preview_layout->addWidget(new QLabel("Wave Amplitude", coupling_preview_group), 1, 0);
  coupling_preview_layout->addWidget(coupling_amplitude_label_, 1, 1);
  coupling_preview_layout->addWidget(coupling_published_amplitude_label_, 1, 2);

  coupling_preview_layout->addWidget(new QLabel("Wave Period", coupling_preview_group), 2, 0);
  coupling_preview_layout->addWidget(coupling_period_label_, 2, 1);
  coupling_preview_layout->addWidget(coupling_published_period_label_, 2, 2);

  coupling_preview_layout->addWidget(new QLabel("Current Speed", coupling_preview_group), 3, 0);
  coupling_preview_layout->addWidget(coupling_current_speed_label_, 3, 1);
  coupling_preview_layout->addWidget(coupling_published_current_speed_label_, 3, 2);

  coupling_preview_layout->addWidget(new QLabel("Current Direction", coupling_preview_group), 4, 0);
  coupling_preview_layout->addWidget(coupling_current_direction_label_, 4, 1);
  coupling_preview_layout->addWidget(coupling_published_current_direction_label_, 4, 2);

  coupling_preview_group->setLayout(coupling_preview_layout);
  main_layout->addWidget(coupling_preview_group);

  auto * wind_group = new QGroupBox("Wind", this);
  auto * wind_layout = new QFormLayout();

  wind_speed_slider_ = new QSlider(Qt::Horizontal, wind_group);
  wind_speed_slider_->setRange(0, static_cast<int>(20.0 * kWindSpeedScale));
  wind_speed_slider_->setValue(0);
  wind_speed_label_ = new QLabel("0.0 m/s", wind_group);

  auto * wind_speed_row = new QHBoxLayout();
  wind_speed_row->addWidget(wind_speed_slider_);
  wind_speed_row->addWidget(wind_speed_label_);
  wind_layout->addRow("Speed", wind_speed_row);

  wind_direction_slider_ = new QSlider(Qt::Horizontal, wind_group);
  wind_direction_slider_->setRange(-180, 180);
  wind_direction_slider_->setValue(0);
  wind_direction_label_ = new QLabel("0 deg", wind_group);

  auto * wind_dir_row = new QHBoxLayout();
  wind_dir_row->addWidget(wind_direction_slider_);
  wind_dir_row->addWidget(wind_direction_label_);
  wind_layout->addRow("Direction", wind_dir_row);

  wind_group->setLayout(wind_layout);
  main_layout->addWidget(wind_group);

  auto * wave_group = new QGroupBox("Waves", this);
  auto * wave_layout = new QFormLayout();

  wave_amplitude_slider_ = new QSlider(Qt::Horizontal, wave_group);
  wave_amplitude_slider_->setRange(0, static_cast<int>(3.0 * kWaveScale));
  wave_amplitude_slider_->setValue(0);
  wave_amplitude_label_ = new QLabel("0.00 m", wave_group);

  auto * wave_amp_row = new QHBoxLayout();
  wave_amp_row->addWidget(wave_amplitude_slider_);
  wave_amp_row->addWidget(wave_amplitude_label_);
  wave_layout->addRow("Amplitude", wave_amp_row);

  wave_period_slider_ = new QSlider(Qt::Horizontal, wave_group);
  wave_period_slider_->setRange(static_cast<int>(1.0 * kWaveScale), static_cast<int>(20.0 * kWaveScale));
  wave_period_slider_->setValue(static_cast<int>(5.0 * kWaveScale));
  wave_period_label_ = new QLabel("5.00 s", wave_group);

  auto * wave_period_row = new QHBoxLayout();
  wave_period_row->addWidget(wave_period_slider_);
  wave_period_row->addWidget(wave_period_label_);
  wave_layout->addRow("Period", wave_period_row);

  wave_group->setLayout(wave_layout);
  main_layout->addWidget(wave_group);

  auto * current_group = new QGroupBox("Current", this);
  auto * current_layout = new QFormLayout();

  current_x_slider_ = new QSlider(Qt::Horizontal, current_group);
  current_x_slider_->setRange(
    static_cast<int>(-5.0 * kCurrentScale), static_cast<int>(5.0 * kCurrentScale));
  current_x_slider_->setValue(0);
  current_x_label_ = new QLabel("0.00 m/s", current_group);

  auto * current_x_row = new QHBoxLayout();
  current_x_row->addWidget(current_x_slider_);
  current_x_row->addWidget(current_x_label_);
  current_layout->addRow("X", current_x_row);

  current_y_slider_ = new QSlider(Qt::Horizontal, current_group);
  current_y_slider_->setRange(
    static_cast<int>(-5.0 * kCurrentScale), static_cast<int>(5.0 * kCurrentScale));
  current_y_slider_->setValue(0);
  current_y_label_ = new QLabel("0.00 m/s", current_group);

  auto * current_y_row = new QHBoxLayout();
  current_y_row->addWidget(current_y_slider_);
  current_y_row->addWidget(current_y_label_);
  current_layout->addRow("Y", current_y_row);

  current_group->setLayout(current_layout);
  main_layout->addWidget(current_group);

  auto * control_row = new QHBoxLayout();
  emergency_stop_button_ = new QPushButton("Emergency Stop", this);
  reset_button_ = new QPushButton("Reset Defaults", this);
  control_row->addWidget(emergency_stop_button_);
  control_row->addWidget(reset_button_);
  main_layout->addLayout(control_row);

  main_layout->addStretch();
  setLayout(main_layout);

  connect(wind_speed_slider_, &QSlider::valueChanged, this, &VrxEnvPanel::onWindSpeedChanged);
  connect(
    wind_direction_slider_, &QSlider::valueChanged, this, &VrxEnvPanel::onWindDirectionChanged);
  connect(
    wave_amplitude_slider_, &QSlider::valueChanged, this, &VrxEnvPanel::onWaveAmplitudeChanged);
  connect(wave_period_slider_, &QSlider::valueChanged, this, &VrxEnvPanel::onWavePeriodChanged);
  connect(current_x_slider_, &QSlider::valueChanged, this, &VrxEnvPanel::onCurrentXChanged);
  connect(current_y_slider_, &QSlider::valueChanged, this, &VrxEnvPanel::onCurrentYChanged);
  connect(
    physics_coupling_checkbox_,
    &QCheckBox::toggled,
    this,
    &VrxEnvPanel::onPhysicsCouplingToggled);
  connect(emergency_stop_button_, &QPushButton::clicked, this, &VrxEnvPanel::onEmergencyStopClicked);
  connect(reset_button_, &QPushButton::clicked, this, &VrxEnvPanel::onResetClicked);

  updateLabels();
}

void VrxEnvPanel::setupPublishers()
{
  if (!raw_node_) {
    return;
  }

  wind_speed_pub_ = raw_node_->create_publisher<std_msgs::msg::Float64>(
    "/vrx/environment/wind/speed", rclcpp::QoS(10));
  wind_direction_pub_ = raw_node_->create_publisher<std_msgs::msg::Float64>(
    "/vrx/environment/wind/direction_deg", rclcpp::QoS(10));
  wave_amplitude_pub_ = raw_node_->create_publisher<std_msgs::msg::Float64>(
    "/vrx/environment/waves/amplitude", rclcpp::QoS(10));
  wave_period_pub_ = raw_node_->create_publisher<std_msgs::msg::Float64>(
    "/vrx/environment/waves/period", rclcpp::QoS(10));
  current_pub_ = raw_node_->create_publisher<geometry_msgs::msg::Twist>(
    "/vrx/environment/current", rclcpp::QoS(10));
  wavefield_param_pub_ = raw_node_->create_publisher<ros_gz_interfaces::msg::ParamVec>(
    "/vrx/wavefield/parameters", rclcpp::QoS(10));
}

void VrxEnvPanel::onWindSpeedChanged(int value)
{
  target_values_.wind_speed = static_cast<double>(value) / kWindSpeedScale;
  updateLabels();
}

void VrxEnvPanel::onWindDirectionChanged(int value)
{
  target_values_.wind_direction_deg = static_cast<double>(value);
  updateLabels();
}

void VrxEnvPanel::onWaveAmplitudeChanged(int value)
{
  target_values_.wave_amplitude = static_cast<double>(value) / kWaveScale;
  updateLabels();
}

void VrxEnvPanel::onWavePeriodChanged(int value)
{
  target_values_.wave_period = static_cast<double>(value) / kWaveScale;
  updateLabels();
}

void VrxEnvPanel::onCurrentXChanged(int value)
{
  target_values_.current_x = static_cast<double>(value) / kCurrentScale;
  updateLabels();
}

void VrxEnvPanel::onCurrentYChanged(int value)
{
  target_values_.current_y = static_cast<double>(value) / kCurrentScale;
  updateLabels();
}

void VrxEnvPanel::onPhysicsCouplingToggled(bool checked)
{
  physics_coupling_enabled_ = checked;

  wave_amplitude_slider_->setEnabled(!checked);
  wave_period_slider_->setEnabled(!checked);
  current_x_slider_->setEnabled(!checked);
  current_y_slider_->setEnabled(!checked);

  if (checked) {
    const double wind_speed = target_values_.wind_speed;
    const double wind_direction_rad = target_values_.wind_direction_deg * kDegToRad;
    const double current_speed = 0.03 * wind_speed;

    target_values_.wave_amplitude = 0.11 * (wind_speed * wind_speed / kGravity);
    target_values_.wave_period = 0.81 * (2.0 * kPi * wind_speed / kGravity);
    target_values_.current_x = current_speed * std::cos(wind_direction_rad);
    target_values_.current_y = current_speed * std::sin(wind_direction_rad);
    updateLabels();
  }
}

void VrxEnvPanel::onEmergencyStopClicked()
{
  // Route through slider handlers to preserve ramp-limited transitions.
  wind_speed_slider_->setValue(0);
  wave_amplitude_slider_->setValue(0);
  current_x_slider_->setValue(0);
  current_y_slider_->setValue(0);
}

void VrxEnvPanel::onResetClicked()
{
  wind_speed_slider_->setValue(0);
  wind_direction_slider_->setValue(0);
  wave_amplitude_slider_->setValue(0);
  wave_period_slider_->setValue(static_cast<int>(5.0 * kWaveScale));
  current_x_slider_->setValue(0);
  current_y_slider_->setValue(0);
}

void VrxEnvPanel::updateLabels()
{
  wind_speed_label_->setText(QString::number(target_values_.wind_speed, 'f', 1) + " m/s");
  wind_direction_label_->setText(
    QString::number(target_values_.wind_direction_deg, 'f', 0) + " deg");
  wave_amplitude_label_->setText(QString::number(target_values_.wave_amplitude, 'f', 2) + " m");
  wave_period_label_->setText(QString::number(target_values_.wave_period, 'f', 2) + " s");
  current_x_label_->setText(QString::number(target_values_.current_x, 'f', 2) + " m/s");
  current_y_label_->setText(QString::number(target_values_.current_y, 'f', 2) + " m/s");
  updateCouplingPreviewLabels();
}

void VrxEnvPanel::updateCouplingPreviewLabels()
{
  const double target_wind_speed = target_values_.wind_speed;
  const double target_derived_amplitude = 0.11 * (target_wind_speed * target_wind_speed / kGravity);
  const double target_derived_period = 0.81 * (2.0 * kPi * target_wind_speed / kGravity);
  const double target_derived_current_speed = 0.03 * target_wind_speed;

  const double published_wind_speed = current_published_values_.wind_speed;
  const double published_derived_amplitude =
    0.11 * (published_wind_speed * published_wind_speed / kGravity);
  const double published_derived_period =
    0.81 * (2.0 * kPi * published_wind_speed / kGravity);
  const double published_derived_current_speed = 0.03 * published_wind_speed;

  coupling_amplitude_label_->setText(QString::number(target_derived_amplitude, 'f', 2) + " m");
  coupling_period_label_->setText(QString::number(target_derived_period, 'f', 2) + " s");
  coupling_current_speed_label_->setText(
    QString::number(target_derived_current_speed, 'f', 2) + " m/s");
  coupling_current_direction_label_->setText(
    QString::number(target_values_.wind_direction_deg, 'f', 0) + " deg");

  coupling_published_amplitude_label_->setText(
    QString::number(published_derived_amplitude, 'f', 2) + " m");
  coupling_published_period_label_->setText(
    QString::number(published_derived_period, 'f', 2) + " s");
  coupling_published_current_speed_label_->setText(
    QString::number(published_derived_current_speed, 'f', 2) + " m/s");
  coupling_published_current_direction_label_->setText(
    QString::number(current_published_values_.wind_direction_deg, 'f', 0) + " deg");
}

double VrxEnvPanel::clampStep(double current, double target, double max_step)
{
  const double delta = target - current;
  if (std::fabs(delta) <= max_step) {
    return target;
  }

  return current + std::copysign(max_step, delta);
}

void VrxEnvPanel::onRampTimer()
{
  if (physics_coupling_enabled_) {
    const double wind_speed = target_values_.wind_speed;
    const double wind_direction_rad = target_values_.wind_direction_deg * kDegToRad;
    const double current_speed = 0.03 * wind_speed;

    target_values_.wave_amplitude = 0.11 * (wind_speed * wind_speed / kGravity);
    target_values_.wave_period = 0.81 * (2.0 * kPi * wind_speed / kGravity);
    target_values_.current_x = current_speed * std::cos(wind_direction_rad);
    target_values_.current_y = current_speed * std::sin(wind_direction_rad);
    updateLabels();
  }

  current_published_values_.wind_speed = clampStep(
    current_published_values_.wind_speed,
    target_values_.wind_speed,
    kWindSpeedRatePerSec * kDtSec);
  current_published_values_.wind_direction_deg = clampStep(
    current_published_values_.wind_direction_deg,
    target_values_.wind_direction_deg,
    kWindDirectionRatePerSec * kDtSec);
  current_published_values_.wave_amplitude = clampStep(
    current_published_values_.wave_amplitude,
    target_values_.wave_amplitude,
    kWaveAmplitudeRatePerSec * kDtSec);
  current_published_values_.wave_period = clampStep(
    current_published_values_.wave_period,
    target_values_.wave_period,
    kWavePeriodRatePerSec * kDtSec);
  current_published_values_.current_x = clampStep(
    current_published_values_.current_x,
    target_values_.current_x,
    kCurrentRatePerSec * kDtSec);
  current_published_values_.current_y = clampStep(
    current_published_values_.current_y,
    target_values_.current_y,
    kCurrentRatePerSec * kDtSec);

  updateCouplingPreviewLabels();

  if (!raw_node_) {
    return;
  }

  if (wind_speed_pub_) {
    std_msgs::msg::Float64 msg;
    msg.data = current_published_values_.wind_speed;
    wind_speed_pub_->publish(msg);
  }

  if (wind_direction_pub_) {
    std_msgs::msg::Float64 msg;
    msg.data = current_published_values_.wind_direction_deg;
    wind_direction_pub_->publish(msg);
  }

  if (wave_amplitude_pub_) {
    std_msgs::msg::Float64 msg;
    msg.data = current_published_values_.wave_amplitude;
    wave_amplitude_pub_->publish(msg);
  }

  if (wave_period_pub_) {
    std_msgs::msg::Float64 msg;
    msg.data = current_published_values_.wave_period;
    wave_period_pub_->publish(msg);
  }

  if (current_pub_) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = current_published_values_.current_x;
    msg.linear.y = current_published_values_.current_y;
    current_pub_->publish(msg);
  }

  if (wavefield_param_pub_) {
    ros_gz_interfaces::msg::ParamVec params_msg;
    params_msg.header.stamp = raw_node_->now();

    rcl_interfaces::msg::Parameter direction_param;
    direction_param.name = "direction";
    direction_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    direction_param.value.double_value = current_published_values_.wind_direction_deg * kDegToRad;

    rcl_interfaces::msg::Parameter gain_param;
    gain_param.name = "gain";
    gain_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gain_param.value.double_value = std::clamp(
      current_published_values_.wave_amplitude * kWaveGainFromAmplitudeScale,
      0.0,
      1.0);

    rcl_interfaces::msg::Parameter period_param;
    period_param.name = "period";
    period_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    period_param.value.double_value = current_published_values_.wave_period;

    rcl_interfaces::msg::Parameter steepness_param;
    steepness_param.name = "steepness";
    steepness_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    steepness_param.value.double_value = 0.0;

    params_msg.params = {direction_param, gain_param, period_param, steepness_param};
    wavefield_param_pub_->publish(params_msg);
  }
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::VrxEnvPanel, rviz_common::Panel)
