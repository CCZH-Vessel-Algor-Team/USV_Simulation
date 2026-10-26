#include "env_panel/vrx_env_panel.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QSignalBlocker>
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

constexpr double kWindSpeedRatePerSec = 0.5;
constexpr double kWindDirectionRatePerSec = 30.0;
constexpr double kWaveAmplitudeRatePerSec = 0.03;
constexpr double kWavePeriodRatePerSec = 0.1;
constexpr double kCurrentRatePerSec = 0.1;

constexpr double kWindSpeedScale = 10.0;
constexpr double kWaveScale = 100.0;
constexpr double kCurrentScale = 100.0;
constexpr double kSunBrightnessScale = 100.0;

constexpr double kWaveGainFromAmplitudeScale = 1.0 / 3.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kMinWavePeriodSec = 1.0;
constexpr double kMaxWavePeriodSec = 20.0;
constexpr double kMaxWaveGain = 1.0;
constexpr double kMaxCurrentSpeed = 2.0;
constexpr double kHsPerGainPeriodSquared = 0.024865;

constexpr double kGravity = 9.81;
constexpr double kPi = 3.14159;
}  // namespace

VrxEnvPanel::VrxEnvPanel(QWidget * parent)
: rviz_common::Panel(parent),
  wind_speed_slider_(nullptr),
  wind_direction_slider_(nullptr),
  wave_amplitude_slider_(nullptr),
  wave_period_slider_(nullptr),
  wave_direction_slider_(nullptr),
  current_x_slider_(nullptr),
  current_y_slider_(nullptr),
  sun_brightness_slider_(nullptr),
  wind_speed_label_(nullptr),
  wind_direction_label_(nullptr),
  wave_amplitude_label_(nullptr),
  wave_period_label_(nullptr),
  wave_direction_label_(nullptr),
  current_x_label_(nullptr),
  current_y_label_(nullptr),
  sun_brightness_label_(nullptr),
  coupling_amplitude_label_(nullptr),
  coupling_period_label_(nullptr),
  coupling_current_speed_label_(nullptr),
  coupling_current_direction_label_(nullptr),
  coupling_published_amplitude_label_(nullptr),
  coupling_published_period_label_(nullptr),
  coupling_published_current_speed_label_(nullptr),
  coupling_published_current_direction_label_(nullptr),
  physics_coupling_checkbox_(nullptr),
  coupling_preview_group_(nullptr),
  mode_status_label_(nullptr),
  emergency_stop_button_(nullptr),
  reset_button_(nullptr),
  ramp_timer_(nullptr),
  physics_coupling_enabled_(false),
  target_sun_brightness_(1.0),
  sun_light_dirty_(false),
  sun_light_published_(false),
  sun_light_debounce_ticks_(0),
  sun_light_publish_ticks_(0),
  target_values_{0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0},
  current_published_values_{0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0},
  last_wave_published_values_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  wave_published_(false),
  wave_publish_ticks_(0)
{
  setupUi();

  ramp_timer_ = new QTimer(this);
  connect(ramp_timer_, &QTimer::timeout, this, &VrxEnvPanel::onRampTimer);
}

void VrxEnvPanel::onInitialize()
{
  auto * display_context = getDisplayContext();
  if (!display_context) {
    return;
  }

  auto ros_node_abstraction = display_context->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    return;
  }

  raw_node_ = ros_node_abstraction->get_raw_node();
  setupPublishers();
  ramp_timer_->start(static_cast<int>(1000.0 / kRampHz));
}

void VrxEnvPanel::setupUi()
{
  auto * content_widget = new QWidget(this);
  auto * main_layout = new QVBoxLayout();
  main_layout->setContentsMargins(12, 10, 12, 12);
  main_layout->setSpacing(10);
  setStyleSheet(
    "QGroupBox { font-weight: 600; margin-top: 10px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
    "QPushButton { min-height: 28px; padding: 3px 10px; }");

  // Keep the first row below the group-box frame even with large fonts or HiDPI scaling.
  const int group_content_top_margin = fontMetrics().height() + 8;
  const auto configure_form_layout = [group_content_top_margin](QFormLayout * layout) {
    layout->setContentsMargins(12, group_content_top_margin, 12, 10);
    layout->setHorizontalSpacing(14);
    layout->setVerticalSpacing(8);
    layout->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    layout->setFormAlignment(Qt::AlignTop);
    layout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  };

  const auto configure_slider_row = [](QHBoxLayout * layout) {
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(12);
  };

  auto * mode_row = new QHBoxLayout();
  mode_row->setContentsMargins(2, 0, 2, 0);
  mode_row->setSpacing(10);
  physics_coupling_checkbox_ =
    new QCheckBox("风与波增益/流耦合", this);
  mode_status_label_ = new QLabel("独立", this);
  mode_status_label_->setMinimumWidth(72);
  mode_row->addWidget(physics_coupling_checkbox_);
  mode_row->addStretch();
  mode_row->addWidget(mode_status_label_);
  main_layout->addLayout(mode_row);
  physics_coupling_checkbox_->setToolTip(
    "根据实际风况推导波增益、波向和流场；波周期保持独立设置。");

  coupling_preview_group_ = new QGroupBox("耦合数值", this);
  auto * coupling_preview_layout = new QGridLayout();
  coupling_preview_layout->setContentsMargins(12, group_content_top_margin, 12, 10);
  coupling_preview_layout->setHorizontalSpacing(24);
  coupling_preview_layout->setVerticalSpacing(8);
  coupling_preview_layout->setColumnStretch(0, 1);
  coupling_preview_layout->setColumnStretch(1, 1);
  coupling_preview_layout->setColumnStretch(2, 1);
  coupling_amplitude_label_ = new QLabel("0.00", coupling_preview_group_);
  coupling_period_label_ = new QLabel("0.00 s", coupling_preview_group_);
  coupling_current_speed_label_ = new QLabel("0.00 m/s", coupling_preview_group_);
  coupling_current_direction_label_ = new QLabel("0 deg", coupling_preview_group_);

  coupling_published_amplitude_label_ = new QLabel("0.00", coupling_preview_group_);
  coupling_published_period_label_ = new QLabel("0.00 s", coupling_preview_group_);
  coupling_published_current_speed_label_ = new QLabel("0.00 m/s", coupling_preview_group_);
  coupling_published_current_direction_label_ = new QLabel("0 deg", coupling_preview_group_);

  coupling_preview_layout->addWidget(
    new QLabel("指标", coupling_preview_group_), 0, 0, Qt::AlignLeft | Qt::AlignVCenter);
  coupling_preview_layout->addWidget(
    new QLabel("目标", coupling_preview_group_), 0, 1, Qt::AlignCenter);
  coupling_preview_layout->addWidget(
    new QLabel("实际", coupling_preview_group_), 0, 2, Qt::AlignCenter);

  coupling_preview_layout->addWidget(new QLabel("波增益", coupling_preview_group_), 1, 0);
  coupling_preview_layout->addWidget(coupling_amplitude_label_, 1, 1, Qt::AlignCenter);
  coupling_preview_layout->addWidget(coupling_published_amplitude_label_, 1, 2, Qt::AlignCenter);

  coupling_preview_layout->addWidget(new QLabel("波周期", coupling_preview_group_), 2, 0);
  coupling_preview_layout->addWidget(coupling_period_label_, 2, 1, Qt::AlignCenter);
  coupling_preview_layout->addWidget(coupling_published_period_label_, 2, 2, Qt::AlignCenter);

  coupling_preview_layout->addWidget(new QLabel("流速", coupling_preview_group_), 3, 0);
  coupling_preview_layout->addWidget(coupling_current_speed_label_, 3, 1, Qt::AlignCenter);
  coupling_preview_layout->addWidget(
    coupling_published_current_speed_label_, 3, 2, Qt::AlignCenter);

  coupling_preview_layout->addWidget(new QLabel("流向", coupling_preview_group_), 4, 0);
  coupling_preview_layout->addWidget(coupling_current_direction_label_, 4, 1, Qt::AlignCenter);
  coupling_preview_layout->addWidget(
    coupling_published_current_direction_label_, 4, 2, Qt::AlignCenter);

  coupling_preview_group_->setLayout(coupling_preview_layout);
  coupling_preview_group_->setVisible(false);
  main_layout->addWidget(coupling_preview_group_);

  auto * wind_group = new QGroupBox("风", this);
  auto * wind_layout = new QFormLayout();
  configure_form_layout(wind_layout);

  wind_speed_slider_ = new QSlider(Qt::Horizontal, wind_group);
  wind_speed_slider_->setRange(0, static_cast<int>(20.0 * kWindSpeedScale));
  wind_speed_slider_->setValue(0);
  wind_speed_slider_->setToolTip("世界坐标系下的风速；变化速率受限。");
  wind_speed_label_ = new QLabel("0.0 m/s", wind_group);

  auto * wind_speed_row = new QHBoxLayout();
  configure_slider_row(wind_speed_row);
  wind_speed_row->addWidget(wind_speed_slider_, 1);
  wind_speed_row->addWidget(wind_speed_label_);
  wind_layout->addRow("风速", wind_speed_row);

  wind_direction_slider_ = new QSlider(Qt::Horizontal, wind_group);
  wind_direction_slider_->setRange(-180, 180);
  wind_direction_slider_->setValue(0);
  wind_direction_slider_->setToolTip("ENU 坐标系中风吹向的方向。");
  wind_direction_label_ = new QLabel("0 deg", wind_group);

  auto * wind_dir_row = new QHBoxLayout();
  configure_slider_row(wind_dir_row);
  wind_dir_row->addWidget(wind_direction_slider_, 1);
  wind_dir_row->addWidget(wind_direction_label_);
  wind_layout->addRow("风向", wind_dir_row);

  wind_group->setLayout(wind_layout);
  main_layout->addWidget(wind_group);

  auto * wave_group = new QGroupBox("浪", this);
  auto * wave_layout = new QFormLayout();
  configure_form_layout(wave_layout);

  wave_amplitude_slider_ = new QSlider(Qt::Horizontal, wave_group);
  wave_amplitude_slider_->setRange(0, static_cast<int>(kMaxWaveGain * kWaveScale));
  wave_amplitude_slider_->setValue(static_cast<int>(0.3 * kWaveScale));
  wave_amplitude_slider_->setToolTip("PMS 波谱增益，并非以米为单位的波高。");
  wave_amplitude_label_ = new QLabel("0.30", wave_group);

  auto * wave_amp_row = new QHBoxLayout();
  configure_slider_row(wave_amp_row);
  wave_amp_row->addWidget(wave_amplitude_slider_, 1);
  wave_amp_row->addWidget(wave_amplitude_label_);
  wave_layout->addRow("增益", wave_amp_row);

  wave_period_slider_ = new QSlider(Qt::Horizontal, wave_group);
  wave_period_slider_->setRange(static_cast<int>(1.0 * kWaveScale), static_cast<int>(20.0 * kWaveScale));
  wave_period_slider_->setValue(static_cast<int>(5.0 * kWaveScale));
  wave_period_slider_->setToolTip(
    "周期越短，船体范围内的空间变化越明显。");
  wave_period_label_ = new QLabel("5.00 s", wave_group);

  auto * wave_period_row = new QHBoxLayout();
  configure_slider_row(wave_period_row);
  wave_period_row->addWidget(wave_period_slider_, 1);
  wave_period_row->addWidget(wave_period_label_);
  wave_layout->addRow("周期", wave_period_row);

  wave_direction_slider_ = new QSlider(Qt::Horizontal, wave_group);
  wave_direction_slider_->setRange(-180, 180);
  wave_direction_slider_->setValue(0);
  wave_direction_slider_->setToolTip(
    "波浪传播方向。船艏指向 +X 时，设为 90 deg 可模拟横浪横摇。");
  wave_direction_label_ = new QLabel("0 deg", wave_group);
  auto * wave_direction_row = new QHBoxLayout();
  configure_slider_row(wave_direction_row);
  wave_direction_row->addWidget(wave_direction_slider_, 1);
  wave_direction_row->addWidget(wave_direction_label_);
  wave_layout->addRow("波向", wave_direction_row);

  wave_group->setLayout(wave_layout);
  main_layout->addWidget(wave_group);

  auto * current_group = new QGroupBox("流", this);
  auto * current_layout = new QFormLayout();
  configure_form_layout(current_layout);

  current_x_slider_ = new QSlider(Qt::Horizontal, current_group);
  current_x_slider_->setRange(
    static_cast<int>(-kMaxCurrentSpeed * kCurrentScale),
    static_cast<int>(kMaxCurrentSpeed * kCurrentScale));
  current_x_slider_->setValue(0);
  current_x_label_ = new QLabel("0.00 m/s", current_group);

  auto * current_x_row = new QHBoxLayout();
  configure_slider_row(current_x_row);
  current_x_row->addWidget(current_x_slider_, 1);
  current_x_row->addWidget(current_x_label_);
  current_layout->addRow("X 分量", current_x_row);

  current_y_slider_ = new QSlider(Qt::Horizontal, current_group);
  current_y_slider_->setRange(
    static_cast<int>(-kMaxCurrentSpeed * kCurrentScale),
    static_cast<int>(kMaxCurrentSpeed * kCurrentScale));
  current_y_slider_->setValue(0);
  current_y_label_ = new QLabel("0.00 m/s", current_group);

  auto * current_y_row = new QHBoxLayout();
  configure_slider_row(current_y_row);
  current_y_row->addWidget(current_y_slider_, 1);
  current_y_row->addWidget(current_y_label_);
  current_layout->addRow("Y 分量", current_y_row);

  current_group->setLayout(current_layout);
  main_layout->addWidget(current_group);

  auto * lighting_group = new QGroupBox("光照", this);
  auto * lighting_layout = new QFormLayout();
  configure_form_layout(lighting_layout);

  sun_brightness_slider_ = new QSlider(Qt::Horizontal, lighting_group);
  sun_brightness_slider_->setRange(0, static_cast<int>(kSunBrightnessScale));
  sun_brightness_slider_->setValue(static_cast<int>(kSunBrightnessScale));
  sun_brightness_slider_->setToolTip(
    "调整 Gazebo 太阳方向光的强度，不改变场景环境光。");
  sun_brightness_label_ = new QLabel("100%", lighting_group);

  auto * sun_brightness_row = new QHBoxLayout();
  configure_slider_row(sun_brightness_row);
  sun_brightness_row->addWidget(sun_brightness_slider_, 1);
  sun_brightness_row->addWidget(sun_brightness_label_);
  lighting_layout->addRow("太阳亮度", sun_brightness_row);

  lighting_group->setLayout(lighting_layout);
  main_layout->addWidget(lighting_group);

  auto * control_row = new QHBoxLayout();
  control_row->setContentsMargins(0, 2, 0, 0);
  control_row->setSpacing(10);
  emergency_stop_button_ = new QPushButton("静水", this);
  emergency_stop_button_->setToolTip("将风、浪和水流逐渐降至零。");
  reset_button_ = new QPushButton("恢复默认", this);
  reset_button_->setToolTip("恢复默认的耦合环境状态。");
  control_row->addWidget(emergency_stop_button_);
  control_row->addWidget(reset_button_);
  main_layout->addLayout(control_row);

  main_layout->addStretch();
  content_widget->setLayout(main_layout);
  auto * scroll_area = new QScrollArea(this);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  scroll_area->setWidget(content_widget);
  auto * panel_layout = new QVBoxLayout();
  panel_layout->setContentsMargins(0, 0, 0, 0);
  panel_layout->addWidget(scroll_area);
  setLayout(panel_layout);

  connect(wind_speed_slider_, &QSlider::valueChanged, this, &VrxEnvPanel::onWindSpeedChanged);
  connect(
    wind_direction_slider_, &QSlider::valueChanged, this, &VrxEnvPanel::onWindDirectionChanged);
  connect(
    wave_amplitude_slider_, &QSlider::valueChanged, this, &VrxEnvPanel::onWaveAmplitudeChanged);
  connect(wave_period_slider_, &QSlider::valueChanged, this, &VrxEnvPanel::onWavePeriodChanged);
  connect(
    wave_direction_slider_, &QSlider::valueChanged,
    this, &VrxEnvPanel::onWaveDirectionChanged);
  connect(current_x_slider_, &QSlider::valueChanged, this, &VrxEnvPanel::onCurrentXChanged);
  connect(current_y_slider_, &QSlider::valueChanged, this, &VrxEnvPanel::onCurrentYChanged);
  connect(
    sun_brightness_slider_, &QSlider::valueChanged,
    this, &VrxEnvPanel::onSunBrightnessChanged);
  connect(
    physics_coupling_checkbox_,
    &QCheckBox::toggled,
    this,
    &VrxEnvPanel::onPhysicsCouplingToggled);
  connect(emergency_stop_button_, &QPushButton::clicked, this, &VrxEnvPanel::onEmergencyStopClicked);
  connect(reset_button_, &QPushButton::clicked, this, &VrxEnvPanel::onResetClicked);

  for (auto * label : {
      wind_speed_label_, wind_direction_label_, wave_amplitude_label_,
      wave_period_label_, wave_direction_label_, current_x_label_, current_y_label_,
      sun_brightness_label_})
  {
    label->setMinimumWidth(76);
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  }

  updateLabels();
  physics_coupling_checkbox_->setChecked(true);
}

void VrxEnvPanel::setupPublishers()
{
  if (!raw_node_) {
    return;
  }

  wind_velocity_pub_ = raw_node_->create_publisher<geometry_msgs::msg::Vector3>(
    "/vrx/wind/velocity_cmd", rclcpp::QoS(10));
  current_velocity_pub_ = raw_node_->create_publisher<geometry_msgs::msg::Vector3>(
    "/ocean_current", rclcpp::QoS(10));
  wavefield_param_pub_ = raw_node_->create_publisher<ros_gz_interfaces::msg::ParamVec>(
    "/vrx/wavefield/parameters", rclcpp::QoS(10));
  sun_light_pub_ = raw_node_->create_publisher<ros_gz_interfaces::msg::Light>(
    "/vrx/environment/sun_light_cmd", rclcpp::QoS(10));
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
  target_values_.wave_height_m = kHsPerGainPeriodSquared *
    target_values_.wave_amplitude * target_values_.wave_period *
    target_values_.wave_period;
  updateLabels();
}

void VrxEnvPanel::onWavePeriodChanged(int value)
{
  target_values_.wave_period = static_cast<double>(value) / kWaveScale;
  updateLabels();
}

void VrxEnvPanel::onWaveDirectionChanged(int value)
{
  target_values_.wave_direction_deg = static_cast<double>(value);
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

void VrxEnvPanel::onSunBrightnessChanged(int value)
{
  target_sun_brightness_ = static_cast<double>(value) / kSunBrightnessScale;
  sun_light_dirty_ = true;
  sun_light_debounce_ticks_ = 0;
  updateLabels();
}

void VrxEnvPanel::onPhysicsCouplingToggled(bool checked)
{
  physics_coupling_enabled_ = checked;

  wave_amplitude_slider_->setEnabled(!checked);
  wave_period_slider_->setEnabled(true);
  wave_direction_slider_->setEnabled(!checked);
  current_x_slider_->setEnabled(!checked);
  current_y_slider_->setEnabled(!checked);
  coupling_preview_group_->setVisible(checked);
  mode_status_label_->setText(checked ? "耦合" : "独立");

  if (checked) {
    const double wind_speed = target_values_.wind_speed;
    const double wind_direction_rad = target_values_.wind_direction_deg * kDegToRad;
    const double current_speed = 0.03 * wind_speed;

    target_values_.wave_amplitude = std::clamp(
      0.11 * (wind_speed * wind_speed / kGravity) * kWaveGainFromAmplitudeScale,
      0.0,
      kMaxWaveGain);
    {
      QSignalBlocker blocker(wave_amplitude_slider_);
      wave_amplitude_slider_->setValue(
        static_cast<int>(target_values_.wave_amplitude * kWaveScale));
    }
    target_values_.wave_height_m = kHsPerGainPeriodSquared *
      target_values_.wave_amplitude * target_values_.wave_period *
      target_values_.wave_period;
    target_values_.wave_direction_deg = target_values_.wind_direction_deg;
    wave_direction_slider_->setValue(
      static_cast<int>(target_values_.wave_direction_deg));
    target_values_.current_x = current_speed * std::cos(wind_direction_rad);
    target_values_.current_y = current_speed * std::sin(wind_direction_rad);
    {
      QSignalBlocker x_blocker(current_x_slider_);
      QSignalBlocker y_blocker(current_y_slider_);
      current_x_slider_->setValue(static_cast<int>(target_values_.current_x * kCurrentScale));
      current_y_slider_->setValue(static_cast<int>(target_values_.current_y * kCurrentScale));
    }
    updateLabels();
  }
}

void VrxEnvPanel::onEmergencyStopClicked()
{
  physics_coupling_checkbox_->setChecked(false);
  wind_speed_slider_->setValue(0);
  wave_amplitude_slider_->setValue(0);
  current_x_slider_->setValue(0);
  current_y_slider_->setValue(0);
}

void VrxEnvPanel::onResetClicked()
{
  physics_coupling_checkbox_->setChecked(false);
  wind_speed_slider_->setValue(0);
  wind_direction_slider_->setValue(0);
  wave_amplitude_slider_->setValue(static_cast<int>(0.3 * kWaveScale));
  wave_period_slider_->setValue(static_cast<int>(5.0 * kWaveScale));
  wave_direction_slider_->setValue(0);
  current_x_slider_->setValue(0);
  current_y_slider_->setValue(0);
  sun_brightness_slider_->setValue(static_cast<int>(kSunBrightnessScale));
  target_values_.wave_height_m = kHsPerGainPeriodSquared *
    target_values_.wave_amplitude * target_values_.wave_period *
    target_values_.wave_period;
  physics_coupling_checkbox_->setChecked(true);
}

void VrxEnvPanel::updateLabels()
{
  wind_speed_label_->setText(QString::number(target_values_.wind_speed, 'f', 1) + " m/s");
  wind_direction_label_->setText(
    QString::number(target_values_.wind_direction_deg, 'f', 0) + " deg");
  wave_amplitude_label_->setText(QString::number(target_values_.wave_amplitude, 'f', 2));
  wave_period_label_->setText(QString::number(target_values_.wave_period, 'f', 2) + " s");
  wave_direction_label_->setText(
    QString::number(target_values_.wave_direction_deg, 'f', 0) + " deg");
  current_x_label_->setText(QString::number(target_values_.current_x, 'f', 2) + " m/s");
  current_y_label_->setText(QString::number(target_values_.current_y, 'f', 2) + " m/s");
  sun_brightness_label_->setText(
    QString::number(target_sun_brightness_ * 100.0, 'f', 0) + "%");
  updateCouplingPreviewLabels();
}

void VrxEnvPanel::updateCouplingPreviewLabels()
{
  const double target_wind_speed = target_values_.wind_speed;
  const double target_derived_amplitude = std::clamp(
    0.11 * (target_wind_speed * target_wind_speed / kGravity) *
    kWaveGainFromAmplitudeScale,
    0.0,
    kMaxWaveGain);
  const double target_derived_period = target_values_.wave_period;
  const double target_derived_current_speed = 0.03 * target_wind_speed;

  const double published_wind_speed = current_published_values_.wind_speed;
  const double published_derived_amplitude = std::clamp(
    0.11 * (published_wind_speed * published_wind_speed / kGravity) *
    kWaveGainFromAmplitudeScale,
    0.0,
    kMaxWaveGain);
  const double published_derived_period = current_published_values_.wave_period;
  const double published_derived_current_speed = 0.03 * published_wind_speed;

  coupling_amplitude_label_->setText(QString::number(target_derived_amplitude, 'f', 2));
  coupling_period_label_->setText(QString::number(target_derived_period, 'f', 2) + " s");
  coupling_current_speed_label_->setText(
    QString::number(target_derived_current_speed, 'f', 2) + " m/s");
  coupling_current_direction_label_->setText(
    QString::number(target_values_.wind_direction_deg, 'f', 0) + " deg");

  coupling_published_amplitude_label_->setText(
    QString::number(published_derived_amplitude, 'f', 2));
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

double VrxEnvPanel::clampAngleStep(double current, double target, double max_step)
{
  const double delta = std::atan2(
    std::sin((target - current) * kDegToRad),
    std::cos((target - current) * kDegToRad)) * kRadToDeg;
  if (std::fabs(delta) <= max_step) {
    return target;
  }
  return current + std::copysign(max_step, delta);
}

void VrxEnvPanel::onRampTimer()
{
  if (physics_coupling_enabled_) {
    const double wind_speed = current_published_values_.wind_speed;
    const double wind_direction_rad =
      current_published_values_.wind_direction_deg * kDegToRad;
    const double current_speed = 0.03 * wind_speed;

    target_values_.wave_amplitude = std::clamp(
      0.11 * (wind_speed * wind_speed / kGravity) * kWaveGainFromAmplitudeScale,
      0.0,
      kMaxWaveGain);
    target_values_.wave_height_m = kHsPerGainPeriodSquared *
      target_values_.wave_amplitude * target_values_.wave_period *
      target_values_.wave_period;
    target_values_.wave_direction_deg =
      current_published_values_.wind_direction_deg;
    {
      QSignalBlocker blocker(wave_direction_slider_);
      wave_direction_slider_->setValue(
        static_cast<int>(target_values_.wave_direction_deg));
    }
    target_values_.current_x = current_speed * std::cos(wind_direction_rad);
    target_values_.current_y = current_speed * std::sin(wind_direction_rad);
    updateLabels();
  }

  current_published_values_.wind_speed = clampStep(
    current_published_values_.wind_speed,
    target_values_.wind_speed,
    kWindSpeedRatePerSec * kDtSec);
  current_published_values_.wind_direction_deg = clampAngleStep(
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
  current_published_values_.wave_direction_deg = clampAngleStep(
    current_published_values_.wave_direction_deg,
    target_values_.wave_direction_deg,
    kWindDirectionRatePerSec * kDtSec);
  current_published_values_.wave_height_m = clampStep(
    current_published_values_.wave_height_m,
    target_values_.wave_height_m,
    0.03 * kDtSec);
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

  const double wind_direction_rad =
    current_published_values_.wind_direction_deg * kDegToRad;
  const double wave_direction_rad =
    current_published_values_.wave_direction_deg * kDegToRad;

  if (!std::isfinite(wind_direction_rad) ||
    !std::isfinite(current_published_values_.wind_speed) ||
    !std::isfinite(current_published_values_.current_x) ||
    !std::isfinite(current_published_values_.current_y) ||
    !std::isfinite(current_published_values_.wave_amplitude) ||
    !std::isfinite(current_published_values_.wave_period) ||
    !std::isfinite(current_published_values_.wave_direction_deg) ||
    !std::isfinite(current_published_values_.wave_height_m) ||
    !std::isfinite(target_sun_brightness_))
  {
    return;
  }

  current_published_values_.wave_amplitude = std::clamp(
    current_published_values_.wave_amplitude, 0.0, kMaxWaveGain);
  current_published_values_.wave_period = std::clamp(
    current_published_values_.wave_period, kMinWavePeriodSec, kMaxWavePeriodSec);
  current_published_values_.wave_height_m = std::clamp(
    current_published_values_.wave_height_m, 0.0, 10.0);
  current_published_values_.current_x = std::clamp(
    current_published_values_.current_x, -kMaxCurrentSpeed, kMaxCurrentSpeed);
  current_published_values_.current_y = std::clamp(
    current_published_values_.current_y, -kMaxCurrentSpeed, kMaxCurrentSpeed);
  target_sun_brightness_ = std::clamp(target_sun_brightness_, 0.0, 1.0);

  if (wind_velocity_pub_) {
    geometry_msgs::msg::Vector3 msg;
    msg.x = current_published_values_.wind_speed * std::cos(wind_direction_rad);
    msg.y = current_published_values_.wind_speed * std::sin(wind_direction_rad);
    wind_velocity_pub_->publish(msg);
  }

  if (current_velocity_pub_) {
    geometry_msgs::msg::Vector3 msg;
    msg.x = current_published_values_.current_x;
    msg.y = current_published_values_.current_y;
    current_velocity_pub_->publish(msg);
  }

  ++wave_publish_ticks_;
  const bool wave_changed = !wave_published_ || wave_publish_ticks_ >= 10 ||
    std::fabs(current_published_values_.wave_amplitude -
    last_wave_published_values_.wave_amplitude) > 1e-6 ||
    std::fabs(current_published_values_.wave_period -
    last_wave_published_values_.wave_period) > 1e-6 ||
    std::fabs(current_published_values_.wave_height_m -
    last_wave_published_values_.wave_height_m) > 1e-6 ||
    std::fabs(current_published_values_.wave_direction_deg -
    last_wave_published_values_.wave_direction_deg) > 1e-6;

  if (wavefield_param_pub_ && wave_changed) {
    ros_gz_interfaces::msg::ParamVec msg;
    msg.header.stamp = raw_node_->now();

    rcl_interfaces::msg::Parameter direction;
    direction.name = "direction";
    direction.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    direction.value.double_value = wave_direction_rad;

    rcl_interfaces::msg::Parameter gain;
    gain.name = "gain";
    gain.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gain.value.double_value = current_published_values_.wave_amplitude;

    rcl_interfaces::msg::Parameter significant_height;
    significant_height.name = "significant_height";
    significant_height.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    significant_height.value.double_value = current_published_values_.wave_height_m;

    rcl_interfaces::msg::Parameter period;
    period.name = "period";
    period.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    period.value.double_value = current_published_values_.wave_period;

    rcl_interfaces::msg::Parameter steepness;
    steepness.name = "steepness";
    steepness.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    steepness.value.double_value = 0.0;

    msg.params = {direction, gain, significant_height, period, steepness};
    wavefield_param_pub_->publish(msg);
    last_wave_published_values_ = current_published_values_;
    wave_published_ = true;
    wave_publish_ticks_ = 0;
  }

  ++sun_light_publish_ticks_;
  if (sun_light_dirty_) {
    ++sun_light_debounce_ticks_;
  }

  const bool sun_light_due = sun_light_dirty_ ?
    sun_light_debounce_ticks_ >= 2 :
    (sun_light_published_ && sun_light_publish_ticks_ >= 10);
  if (sun_light_pub_ && sun_light_due) {
    ros_gz_interfaces::msg::Light msg;
    msg.header.stamp = raw_node_->now();
    msg.name = "sun";
    msg.type = ros_gz_interfaces::msg::Light::DIRECTIONAL;
    msg.pose.position.z = 10.0;
    msg.pose.orientation.w = 1.0;
    msg.diffuse.r = 0.8;
    msg.diffuse.g = 0.8;
    msg.diffuse.b = 0.8;
    msg.diffuse.a = 1.0;
    msg.specular.r = 0.2;
    msg.specular.g = 0.2;
    msg.specular.b = 0.2;
    msg.specular.a = 1.0;
    msg.attenuation_constant = 0.9;
    msg.attenuation_linear = 0.01;
    msg.attenuation_quadratic = 0.001;
    msg.direction.x = -0.5;
    msg.direction.y = 0.1;
    msg.direction.z = -0.9;
    msg.range = 1000.0;
    msg.cast_shadows = true;
    msg.intensity = target_sun_brightness_;
    sun_light_pub_->publish(msg);
    sun_light_dirty_ = false;
    sun_light_published_ = true;
    sun_light_debounce_ticks_ = 0;
    sun_light_publish_ticks_ = 0;
  }
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::VrxEnvPanel, rviz_common::Panel)
