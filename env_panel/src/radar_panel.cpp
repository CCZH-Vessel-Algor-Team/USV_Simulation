#include "env_panel/radar_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <utility>

#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <QVariant>

#include <pluginlib/class_list_macros.hpp>

namespace env_panel
{

RadarPanel::RadarPanel(QWidget * parent)
: rviz_common::Panel(parent),
  range_combo_(nullptr),
  rpm_combo_(nullptr),
  gain_auto_checkbox_(nullptr),
  gain_slider_(nullptr),
  gain_value_label_(nullptr),
  sea_auto_checkbox_(nullptr),
  sea_slider_(nullptr),
  sea_value_label_(nullptr),
  side_auto_checkbox_(nullptr),
  side_slider_(nullptr),
  side_value_label_(nullptr),
  rain_slider_(nullptr),
  rain_value_label_(nullptr),
  beam_spinbox_(nullptr),
  interference_spinbox_(nullptr),
  noise_spinbox_(nullptr),
  boost_spinbox_(nullptr),
  stretch_checkbox_(nullptr),
  ins_enable_checkbox_(nullptr),
  ins_ip_edit_(nullptr),
  ins_port_spinbox_(nullptr),
  connection_label_(nullptr),
  radar_state_label_(nullptr),
  connection_timer_(nullptr),
  connected_(false)
{
  setupUi();
}

RadarPanel::~RadarPanel()
{
  if (connection_timer_) {
    connection_timer_->stop();
  }
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void RadarPanel::onInitialize()
{
  setupRos();
}

void RadarPanel::setupUi()
{
  auto * content_widget = new QWidget(this);
  auto * main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(8, 8, 8, 8);
  main_layout->setSpacing(6);

  // Keep the native QGroupBox title geometry. A fixed pixel top margin makes
  // titles overlap the frame when RViz runs with a large font or HiDPI scale.
  setStyleSheet("QPushButton { min-height: 28px; padding: 2px 8px; }");

  auto * power_group = new QGroupBox("电源/发射控制", content_widget);
  auto * power_layout = new QGridLayout;
  power_layout->setHorizontalSpacing(8);
  power_layout->setVerticalSpacing(6);
  auto * power_on_button = new QPushButton("开机", power_group);
  auto * power_off_button = new QPushButton("关机", power_group);
  auto * transmit_on_button = new QPushButton("发射", power_group);
  auto * transmit_off_button = new QPushButton("待机", power_group);
  power_on_button->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;");
  power_off_button->setStyleSheet("background-color: #f44336; color: white; font-weight: bold;");
  transmit_on_button->setStyleSheet(
    "background-color: #2196F3; color: white; font-weight: bold;");
  transmit_off_button->setStyleSheet(
    "background-color: #FF9800; color: white; font-weight: bold;");
  power_layout->addWidget(new QLabel("电源", power_group), 0, 0);
  power_layout->addWidget(power_on_button, 0, 1);
  power_layout->addWidget(power_off_button, 0, 2);
  power_layout->addWidget(new QLabel("发射", power_group), 1, 0);
  power_layout->addWidget(transmit_on_button, 1, 1);
  power_layout->addWidget(transmit_off_button, 1, 2);
  power_layout->setColumnStretch(1, 1);
  power_layout->setColumnStretch(2, 1);
  power_group->setLayout(power_layout);
  main_layout->addWidget(power_group);

  auto * basic_group = new QGroupBox("基本参数", content_widget);
  auto * basic_layout = new QFormLayout;
  basic_layout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  basic_layout->setRowWrapPolicy(QFormLayout::WrapLongRows);
  range_combo_ = new QComboBox(basic_group);
  const std::vector<int> ranges{
    250, 500, 750, 1000, 1500, 2000, 3000, 4000, 6000,
    8000, 12000, 16000, 24000, 36000, 48000, 64000, 80000};
  for (const int range : ranges) {
    const QString label = range >= 1000 ?
      QString("%1 km").arg(static_cast<double>(range) / 1000.0, 0, 'f', 1) :
      QString("%1 m").arg(range);
    range_combo_->addItem(label, range);
  }
  range_combo_->setCurrentIndex(range_combo_->findData(1000));
  range_combo_->setMinimumWidth(150);

  rpm_combo_ = new QComboBox(basic_group);
  rpm_combo_->addItems({"24 rpm", "36 rpm", "48 rpm", "60 rpm", "96 rpm"});
  rpm_combo_->setCurrentIndex(2);
  basic_layout->addRow("量程：", range_combo_);
  basic_layout->addRow("转速：", rpm_combo_);
  basic_group->setLayout(basic_layout);
  main_layout->addWidget(basic_group);

  auto * echo_group = new QGroupBox("回波调节", content_widget);
  auto * echo_layout = new QGridLayout;
  echo_layout->setHorizontalSpacing(8);
  echo_layout->setVerticalSpacing(6);
  echo_layout->setColumnStretch(2, 1);

  auto add_auto_slider_row =
    [echo_group, echo_layout](
    int row, const QString & title, QCheckBox * & automatic,
    QSlider * & slider, QLabel * & value_label)
    {
      automatic = new QCheckBox("自动", echo_group);
      automatic->setChecked(true);
      slider = new QSlider(Qt::Horizontal, echo_group);
      slider->setRange(0, 255);
      slider->setEnabled(false);
      value_label = new QLabel("Auto", echo_group);
      value_label->setMinimumWidth(48);
      value_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      echo_layout->addWidget(new QLabel(title, echo_group), row, 0);
      echo_layout->addWidget(automatic, row, 1);
      echo_layout->addWidget(slider, row, 2);
      echo_layout->addWidget(value_label, row, 3);
    };

  add_auto_slider_row(
    0, "增益", gain_auto_checkbox_, gain_slider_, gain_value_label_);
  add_auto_slider_row(
    1, "海杂波", sea_auto_checkbox_, sea_slider_, sea_value_label_);
  add_auto_slider_row(
    2, "旁瓣抑制", side_auto_checkbox_, side_slider_, side_value_label_);

  rain_slider_ = new QSlider(Qt::Horizontal, echo_group);
  rain_slider_->setRange(0, 255);
  rain_value_label_ = new QLabel("0", echo_group);
  rain_value_label_->setMinimumWidth(48);
  rain_value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  auto * manual_label = new QLabel("手动", echo_group);
  manual_label->setAlignment(Qt::AlignCenter);
  echo_layout->addWidget(new QLabel("雨雪杂波", echo_group), 3, 0);
  echo_layout->addWidget(manual_label, 3, 1);
  echo_layout->addWidget(rain_slider_, 3, 2);
  echo_layout->addWidget(rain_value_label_, 3, 3);
  echo_group->setLayout(echo_layout);
  main_layout->addWidget(echo_group);

  auto * filters_group = new QGroupBox("高级设置", content_widget);
  auto * filters_layout = new QFormLayout;
  filters_layout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  filters_layout->setRowWrapPolicy(QFormLayout::WrapLongRows);
  beam_spinbox_ = new QSpinBox(filters_group);
  beam_spinbox_->setRange(0, 3);
  beam_spinbox_->setValue(2);
  interference_spinbox_ = new QSpinBox(filters_group);
  interference_spinbox_->setRange(0, 3);
  noise_spinbox_ = new QSpinBox(filters_group);
  noise_spinbox_->setRange(0, 3);
  boost_spinbox_ = new QSpinBox(filters_group);
  boost_spinbox_->setRange(0, 2);
  boost_spinbox_->setValue(2);
  stretch_checkbox_ = new QCheckBox("启用", filters_group);
  stretch_checkbox_->setChecked(true);
  filters_layout->addRow("波束锐化 (0-3)：", beam_spinbox_);
  filters_layout->addRow("抑制同频干扰 (0-3)：", interference_spinbox_);
  filters_layout->addRow("噪声抑制 (0-3)：", noise_spinbox_);
  filters_layout->addRow("目标增强 (0-2)：", boost_spinbox_);
  filters_layout->addRow("目标扩展：", stretch_checkbox_);
  filters_group->setLayout(filters_layout);
  main_layout->addWidget(filters_group);

  auto * ins_group = new QGroupBox("惯导配置", content_widget);
  auto * ins_layout = new QFormLayout;
  ins_layout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  ins_layout->setRowWrapPolicy(QFormLayout::WrapLongRows);
  ins_enable_checkbox_ = new QCheckBox("启用", ins_group);
  ins_enable_checkbox_->setChecked(true);
  ins_ip_edit_ = new QLineEdit("224.100.100.30", ins_group);
  ins_port_spinbox_ = new QSpinBox(ins_group);
  ins_port_spinbox_->setRange(1, 65535);
  ins_port_spinbox_->setValue(20842);
  ins_layout->addRow("惯导使能：", ins_enable_checkbox_);
  ins_layout->addRow("惯导 IP：", ins_ip_edit_);
  ins_layout->addRow("惯导端口：", ins_port_spinbox_);
  ins_group->setLayout(ins_layout);
  main_layout->addWidget(ins_group);

  auto * status_group = new QGroupBox("状态", content_widget);
  auto * status_layout = new QFormLayout;
  connection_label_ = new QLabel("Disconnected", status_group);
  connection_label_->setStyleSheet("color: red; font-weight: bold;");
  radar_state_label_ = new QLabel("Unknown", status_group);
  radar_state_label_->setWordWrap(true);
  radar_state_label_->setStyleSheet("color: gray; font-weight: bold;");
  status_layout->addRow("连接状态：", connection_label_);
  status_layout->addRow("雷达状态：", radar_state_label_);
  status_group->setLayout(status_layout);
  main_layout->addWidget(status_group);

  main_layout->addStretch();

  content_widget->setLayout(main_layout);
  auto * scroll_area = new QScrollArea(this);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  scroll_area->setWidget(content_widget);

  // Keep the primary actions visible while the settings area scrolls.
  auto * action_widget = new QWidget(this);
  auto * button_layout = new QHBoxLayout(action_widget);
  button_layout->setContentsMargins(8, 6, 8, 8);
  button_layout->setSpacing(6);
  auto * refresh_button = new QPushButton("刷新参数", action_widget);
  auto * apply_button = new QPushButton("应用全部", action_widget);
  auto * query_button = new QPushButton("查询状态", action_widget);
  button_layout->addWidget(refresh_button);
  button_layout->addWidget(apply_button);
  button_layout->addWidget(query_button);

  auto * panel_layout = new QVBoxLayout;
  panel_layout->setContentsMargins(0, 0, 0, 0);
  panel_layout->setSpacing(0);
  panel_layout->addWidget(scroll_area, 1);
  panel_layout->addWidget(action_widget, 0);
  setLayout(panel_layout);

  connect(power_on_button, &QPushButton::clicked, this, [this]() {callPowerService(true);});
  connect(power_off_button, &QPushButton::clicked, this, [this]() {callPowerService(false);});
  connect(
    transmit_on_button, &QPushButton::clicked, this,
    [this]() {callTransmitService(true);});
  connect(
    transmit_off_button, &QPushButton::clicked, this,
    [this]() {callTransmitService(false);});
  connect(
    range_combo_, QOverload<int>::of(&QComboBox::activated),
    this, &RadarPanel::onRangeChanged);
  connect(
    rpm_combo_, QOverload<int>::of(&QComboBox::activated),
    this, &RadarPanel::onRpmChanged);
  connect(
    gain_auto_checkbox_, &QCheckBox::toggled,
    this, &RadarPanel::onGainAutoToggled);
  connect(sea_auto_checkbox_, &QCheckBox::toggled, this, &RadarPanel::onSeaAutoToggled);
  connect(side_auto_checkbox_, &QCheckBox::toggled, this, &RadarPanel::onSideAutoToggled);
  connect(gain_slider_, &QSlider::valueChanged, gain_value_label_, qOverload<int>(&QLabel::setNum));
  connect(sea_slider_, &QSlider::valueChanged, sea_value_label_, qOverload<int>(&QLabel::setNum));
  connect(side_slider_, &QSlider::valueChanged, side_value_label_, qOverload<int>(&QLabel::setNum));
  connect(rain_slider_, &QSlider::valueChanged, rain_value_label_, qOverload<int>(&QLabel::setNum));
  connect(gain_slider_, &QSlider::sliderReleased, this, &RadarPanel::onGainReleased);
  connect(sea_slider_, &QSlider::sliderReleased, this, &RadarPanel::onSeaReleased);
  connect(side_slider_, &QSlider::sliderReleased, this, &RadarPanel::onSideReleased);
  connect(rain_slider_, &QSlider::sliderReleased, this, &RadarPanel::onRainReleased);
  connect(
    beam_spinbox_, qOverload<int>(&QSpinBox::valueChanged), this,
    [this](int value) {setIntegerParameter("beam_sharpening", value);});
  connect(
    interference_spinbox_, qOverload<int>(&QSpinBox::valueChanged), this,
    [this](int value) {setIntegerParameter("interference_reject", value);});
  connect(
    noise_spinbox_, qOverload<int>(&QSpinBox::valueChanged), this,
    [this](int value) {setIntegerParameter("noise_reject", value);});
  connect(
    boost_spinbox_, qOverload<int>(&QSpinBox::valueChanged), this,
    [this](int value) {setIntegerParameter("target_boost", value);});
  connect(
    stretch_checkbox_, &QCheckBox::toggled, this,
    [this](bool checked) {setBooleanParameter("target_stretch", checked);});
  connect(
    ins_enable_checkbox_, &QCheckBox::toggled, this,
    [this](bool checked) {setBooleanParameter("enable_ins", checked);});
  connect(
    ins_ip_edit_, &QLineEdit::editingFinished,
    this, &RadarPanel::onInsIpEditingFinished);
  connect(
    ins_port_spinbox_, qOverload<int>(&QSpinBox::valueChanged), this,
    [this](int value) {setIntegerParameter("ins_port", value);});
  connect(refresh_button, &QPushButton::clicked, this, &RadarPanel::refreshParameters);
  connect(apply_button, &QPushButton::clicked, this, &RadarPanel::applyAllParameters);
  connect(query_button, &QPushButton::clicked, this, &RadarPanel::queryRadarState);

  connection_timer_ = new QTimer(this);
  connection_timer_->setInterval(1000);
  connect(connection_timer_, &QTimer::timeout, this, &RadarPanel::checkConnection);
}

void RadarPanel::setupRos()
{
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "--remap", "__node:=rviz_radar_control_panel", "--"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  set_parameters_client_ =
    client_node_->create_client<rcl_interfaces::srv::SetParameters>(
    target_node_ + "/set_parameters");
  get_parameters_client_ =
    client_node_->create_client<rcl_interfaces::srv::GetParameters>(
    target_node_ + "/get_parameters");
  power_client_ = client_node_->create_client<std_srvs::srv::SetBool>("/radar/set_power");
  transmit_client_ =
    client_node_->create_client<std_srvs::srv::SetBool>("/radar/set_transmit");
  query_client_ = client_node_->create_client<std_srvs::srv::Trigger>("/radar/query_all");

  parameter_event_sub_ =
    client_node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", rclcpp::ParameterEventsQoS(),
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
      handleParameterEvent(msg);
    });

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(client_node_);
  spin_thread_ = std::thread([this]() {executor_->spin();});
  connection_timer_->start();
  checkConnection();
}

void RadarPanel::checkConnection()
{
  const bool available =
    set_parameters_client_ && get_parameters_client_ &&
    set_parameters_client_->service_is_ready() && get_parameters_client_->service_is_ready();
  if (available != connected_) {
    connected_ = available;
    updateConnectionStatus(connected_);
    if (connected_) {
      refreshParameters();
    }
  }
}

void RadarPanel::updateConnectionStatus(bool connected)
{
  connection_label_->setText(connected ? "Connected" : "Disconnected");
  connection_label_->setStyleSheet(
    connected ? "color: green; font-weight: bold;" :
    "color: red; font-weight: bold;");
}

void RadarPanel::updateOperationStatus(const QString & text, bool success)
{
  radar_state_label_->setText(text);
  radar_state_label_->setStyleSheet(
    success ? "color: green; font-weight: bold;" :
    "color: red; font-weight: bold;");
}

std::vector<std::string> RadarPanel::parameterNames()
{
  return {
    "radar_range", "radar_rpm", "radar_gain", "sea_clutter", "side_lobe",
    "rain_clutter", "beam_sharpening", "interference_reject", "noise_reject",
    "target_boost", "target_stretch", "enable_ins", "ins_ip", "ins_port"};
}

rcl_interfaces::msg::Parameter RadarPanel::makeIntegerParameter(
  const std::string & name, int64_t value)
{
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = name;
  parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  parameter.value.integer_value = value;
  return parameter;
}

rcl_interfaces::msg::Parameter RadarPanel::makeBooleanParameter(
  const std::string & name, bool value)
{
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = name;
  parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  parameter.value.bool_value = value;
  return parameter;
}

rcl_interfaces::msg::Parameter RadarPanel::makeStringParameter(
  const std::string & name, const std::string & value)
{
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = name;
  parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  parameter.value.string_value = value;
  return parameter;
}

void RadarPanel::setIntegerParameter(const std::string & name, int64_t value)
{
  sendParameters({makeIntegerParameter(name, value)}, "参数已更新");
}

void RadarPanel::setBooleanParameter(const std::string & name, bool value)
{
  sendParameters({makeBooleanParameter(name, value)}, "参数已更新");
}

void RadarPanel::setStringParameter(const std::string & name, const std::string & value)
{
  sendParameters({makeStringParameter(name, value)}, "参数已更新");
}

void RadarPanel::sendParameters(
  const std::vector<rcl_interfaces::msg::Parameter> & parameters,
  const QString & success_text)
{
  if (!set_parameters_client_ || !set_parameters_client_->service_is_ready()) {
    updateOperationStatus("参数服务不可用", false);
    return;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  request->parameters = parameters;
  set_parameters_client_->async_send_request(
    request,
    [this, success_text](
      rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future)
    {
      bool successful = true;
      QString reason;
      try {
        const auto response = future.get();
        for (const auto & result : response->results) {
          if (!result.successful) {
            successful = false;
            reason = QString::fromStdString(result.reason);
            break;
          }
        }
      } catch (const std::exception & error) {
        successful = false;
        reason = QString::fromLocal8Bit(error.what());
      }
      const QString text = successful ? success_text :
      (reason.isEmpty() ? "参数设置失败" : "参数设置失败：" + reason);
      QMetaObject::invokeMethod(
        this, [this, text, successful]() {updateOperationStatus(text, successful);},
        Qt::QueuedConnection);
    });
}

void RadarPanel::refreshParameters()
{
  if (!get_parameters_client_ || !get_parameters_client_->service_is_ready()) {
    updateOperationStatus("参数查询服务不可用", false);
    return;
  }

  const auto names = parameterNames();
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = names;
  get_parameters_client_->async_send_request(
    request,
    [this, names](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
      try {
        const auto response = future.get();
        const auto values = response->values;
        QMetaObject::invokeMethod(
          this,
          [this, names, values]() {
            const std::size_t count = std::min(names.size(), values.size());
            for (std::size_t index = 0; index < count; ++index) {
              updateParameter(names[index], values[index]);
            }
            updateOperationStatus("参数已刷新", true);
          },
          Qt::QueuedConnection);
      } catch (const std::exception & error) {
        const QString text = "参数刷新失败：" + QString::fromLocal8Bit(error.what());
        QMetaObject::invokeMethod(
          this, [this, text]() {updateOperationStatus(text, false);}, Qt::QueuedConnection);
      }
    });
}

void RadarPanel::applyAllParameters()
{
  const int range = range_combo_->currentData().toInt();
  std::vector<rcl_interfaces::msg::Parameter> parameters;
  parameters.reserve(14);
  parameters.push_back(makeIntegerParameter("radar_range", range));
  parameters.push_back(makeIntegerParameter("radar_rpm", rpm_combo_->currentIndex()));
  parameters.push_back(
    makeIntegerParameter(
      "radar_gain", gain_auto_checkbox_->isChecked() ? -1 : gain_slider_->value()));
  parameters.push_back(
    makeIntegerParameter(
      "sea_clutter", sea_auto_checkbox_->isChecked() ? -1 : sea_slider_->value()));
  parameters.push_back(
    makeIntegerParameter(
      "side_lobe", side_auto_checkbox_->isChecked() ? -1 : side_slider_->value()));
  parameters.push_back(makeIntegerParameter("rain_clutter", rain_slider_->value()));
  parameters.push_back(makeIntegerParameter("beam_sharpening", beam_spinbox_->value()));
  parameters.push_back(
    makeIntegerParameter(
      "interference_reject", interference_spinbox_->value()));
  parameters.push_back(makeIntegerParameter("noise_reject", noise_spinbox_->value()));
  parameters.push_back(makeIntegerParameter("target_boost", boost_spinbox_->value()));
  parameters.push_back(makeBooleanParameter("target_stretch", stretch_checkbox_->isChecked()));
  parameters.push_back(makeBooleanParameter("enable_ins", ins_enable_checkbox_->isChecked()));
  parameters.push_back(makeStringParameter("ins_ip", ins_ip_edit_->text().toStdString()));
  parameters.push_back(makeIntegerParameter("ins_port", ins_port_spinbox_->value()));
  sendParameters(parameters, "全部参数已应用");
}

void RadarPanel::callPowerService(bool enabled)
{
  if (!power_client_ || !power_client_->service_is_ready()) {
    updateOperationStatus("电源服务不可用", false);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  const QString action = enabled ? "开机" : "关机";
  power_client_->async_send_request(
    request,
    [this, action](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      bool success = false;
      QString text;
      try {
        const auto response = future.get();
        success = response->success;
        const QString message = QString::fromStdString(response->message);
        text = action + (success ? "成功" : "失败");
        if (!message.isEmpty()) {
          text += "：" + message;
        }
      } catch (const std::exception & error) {
        text = action + "异常：" + QString::fromLocal8Bit(error.what());
      }
      QMetaObject::invokeMethod(
        this, [this, text, success]() {updateOperationStatus(text, success);},
        Qt::QueuedConnection);
    });
}

void RadarPanel::callTransmitService(bool enabled)
{
  if (!transmit_client_ || !transmit_client_->service_is_ready()) {
    updateOperationStatus("发射服务不可用", false);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  const QString action = enabled ? "发射" : "待机";
  transmit_client_->async_send_request(
    request,
    [this, action](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      bool success = false;
      QString text;
      try {
        const auto response = future.get();
        success = response->success;
        const QString message = QString::fromStdString(response->message);
        text = action + (success ? "成功" : "失败");
        if (!message.isEmpty()) {
          text += "：" + message;
        }
      } catch (const std::exception & error) {
        text = action + "异常：" + QString::fromLocal8Bit(error.what());
      }
      QMetaObject::invokeMethod(
        this, [this, text, success]() {updateOperationStatus(text, success);},
        Qt::QueuedConnection);
    });
}

void RadarPanel::queryRadarState()
{
  if (!query_client_ || !query_client_->service_is_ready()) {
    updateOperationStatus("查询服务不可用", false);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  query_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      bool success = false;
      QString text;
      try {
        const auto response = future.get();
        success = response->success;
        text = QString::fromStdString(response->message);
        if (text.isEmpty()) {
          text = success ? "查询成功" : "查询失败";
        }
      } catch (const std::exception & error) {
        text = "查询异常：" + QString::fromLocal8Bit(error.what());
      }
      QMetaObject::invokeMethod(
        this, [this, text, success]() {updateOperationStatus(text, success);},
        Qt::QueuedConnection);
    });
}

void RadarPanel::handleParameterEvent(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
{
  if (msg->node != target_node_) {
    return;
  }

  auto enqueue = [this](const rcl_interfaces::msg::Parameter & parameter) {
      const auto name = parameter.name;
      const auto value = parameter.value;
      QMetaObject::invokeMethod(
        this, [this, name, value]() {updateParameter(name, value);}, Qt::QueuedConnection);
    };
  for (const auto & parameter : msg->new_parameters) {
    enqueue(parameter);
  }
  for (const auto & parameter : msg->changed_parameters) {
    enqueue(parameter);
  }
}

void RadarPanel::updateParameter(
  const std::string & name,
  const rcl_interfaces::msg::ParameterValue & value)
{
  const bool is_boolean_parameter = name == "target_stretch" || name == "enable_ins";
  const bool is_string_parameter = name == "ins_ip";
  if ((is_boolean_parameter &&
    value.type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) ||
    (is_string_parameter &&
    value.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) ||
    (!is_boolean_parameter && !is_string_parameter &&
    value.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER))
  {
    return;
  }

  if (name == "radar_range") {
    const int index = range_combo_->findData(static_cast<int>(value.integer_value));
    if (index >= 0) {
      const QSignalBlocker blocker(range_combo_);
      range_combo_->setCurrentIndex(index);
    }
  } else if (name == "radar_rpm") {
    if (value.integer_value >= 0 && value.integer_value < rpm_combo_->count()) {
      const QSignalBlocker blocker(rpm_combo_);
      rpm_combo_->setCurrentIndex(static_cast<int>(value.integer_value));
    }
  } else if (name == "radar_gain") {
    updateAutoSlider(value.integer_value, gain_auto_checkbox_, gain_slider_, gain_value_label_);
  } else if (name == "sea_clutter") {
    updateAutoSlider(value.integer_value, sea_auto_checkbox_, sea_slider_, sea_value_label_);
  } else if (name == "side_lobe") {
    updateAutoSlider(value.integer_value, side_auto_checkbox_, side_slider_, side_value_label_);
  } else if (name == "rain_clutter") {
    const QSignalBlocker blocker(rain_slider_);
    rain_slider_->setValue(static_cast<int>(value.integer_value));
    rain_value_label_->setText(QString::number(static_cast<qlonglong>(value.integer_value)));
  } else if (name == "beam_sharpening") {
    const QSignalBlocker blocker(beam_spinbox_);
    beam_spinbox_->setValue(static_cast<int>(value.integer_value));
  } else if (name == "interference_reject") {
    const QSignalBlocker blocker(interference_spinbox_);
    interference_spinbox_->setValue(static_cast<int>(value.integer_value));
  } else if (name == "noise_reject") {
    const QSignalBlocker blocker(noise_spinbox_);
    noise_spinbox_->setValue(static_cast<int>(value.integer_value));
  } else if (name == "target_boost") {
    const QSignalBlocker blocker(boost_spinbox_);
    boost_spinbox_->setValue(static_cast<int>(value.integer_value));
  } else if (name == "target_stretch") {
    const QSignalBlocker blocker(stretch_checkbox_);
    stretch_checkbox_->setChecked(value.bool_value);
  } else if (name == "enable_ins") {
    const QSignalBlocker blocker(ins_enable_checkbox_);
    ins_enable_checkbox_->setChecked(value.bool_value);
  } else if (name == "ins_ip") {
    const QSignalBlocker blocker(ins_ip_edit_);
    ins_ip_edit_->setText(QString::fromStdString(value.string_value));
  } else if (name == "ins_port") {
    const QSignalBlocker blocker(ins_port_spinbox_);
    ins_port_spinbox_->setValue(static_cast<int>(value.integer_value));
  }
}

void RadarPanel::updateAutoSlider(
  int64_t value, QCheckBox * automatic, QSlider * slider, QLabel * label)
{
  const bool is_automatic = value == -1;
  const QSignalBlocker checkbox_blocker(automatic);
  const QSignalBlocker slider_blocker(slider);
  automatic->setChecked(is_automatic);
  slider->setEnabled(!is_automatic);
  if (!is_automatic) {
    slider->setValue(static_cast<int>(value));
  }
  label->setText(is_automatic ? "Auto" : QString::number(value));
}

void RadarPanel::onRangeChanged(int index)
{
  setIntegerParameter("radar_range", range_combo_->itemData(index).toInt());
}

void RadarPanel::onRpmChanged(int index)
{
  setIntegerParameter("radar_rpm", index);
}

void RadarPanel::onGainAutoToggled(bool checked)
{
  gain_slider_->setEnabled(!checked);
  gain_value_label_->setText(checked ? "Auto" : QString::number(gain_slider_->value()));
  setIntegerParameter("radar_gain", checked ? -1 : gain_slider_->value());
}

void RadarPanel::onSeaAutoToggled(bool checked)
{
  sea_slider_->setEnabled(!checked);
  sea_value_label_->setText(checked ? "Auto" : QString::number(sea_slider_->value()));
  setIntegerParameter("sea_clutter", checked ? -1 : sea_slider_->value());
}

void RadarPanel::onSideAutoToggled(bool checked)
{
  side_slider_->setEnabled(!checked);
  side_value_label_->setText(checked ? "Auto" : QString::number(side_slider_->value()));
  setIntegerParameter("side_lobe", checked ? -1 : side_slider_->value());
}

void RadarPanel::onGainReleased()
{
  if (!gain_auto_checkbox_->isChecked()) {
    setIntegerParameter("radar_gain", gain_slider_->value());
  }
}

void RadarPanel::onSeaReleased()
{
  if (!sea_auto_checkbox_->isChecked()) {
    setIntegerParameter("sea_clutter", sea_slider_->value());
  }
}

void RadarPanel::onSideReleased()
{
  if (!side_auto_checkbox_->isChecked()) {
    setIntegerParameter("side_lobe", side_slider_->value());
  }
}

void RadarPanel::onRainReleased()
{
  setIntegerParameter("rain_clutter", rain_slider_->value());
}

void RadarPanel::onInsIpEditingFinished()
{
  setStringParameter("ins_ip", ins_ip_edit_->text().toStdString());
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::RadarPanel, rviz_common::Panel)
