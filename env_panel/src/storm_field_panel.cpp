#include "env_panel/storm_field_panel.hpp"
#include "env_panel/target_list.hpp"

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace env_panel
{

StormFieldPanel::StormFieldPanel(QWidget * parent)
: rviz_common::Panel(parent),
  drift_heading_deg_(0.0),
  drift_speed_(0.0),
  radius_(30.0),
  weather_validity_duration_s_(3600.0),
  weather_grid_resolution_m_(100.0)
{
  setupUi();
}

void StormFieldPanel::setupUi()
{
  auto main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(12, 10, 12, 12);
  main_layout->setSpacing(10);
  setStyleSheet(
    "QGroupBox { font-weight: 600; margin-top: 10px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
    "QPushButton { min-height: 28px; padding: 3px 10px; }");

  const int group_content_top_margin = fontMetrics().height() + 8;
  const auto configure_slider_row = [](QHBoxLayout * layout) {
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(12);
  };

  auto config_group = new QGroupBox("风暴配置");
  auto form = new QFormLayout;
  form->setContentsMargins(12, group_content_top_margin, 12, 10);
  form->setHorizontalSpacing(14);
  form->setVerticalSpacing(8);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

  auto drift_heading_slider = new QSlider(Qt::Horizontal);
  drift_heading_slider->setRange(0, 359);
  drift_heading_slider->setValue(static_cast<int>(drift_heading_deg_));
  drift_heading_label_ = new QLabel("0 deg");
  auto heading_row = new QHBoxLayout;
  configure_slider_row(heading_row);
  heading_row->addWidget(drift_heading_slider, 1);
  heading_row->addWidget(drift_heading_label_);
  drift_heading_slider->setToolTip("北向为 0°，顺时针增加；单击使用此值，拖拽使用箭头方向。");
  form->addRow("默认漂移方向：", heading_row);
  QObject::connect(drift_heading_slider, &QSlider::valueChanged,
                   this, &StormFieldPanel::onDriftHeadingChanged);

  auto drift_speed_slider = new QSlider(Qt::Horizontal);
  drift_speed_slider->setRange(0, 100);
  drift_speed_slider->setValue(static_cast<int>(drift_speed_ * 10.0));
  drift_speed_label_ = new QLabel("0.0 m/s");
  auto speed_row = new QHBoxLayout;
  configure_slider_row(speed_row);
  speed_row->addWidget(drift_speed_slider, 1);
  speed_row->addWidget(drift_speed_label_);
  form->addRow("漂移速度：", speed_row);
  QObject::connect(drift_speed_slider, &QSlider::valueChanged,
                   this, &StormFieldPanel::onDriftSpeedChanged);

  auto radius_slider = new QSlider(Qt::Horizontal);
  radius_slider->setRange(10, 5000);
  radius_slider->setValue(static_cast<int>(radius_ * 10.0));
  radius_label_ = new QLabel("30.0 m");
  auto radius_row = new QHBoxLayout;
  configure_slider_row(radius_row);
  radius_row->addWidget(radius_slider, 1);
  radius_row->addWidget(radius_label_);
  form->addRow("半径：", radius_row);
  QObject::connect(radius_slider, &QSlider::valueChanged,
                   this, &StormFieldPanel::onRadiusChanged);

  auto weather_validity_slider = new QSlider(Qt::Horizontal);
  weather_validity_slider->setRange(1, 1440);
  weather_validity_slider->setValue(
      static_cast<int>(weather_validity_duration_s_ / 60.0));
  weather_validity_label_ = new QLabel("60 min");
  auto validity_row = new QHBoxLayout;
  configure_slider_row(validity_row);
  validity_row->addWidget(weather_validity_slider, 1);
  validity_row->addWidget(weather_validity_label_);
  form->addRow("有效时长：", validity_row);
  QObject::connect(weather_validity_slider, &QSlider::valueChanged,
                   this, &StormFieldPanel::onWeatherValidityChanged);

  auto weather_grid_slider = new QSlider(Qt::Horizontal);
  weather_grid_slider->setRange(1, 10000);
  weather_grid_slider->setValue(static_cast<int>(weather_grid_resolution_m_));
  weather_grid_label_ = new QLabel("100 m");
  auto grid_row = new QHBoxLayout;
  configure_slider_row(grid_row);
  grid_row->addWidget(weather_grid_slider, 1);
  grid_row->addWidget(weather_grid_label_);
  form->addRow("网格间距：", grid_row);
  QObject::connect(weather_grid_slider, &QSlider::valueChanged,
                   this, &StormFieldPanel::onWeatherGridChanged);

  config_group->setLayout(form);
  main_layout->addWidget(config_group);

  auto list_group = new QGroupBox("风暴列表");
  auto list_layout = new QVBoxLayout;
  list_layout->setContentsMargins(12, group_content_top_margin, 12, 10);
  list_layout->setSpacing(8);
  storm_list_ = new QListWidget;
  list_layout->addWidget(storm_list_);

  auto btn_layout = new QHBoxLayout;
  btn_layout->setContentsMargins(0, 0, 0, 0);
  btn_layout->setSpacing(10);
  auto delete_btn = new QPushButton("删除所选");
  QObject::connect(delete_btn, &QPushButton::clicked,
                   this, &StormFieldPanel::onDeleteClicked);
  btn_layout->addWidget(delete_btn);

  auto clear_all_button = new QPushButton("全部清除");
  QObject::connect(clear_all_button, &QPushButton::clicked,
                   this, &StormFieldPanel::onClearAllClicked);
  btn_layout->addWidget(clear_all_button);

  list_layout->addLayout(btn_layout);
  list_group->setLayout(list_layout);
  main_layout->addWidget(list_group);

  auto placement_hint = new QLabel(
    "备注：使用工具栏“添加风暴”；单击使用默认漂移方向，拖拽指定漂移方向。"
    "漂移速度为 0 时保持静止。右键取消，Esc 退出。");
  placement_hint->setObjectName("placement_note");
  placement_hint->setWordWrap(true);
  main_layout->addWidget(placement_hint);

  for (auto * label : {
      drift_heading_label_, drift_speed_label_, radius_label_,
      weather_validity_label_, weather_grid_label_})
  {
    label->setMinimumWidth(76);
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  }
  setLayout(main_layout);

  config_timer_ = new QTimer(this);
  config_timer_->setInterval(200);
  QObject::connect(config_timer_, &QTimer::timeout,
                   this, &StormFieldPanel::publishConfig);
}

void StormFieldPanel::onInitialize()
{
  auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "--remap",
       "__node:=rviz_storm_field_config_panel", "--"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  config_client_ =
      client_node_->create_client<usv_interfaces::srv::SetStormFieldConfig>(
          "/storm_field/set_config");
  delete_client_ =
      client_node_->create_client<usv_interfaces::srv::DeleteStormField>(
          "/storm_field/delete");
  clear_client_ =
      client_node_->create_client<usv_interfaces::srv::ClearStormFields>(
          "/storm_field/clear");

  names_sub_ = client_node_->create_subscription<std_msgs::msg::String>(
      "/storm_field/names", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        updateTargetNames(storm_list_, msg->data);
      });

  config_timer_->start();

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(client_node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });
}

StormFieldPanel::~StormFieldPanel()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void StormFieldPanel::onDriftHeadingChanged(int value)
{
  drift_heading_deg_ = static_cast<double>(value);
  drift_heading_label_->setText(
      QString("%1 deg").arg(static_cast<int>(drift_heading_deg_)));
}

void StormFieldPanel::onDriftSpeedChanged(int value)
{
  drift_speed_ = static_cast<double>(value) / 10.0;
  drift_speed_label_->setText(
      QString("%1 m/s").arg(drift_speed_, 0, 'f', 1));
}

void StormFieldPanel::onRadiusChanged(int value)
{
  radius_ = static_cast<double>(value) / 10.0;
  radius_label_->setText(
      QString("%1 m").arg(radius_, 0, 'f', 1));
}

void StormFieldPanel::onWeatherValidityChanged(int value)
{
  weather_validity_duration_s_ = static_cast<double>(value) * 60.0;
  weather_validity_label_->setText(QString("%1 min").arg(value));
}

void StormFieldPanel::onWeatherGridChanged(int value)
{
  weather_grid_resolution_m_ = static_cast<double>(value);
  weather_grid_label_->setText(QString("%1 m").arg(value));
}

void StormFieldPanel::publishConfig()
{
  if (!config_client_->service_is_ready()) {
    last_config_.reset();
    return;
  }

  auto req =
      std::make_shared<usv_interfaces::srv::SetStormFieldConfig::Request>();
  req->drift_heading_deg = drift_heading_deg_;
  req->drift_speed = drift_speed_;
  req->radius = radius_;
  req->weather_validity_duration_s = weather_validity_duration_s_;
  req->weather_grid_resolution_m = weather_grid_resolution_m_;
  if (last_config_ && *last_config_ == *req) {
    return;
  }
  last_config_ = req;
  config_client_->async_send_request(req,
    [this, req](rclcpp::Client<usv_interfaces::srv::SetStormFieldConfig>::SharedFuture future) {
      if (!future.get()->success) {
        QMetaObject::invokeMethod(this, [this, req]() {
          if (last_config_ == req) {last_config_.reset();}
        }, Qt::QueuedConnection);
      }
    });
}

void StormFieldPanel::onDeleteClicked()
{
  auto selected = storm_list_->currentItem();
  if (!selected) return;
  if (!delete_client_->service_is_ready()) return;

  auto req =
      std::make_shared<usv_interfaces::srv::DeleteStormField::Request>();
  req->storm_name = selected->text().toStdString();
  delete_client_->async_send_request(req);
}

void StormFieldPanel::onClearAllClicked()
{
  if (!clear_client_->service_is_ready()) return;

  auto req =
      std::make_shared<usv_interfaces::srv::ClearStormFields::Request>();
  clear_client_->async_send_request(req);
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::StormFieldPanel, rviz_common::Panel)
