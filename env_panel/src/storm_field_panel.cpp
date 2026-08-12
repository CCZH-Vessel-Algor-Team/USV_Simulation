#include "env_panel/storm_field_panel.hpp"

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

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

  auto config_group = new QGroupBox("Storm Config");
  auto form = new QFormLayout;

  drift_heading_slider_ = new QSlider(Qt::Horizontal);
  drift_heading_slider_->setRange(0, 359);
  drift_heading_slider_->setValue(static_cast<int>(drift_heading_deg_));
  drift_heading_label_ = new QLabel("0 deg");
  auto heading_row = new QHBoxLayout;
  heading_row->addWidget(drift_heading_slider_);
  heading_row->addWidget(drift_heading_label_);
  form->addRow("Drift Dir:", heading_row);
  QObject::connect(drift_heading_slider_, &QSlider::valueChanged,
                   this, &StormFieldPanel::onDriftHeadingChanged);

  drift_speed_slider_ = new QSlider(Qt::Horizontal);
  drift_speed_slider_->setRange(0, 100);
  drift_speed_slider_->setValue(static_cast<int>(drift_speed_ * 10.0));
  drift_speed_label_ = new QLabel("0.0 m/s");
  auto speed_row = new QHBoxLayout;
  speed_row->addWidget(drift_speed_slider_);
  speed_row->addWidget(drift_speed_label_);
  form->addRow("Drift Speed:", speed_row);
  QObject::connect(drift_speed_slider_, &QSlider::valueChanged,
                   this, &StormFieldPanel::onDriftSpeedChanged);

  radius_slider_ = new QSlider(Qt::Horizontal);
  radius_slider_->setRange(10, 5000);
  radius_slider_->setValue(static_cast<int>(radius_ * 10.0));
  radius_label_ = new QLabel("30.0 m");
  auto radius_row = new QHBoxLayout;
  radius_row->addWidget(radius_slider_);
  radius_row->addWidget(radius_label_);
  form->addRow("Radius:", radius_row);
  QObject::connect(radius_slider_, &QSlider::valueChanged,
                   this, &StormFieldPanel::onRadiusChanged);

  weather_validity_slider_ = new QSlider(Qt::Horizontal);
  weather_validity_slider_->setRange(1, 1440);
  weather_validity_slider_->setValue(
      static_cast<int>(weather_validity_duration_s_ / 60.0));
  weather_validity_label_ = new QLabel("60 min");
  auto validity_row = new QHBoxLayout;
  validity_row->addWidget(weather_validity_slider_);
  validity_row->addWidget(weather_validity_label_);
  form->addRow("Weather Validity:", validity_row);
  QObject::connect(weather_validity_slider_, &QSlider::valueChanged,
                   this, &StormFieldPanel::onWeatherValidityChanged);

  weather_grid_slider_ = new QSlider(Qt::Horizontal);
  weather_grid_slider_->setRange(1, 10000);
  weather_grid_slider_->setValue(static_cast<int>(weather_grid_resolution_m_));
  weather_grid_label_ = new QLabel("100 m");
  auto grid_row = new QHBoxLayout;
  grid_row->addWidget(weather_grid_slider_);
  grid_row->addWidget(weather_grid_label_);
  form->addRow("Weather Grid:", grid_row);
  QObject::connect(weather_grid_slider_, &QSlider::valueChanged,
                   this, &StormFieldPanel::onWeatherGridChanged);

  config_group->setLayout(form);
  main_layout->addWidget(config_group);

  auto list_group = new QGroupBox("Active Storm");
  auto list_layout = new QVBoxLayout;
  storm_list_ = new QListWidget;
  list_layout->addWidget(storm_list_);

  auto btn_layout = new QHBoxLayout;
  auto delete_btn = new QPushButton("Delete Selected");
  QObject::connect(delete_btn, &QPushButton::clicked,
                   this, &StormFieldPanel::onDeleteClicked);
  btn_layout->addWidget(delete_btn);

  clear_all_button_ = new QPushButton("Clear All");
  QObject::connect(clear_all_button_, &QPushButton::clicked,
                   this, &StormFieldPanel::onClearAllClicked);
  btn_layout->addWidget(clear_all_button_);

  list_layout->addLayout(btn_layout);
  list_group->setLayout(list_layout);
  main_layout->addWidget(list_group);

  auto hint = new QLabel(
      "Use RViz 'Storm Point' tool to add storm centers.\n"
      "Storms publish /storm_field/markers and TrackedShipList hazards.");
  hint->setWordWrap(true);
  main_layout->addWidget(hint);

  main_layout->setContentsMargins(8, 8, 8, 8);
  setLayout(main_layout);

  config_timer_ = new QTimer(this);
  config_timer_->setInterval(200);
  QObject::connect(config_timer_, &QTimer::timeout,
                   this, &StormFieldPanel::publishConfig);
}

void StormFieldPanel::onInitialize()
{
  auto raw_node =
      getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

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
        onNamesReceived(msg);
      });

  (void)raw_node;
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
  if (!config_client_->wait_for_service(std::chrono::milliseconds(100))) {
    return;
  }

  static double last_heading = -1.0, last_speed = -1.0,
                last_radius = -1.0, last_validity = -1.0,
                last_grid = -1.0;

  if (last_heading == drift_heading_deg_ && last_speed == drift_speed_ &&
      last_radius == radius_ &&
      last_validity == weather_validity_duration_s_ &&
      last_grid == weather_grid_resolution_m_) {
    return;
  }

  last_heading = drift_heading_deg_;
  last_speed = drift_speed_;
  last_radius = radius_;
  last_validity = weather_validity_duration_s_;
  last_grid = weather_grid_resolution_m_;

  auto req =
      std::make_shared<usv_interfaces::srv::SetStormFieldConfig::Request>();
  req->drift_heading_deg = drift_heading_deg_;
  req->drift_speed = drift_speed_;
  req->radius = radius_;
  req->weather_validity_duration_s = weather_validity_duration_s_;
  req->weather_grid_resolution_m = weather_grid_resolution_m_;
  config_client_->async_send_request(req);
}

void StormFieldPanel::onDeleteClicked()
{
  auto selected = storm_list_->currentItem();
  if (!selected) return;
  if (!delete_client_->wait_for_service(std::chrono::seconds(1))) return;

  auto req =
      std::make_shared<usv_interfaces::srv::DeleteStormField::Request>();
  req->storm_name = selected->text().toStdString();
  delete_client_->async_send_request(req);
}

void StormFieldPanel::onClearAllClicked()
{
  if (!clear_client_->wait_for_service(std::chrono::seconds(1))) return;

  auto req =
      std::make_shared<usv_interfaces::srv::ClearStormFields::Request>();
  clear_client_->async_send_request(req);
}

void StormFieldPanel::onNamesReceived(
    const std_msgs::msg::String::SharedPtr msg)
{
  std::vector<std::string> new_names;
  std::string buffer;
  bool in_string = false;
  for (char c : msg->data) {
    if (c == '"') {
      in_string = !in_string;
      if (!in_string && !buffer.empty()) {
        new_names.push_back(buffer);
        buffer.clear();
      }
    } else if (in_string) {
      buffer += c;
    }
  }

  if (new_names != storm_names_) {
    storm_names_ = new_names;
    auto names_copy = new_names;
    QMetaObject::invokeMethod(
        this,
        [this, names_copy]() {
          storm_list_->clear();
          for (const auto & name : names_copy) {
            storm_list_->addItem(QString::fromStdString(name));
          }
        },
        Qt::QueuedConnection);
  }
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::StormFieldPanel, rviz_common::Panel)
