#include "env_panel/static_buoy_panel.hpp"

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QMetaObject>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace env_panel
{

StaticBuoyPanel::StaticBuoyPanel(QWidget * parent)
: rviz_common::Panel(parent),
  buoy_type_("rgb"),
  radius_m_(0.8)
{
  setupUi();
}

void StaticBuoyPanel::setupUi()
{
  auto main_layout = new QVBoxLayout;

  auto config_group = new QGroupBox("Spawn Config");
  auto form = new QFormLayout;

  buoy_type_combo_ = new QComboBox;
  buoy_type_combo_->addItems({
      "rgb", "rgy", "yrg", "bgr", "bgy", "gbr", "rbg", "ybr", "ygb"});
  buoy_type_combo_->setCurrentText("rgb");
  form->addRow("Light Sequence:", buoy_type_combo_);
  QObject::connect(
      buoy_type_combo_,
      QOverload<const QString &>::of(&QComboBox::currentTextChanged),
      this, &StaticBuoyPanel::onBuoyTypeChanged);

  radius_slider_ = new QSlider(Qt::Horizontal);
  radius_slider_->setRange(2, 50);
  radius_slider_->setValue(static_cast<int>(radius_m_ * 10.0));
  radius_label_ = new QLabel("0.8 m");
  auto radius_row = new QHBoxLayout;
  radius_row->addWidget(radius_slider_);
  radius_row->addWidget(radius_label_);
  form->addRow("Radius:", radius_row);
  QObject::connect(radius_slider_, &QSlider::valueChanged,
                   this, &StaticBuoyPanel::onRadiusChanged);

  config_group->setLayout(form);
  main_layout->addWidget(config_group);

  auto list_group = new QGroupBox("Active Static Buoys");
  auto list_layout = new QVBoxLayout;
  buoy_list_ = new QListWidget;
  list_layout->addWidget(buoy_list_);

  auto btn_layout = new QHBoxLayout;
  auto delete_btn = new QPushButton("Delete Selected");
  QObject::connect(delete_btn, &QPushButton::clicked,
                   this, &StaticBuoyPanel::onDeleteClicked);
  btn_layout->addWidget(delete_btn);

  clear_all_button_ = new QPushButton("Clear All");
  QObject::connect(clear_all_button_, &QPushButton::clicked,
                   this, &StaticBuoyPanel::onClearAllClicked);
  btn_layout->addWidget(clear_all_button_);

  list_layout->addLayout(btn_layout);
  list_group->setLayout(list_layout);
  main_layout->addWidget(list_group);

  auto hint = new QLabel(
      "Use RViz 'Static Buoy Point' tool to click in 3D view.\n"
      "Static VRX light buoys spawn at the clicked position.\n"
      "BuoyArray, markers, and COLREGS/ground-truth bridges update automatically.");
  hint->setWordWrap(true);
  main_layout->addWidget(hint);

  main_layout->setContentsMargins(8, 8, 8, 8);
  setLayout(main_layout);

  config_timer_ = new QTimer(this);
  config_timer_->setInterval(200);
  QObject::connect(config_timer_, &QTimer::timeout,
                   this, &StaticBuoyPanel::publishConfig);
}

void StaticBuoyPanel::onInitialize()
{
  auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "--remap",
       "__node:=rviz_static_buoy_config_panel", "--"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  delete_client_ =
      client_node_->create_client<usv_interfaces::srv::DeleteDynamicBuoy>(
          "/dynamic_buoy/delete");
  clear_client_ =
      client_node_->create_client<usv_interfaces::srv::ClearDynamicBuoys>(
          "/dynamic_buoy/clear");
  config_client_ =
      client_node_->create_client<usv_interfaces::srv::SetDynamicBuoyConfig>(
          "/dynamic_buoy/set_config");

  names_sub_ = client_node_->create_subscription<std_msgs::msg::String>(
      "/dynamic_buoy/names", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        onNamesReceived(msg);
      });

  config_timer_->start();

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(client_node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });
}

StaticBuoyPanel::~StaticBuoyPanel()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void StaticBuoyPanel::onBuoyTypeChanged(const QString & text)
{
  buoy_type_ = text.toStdString();
}

void StaticBuoyPanel::onRadiusChanged(int value)
{
  radius_m_ = static_cast<double>(value) / 10.0;
  radius_label_->setText(QString("%1 m").arg(radius_m_, 0, 'f', 1));
}

void StaticBuoyPanel::publishConfig()
{
  if (!config_client_->wait_for_service(std::chrono::milliseconds(100))) {
    return;
  }

  static std::string last_type;
  static double last_radius = -1.0;
  if (last_type == buoy_type_ && last_radius == radius_m_) {
    return;
  }
  last_type = buoy_type_;
  last_radius = radius_m_;

  auto req = std::make_shared<usv_interfaces::srv::SetDynamicBuoyConfig::Request>();
  req->buoy_type = buoy_type_;
  req->radius_m = radius_m_;
  config_client_->async_send_request(req);
}

void StaticBuoyPanel::onDeleteClicked()
{
  auto selected = buoy_list_->currentItem();
  if (!selected) {
    return;
  }
  if (!delete_client_->wait_for_service(std::chrono::seconds(1))) {
    return;
  }
  auto req = std::make_shared<usv_interfaces::srv::DeleteDynamicBuoy::Request>();
  req->model_name = selected->text().toStdString();
  delete_client_->async_send_request(req);
}

void StaticBuoyPanel::onClearAllClicked()
{
  if (!clear_client_->wait_for_service(std::chrono::seconds(1))) {
    return;
  }
  auto req = std::make_shared<usv_interfaces::srv::ClearDynamicBuoys::Request>();
  clear_client_->async_send_request(req);
}

void StaticBuoyPanel::onNamesReceived(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == last_names_json_) {
    return;
  }
  last_names_json_ = msg->data;

  const QJsonDocument doc =
      QJsonDocument::fromJson(QString::fromStdString(msg->data).toUtf8());
  if (!doc.isArray()) {
    return;
  }

  std::vector<std::string> new_names;
  new_names.reserve(doc.array().size());
  for (const QJsonValue & value : doc.array()) {
    if (!value.isString()) {
      continue;
    }
    new_names.push_back(value.toString().toStdString());
  }

  if (new_names == buoy_names_) {
    return;
  }
  buoy_names_ = new_names;
  auto names_copy = new_names;
  QMetaObject::invokeMethod(
      this,
      [this, names_copy]() {
        updateBuoyList(names_copy);
      },
      Qt::QueuedConnection);
}

void StaticBuoyPanel::updateBuoyList(const std::vector<std::string> & names)
{
  const QString selected =
      buoy_list_->currentItem() ? buoy_list_->currentItem()->text() : QString();

  QStringList existing;
  existing.reserve(buoy_list_->count());
  for (int i = 0; i < buoy_list_->count(); ++i) {
    existing.push_back(buoy_list_->item(i)->text());
  }

  QStringList incoming;
  incoming.reserve(static_cast<int>(names.size()));
  for (const auto & name : names) {
    incoming.push_back(QString::fromStdString(name));
  }

  if (existing == incoming) {
    return;
  }

  buoy_list_->blockSignals(true);
  buoy_list_->clear();
  for (const auto & name : names) {
    buoy_list_->addItem(QString::fromStdString(name));
  }

  if (!selected.isEmpty()) {
    const auto matches = buoy_list_->findItems(selected, Qt::MatchExactly);
    if (!matches.isEmpty()) {
      buoy_list_->setCurrentItem(matches.front());
    }
  }
  buoy_list_->blockSignals(false);
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::StaticBuoyPanel, rviz_common::Panel)
