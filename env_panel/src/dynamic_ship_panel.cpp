#include "env_panel/dynamic_ship_panel.hpp"

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace env_panel
{

DynamicShipPanel::DynamicShipPanel(QWidget * parent)
: rviz_common::Panel(parent),
  heading_deg_(0.0),
  speed_(3.0),
  half_distance_(50.0),
  shape_("mesh_profile")
{
  setupUi();
}

void DynamicShipPanel::setupUi()
{
  auto main_layout = new QVBoxLayout;

  auto config_group = new QGroupBox("Spawn Config");
  auto form = new QFormLayout;

  heading_slider_ = new QSlider(Qt::Horizontal);
  heading_slider_->setRange(0, 359);
  heading_slider_->setValue(static_cast<int>(heading_deg_));
  heading_label_ = new QLabel("0 deg");
  auto heading_row = new QHBoxLayout;
  heading_row->addWidget(heading_slider_);
  heading_row->addWidget(heading_label_);
  form->addRow("Heading:", heading_row);
  QObject::connect(heading_slider_, &QSlider::valueChanged,
                   this, &DynamicShipPanel::onHeadingChanged);

  speed_slider_ = new QSlider(Qt::Horizontal);
  speed_slider_->setRange(5, 200);
  speed_slider_->setValue(static_cast<int>(speed_ * 10.0));
  speed_label_ = new QLabel("3.0 m/s");
  auto speed_row = new QHBoxLayout;
  speed_row->addWidget(speed_slider_);
  speed_row->addWidget(speed_label_);
  form->addRow("Speed:", speed_row);
  QObject::connect(speed_slider_, &QSlider::valueChanged,
                   this, &DynamicShipPanel::onSpeedChanged);

  half_dist_slider_ = new QSlider(Qt::Horizontal);
  half_dist_slider_->setRange(10, 5000);
  half_dist_slider_->setValue(static_cast<int>(half_distance_ * 10.0));
  half_dist_label_ = new QLabel("50.0 m");
  auto hd_row = new QHBoxLayout;
  hd_row->addWidget(half_dist_slider_);
  hd_row->addWidget(half_dist_label_);
  form->addRow("Half Dist:", hd_row);
  QObject::connect(half_dist_slider_, &QSlider::valueChanged,
                   this, &DynamicShipPanel::onHalfDistChanged);

  shape_combo_ = new QComboBox;
  shape_combo_->addItem("mesh_profile");
  shape_combo_->addItem("box");
  shape_combo_->addItem("cylinder");
  form->addRow("Shape:", shape_combo_);
  QObject::connect(shape_combo_,
                   QOverload<const QString &>::of(&QComboBox::currentTextChanged),
                   this, &DynamicShipPanel::onShapeChanged);

  config_group->setLayout(form);
  main_layout->addWidget(config_group);

  auto list_group = new QGroupBox("Active Ships");
  auto list_layout = new QVBoxLayout;
  ship_list_ = new QListWidget;
  list_layout->addWidget(ship_list_);

  auto btn_layout = new QHBoxLayout;
  auto delete_btn = new QPushButton("Delete Selected");
  QObject::connect(delete_btn, &QPushButton::clicked,
                   this, &DynamicShipPanel::onDeleteClicked);
  btn_layout->addWidget(delete_btn);

  clear_all_button_ = new QPushButton("Clear All");
  QObject::connect(clear_all_button_, &QPushButton::clicked,
                   this, &DynamicShipPanel::onClearAllClicked);
  btn_layout->addWidget(clear_all_button_);

  list_layout->addLayout(btn_layout);
  list_group->setLayout(list_layout);
  main_layout->addWidget(list_group);

  auto hint = new QLabel(
      "Use RViz 'Publish Point' tool to click in 3D view.\n"
      "A target ship will spawn at the clicked position\n"
      "and move back and forth, and publishes\n"
      "TrackedShipList msg.");
  hint->setWordWrap(true);
  main_layout->addWidget(hint);

  main_layout->setContentsMargins(8, 8, 8, 8);
  setLayout(main_layout);

  config_timer_ = new QTimer(this);
  config_timer_->setInterval(200);
  QObject::connect(config_timer_, &QTimer::timeout,
                   this, &DynamicShipPanel::publishConfig);
}

void DynamicShipPanel::onInitialize()
{
  auto raw_node =
      getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "--remap",
       "__node:=rviz_dynamic_ship_config_panel", "--"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  delete_client_ =
      client_node_->create_client<usv_interfaces::srv::DeleteDynamicShip>(
          "/dynamic_ship/delete");
  clear_client_ =
      client_node_->create_client<usv_interfaces::srv::ClearDynamicShips>(
          "/dynamic_ship/clear");
  config_client_ =
      client_node_->create_client<usv_interfaces::srv::SetDynamicShipConfig>(
          "/dynamic_ship/set_config");

  names_sub_ = client_node_->create_subscription<std_msgs::msg::String>(
      "/dynamic_ship/names", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        onNamesReceived(msg);
      });

  (void)raw_node;
  config_timer_->start();
}

void DynamicShipPanel::onHeadingChanged(int value)
{
  heading_deg_ = static_cast<double>(value);
  heading_label_->setText(
      QString("%1 deg").arg(static_cast<int>(heading_deg_)));
}

void DynamicShipPanel::onSpeedChanged(int value)
{
  speed_ = static_cast<double>(value) / 10.0;
  speed_label_->setText(
      QString("%1 m/s").arg(speed_, 0, 'f', 1));
}

void DynamicShipPanel::onHalfDistChanged(int value)
{
  half_distance_ = static_cast<double>(value) / 10.0;
  half_dist_label_->setText(
      QString("%1 m").arg(half_distance_, 0, 'f', 1));
}

void DynamicShipPanel::onShapeChanged(const QString & text)
{
  shape_ = text.toStdString();
}

void DynamicShipPanel::publishConfig()
{
  if (!config_client_->wait_for_service(std::chrono::milliseconds(100))) {
    return;
  }

  static double last_heading = -1, last_speed = -1,
                last_hd = -1;
  static std::string last_shape;

  if (last_heading == heading_deg_ && last_speed == speed_ &&
      last_hd == half_distance_ && last_shape == shape_) {
    return;
  }

  last_heading = heading_deg_;
  last_speed = speed_;
  last_hd = half_distance_;
  last_shape = shape_;

  auto req =
      std::make_shared<usv_interfaces::srv::SetDynamicShipConfig::Request>();
  req->heading_deg = heading_deg_;
  req->speed = speed_;
  req->shape = shape_;
  req->half_distance = half_distance_;
  config_client_->async_send_request(req);
}

void DynamicShipPanel::onDeleteClicked()
{
  auto selected = ship_list_->currentItem();
  if (!selected) return;

  if (!delete_client_->wait_for_service(std::chrono::seconds(1))) return;

  auto req =
      std::make_shared<usv_interfaces::srv::DeleteDynamicShip::Request>();
  req->model_name = selected->text().toStdString();
  delete_client_->async_send_request(req);
}

void DynamicShipPanel::onClearAllClicked()
{
  if (!clear_client_->wait_for_service(std::chrono::seconds(1))) return;

  auto req =
      std::make_shared<usv_interfaces::srv::ClearDynamicShips::Request>();
  clear_client_->async_send_request(req);
}

void DynamicShipPanel::onNamesReceived(
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

  if (new_names != ship_names_) {
    ship_names_ = new_names;
    ship_list_->clear();
    for (const auto & name : ship_names_) {
      ship_list_->addItem(QString::fromStdString(name));
    }
  }
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::DynamicShipPanel, rviz_common::Panel)
