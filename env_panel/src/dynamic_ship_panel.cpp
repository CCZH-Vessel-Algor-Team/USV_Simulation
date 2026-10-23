#include "env_panel/dynamic_ship_panel.hpp"

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace env_panel
{

DynamicShipPanel::DynamicShipPanel(QWidget * parent)
: rviz_common::Panel(parent),
  heading_deg_(0.0),
  speed_(3.0),
  half_distance_(100.0),
  shape_("mesh_profile")
{
  setupUi();
}

void DynamicShipPanel::setupUi()
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

  auto config_group = new QGroupBox("生成配置");
  auto form = new QFormLayout;
  form->setContentsMargins(12, group_content_top_margin, 12, 10);
  form->setHorizontalSpacing(14);
  form->setVerticalSpacing(8);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

  heading_slider_ = new QSlider(Qt::Horizontal);
  heading_slider_->setRange(0, 359);
  heading_slider_->setValue(static_cast<int>(heading_deg_));
  heading_label_ = new QLabel("0 deg");
  auto heading_row = new QHBoxLayout;
  configure_slider_row(heading_row);
  heading_row->addWidget(heading_slider_, 1);
  heading_row->addWidget(heading_label_);
  form->addRow("航向：", heading_row);
  QObject::connect(heading_slider_, &QSlider::valueChanged,
                   this, &DynamicShipPanel::onHeadingChanged);

  speed_slider_ = new QSlider(Qt::Horizontal);
  speed_slider_->setRange(5, 200);
  speed_slider_->setValue(static_cast<int>(speed_ * 10.0));
  speed_label_ = new QLabel("3.0 m/s");
  auto speed_row = new QHBoxLayout;
  configure_slider_row(speed_row);
  speed_row->addWidget(speed_slider_, 1);
  speed_row->addWidget(speed_label_);
  form->addRow("速度：", speed_row);
  QObject::connect(speed_slider_, &QSlider::valueChanged,
                   this, &DynamicShipPanel::onSpeedChanged);

  // 界面显示单程行程长度（A→B），服务接口字段为 half_distance（单程的一半）
  leg_dist_slider_ = new QSlider(Qt::Horizontal);
  leg_dist_slider_->setRange(20, 10000);  // 单程 2 m ~ 1000 m
  leg_dist_slider_->setValue(
      static_cast<int>(half_distance_ * 2.0 * 10.0));
  leg_dist_label_ = new QLabel("200.0 m");
  auto hd_row = new QHBoxLayout;
  configure_slider_row(hd_row);
  hd_row->addWidget(leg_dist_slider_, 1);
  hd_row->addWidget(leg_dist_label_);
  form->addRow("单程距离：", hd_row);
  QObject::connect(leg_dist_slider_, &QSlider::valueChanged,
                   this, &DynamicShipPanel::onLegDistChanged);

  shape_combo_ = new QComboBox;
  shape_combo_->addItem("mesh_profile");
  shape_combo_->addItem("box");
  shape_combo_->addItem("cylinder");
  form->addRow("形状：", shape_combo_);
  QObject::connect(shape_combo_,
                   QOverload<const QString &>::of(&QComboBox::currentTextChanged),
                   this, &DynamicShipPanel::onShapeChanged);

  config_group->setLayout(form);
  main_layout->addWidget(config_group);

  auto list_group = new QGroupBox("目标船列表");
  auto list_layout = new QVBoxLayout;
  list_layout->setContentsMargins(12, group_content_top_margin, 12, 10);
  list_layout->setSpacing(8);
  ship_list_ = new QListWidget;
  list_layout->addWidget(ship_list_);

  auto btn_layout = new QHBoxLayout;
  btn_layout->setContentsMargins(0, 0, 0, 0);
  btn_layout->setSpacing(10);
  auto delete_btn = new QPushButton("删除所选");
  QObject::connect(delete_btn, &QPushButton::clicked,
                   this, &DynamicShipPanel::onDeleteClicked);
  btn_layout->addWidget(delete_btn);

  clear_all_button_ = new QPushButton("全部清除");
  QObject::connect(clear_all_button_, &QPushButton::clicked,
                   this, &DynamicShipPanel::onClearAllClicked);
  btn_layout->addWidget(clear_all_button_);

  list_layout->addLayout(btn_layout);
  list_group->setLayout(list_layout);
  main_layout->addWidget(list_group);

  for (auto * label : {heading_label_, speed_label_, leg_dist_label_}) {
    label->setMinimumWidth(76);
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  }
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

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(client_node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });
}

DynamicShipPanel::~DynamicShipPanel()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
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

void DynamicShipPanel::onLegDistChanged(int value)
{
  const double leg_distance = static_cast<double>(value) / 10.0;
  half_distance_ = leg_distance / 2.0;
  leg_dist_label_->setText(
      QString("%1 m").arg(leg_distance, 0, 'f', 1));
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
    auto names_copy = new_names;
    QMetaObject::invokeMethod(
        this,
        [this, names_copy]() {
          ship_list_->clear();
          for (const auto & name : names_copy) {
            ship_list_->addItem(QString::fromStdString(name));
          }
        },
        Qt::QueuedConnection);
  }
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::DynamicShipPanel, rviz_common::Panel)
