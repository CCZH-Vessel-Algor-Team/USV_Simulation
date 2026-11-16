#include "env_panel/dynamic_ship_panel.hpp"
#include "env_panel/target_list.hpp"

#include <QButtonGroup>
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
    "QPushButton { min-height: 28px; padding: 3px 10px; }"
    "QPushButton:checked { background-color: #d8eaff; border: 1px solid #6f9dcc; }");

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

  auto sim_ship_button = new QPushButton("仿真船");
  sim_ship_button->setObjectName("sim_ship_mode");
  sim_ship_button->setToolTip("在仿真环境中创建船舶模型，仅影响后续添加。");
  virtual_ship_button_ = new QPushButton("虚拟船");
  virtual_ship_button_->setObjectName("virtual_ship_mode");
  virtual_ship_button_->setToolTip("只生成目标航迹，不创建船舶模型，仅影响后续添加。");
  auto mode_group = new QButtonGroup(this);
  auto mode_row = new QHBoxLayout;
  configure_slider_row(mode_row);
  for (auto button : {sim_ship_button, virtual_ship_button_}) {
    button->setCheckable(true);
    mode_group->addButton(button);
    mode_row->addWidget(button, 1);
  }
  sim_ship_button->setChecked(true);
  form->addRow("船类型：", mode_row);
  QObject::connect(virtual_ship_button_, &QPushButton::toggled,
    this, [this](bool checked) {
      if (getDisplayContext()) {
        getDisplayContext()->setProperty("env_panel.virtual_ship", checked);
      }
    });

  auto heading_slider = new QSlider(Qt::Horizontal);
  heading_slider->setRange(0, 359);
  heading_slider->setValue(static_cast<int>(heading_deg_));
  heading_label_ = new QLabel("0 deg");
  auto heading_row = new QHBoxLayout;
  configure_slider_row(heading_row);
  heading_row->addWidget(heading_slider, 1);
  heading_row->addWidget(heading_label_);
  heading_slider->setToolTip("北向为 0°，顺时针增加；单击添加使用此值，拖拽添加使用箭头方向。");
  form->addRow("默认航向：", heading_row);
  QObject::connect(heading_slider, &QSlider::valueChanged,
                   this, &DynamicShipPanel::onHeadingChanged);

  auto speed_slider = new QSlider(Qt::Horizontal);
  speed_slider->setRange(5, 200);
  speed_slider->setValue(static_cast<int>(speed_ * 10.0));
  speed_label_ = new QLabel("3.0 m/s");
  auto speed_row = new QHBoxLayout;
  configure_slider_row(speed_row);
  speed_row->addWidget(speed_slider, 1);
  speed_row->addWidget(speed_label_);
  form->addRow("速度：", speed_row);
  QObject::connect(speed_slider, &QSlider::valueChanged,
                   this, &DynamicShipPanel::onSpeedChanged);

  // 界面显示单程行程长度（A→B），服务接口字段为 half_distance（单程的一半）
  auto leg_dist_slider = new QSlider(Qt::Horizontal);
  leg_dist_slider->setRange(20, 10000);  // 单程 2 m ~ 1000 m
  leg_dist_slider->setValue(
      static_cast<int>(half_distance_ * 2.0 * 10.0));
  leg_dist_label_ = new QLabel("200.0 m");
  auto hd_row = new QHBoxLayout;
  configure_slider_row(hd_row);
  hd_row->addWidget(leg_dist_slider, 1);
  hd_row->addWidget(leg_dist_label_);
  form->addRow("单程距离：", hd_row);
  QObject::connect(leg_dist_slider, &QSlider::valueChanged,
                   this, &DynamicShipPanel::onLegDistChanged);

  auto shape_combo = new QComboBox;
  shape_combo->addItem("mesh_profile");
  shape_combo->addItem("box");
  shape_combo->addItem("cylinder");
  form->addRow("形状：", shape_combo);
  QObject::connect(shape_combo,
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

  auto clear_all_button = new QPushButton("全部清除");
  QObject::connect(clear_all_button, &QPushButton::clicked,
                   this, &DynamicShipPanel::onClearAllClicked);
  btn_layout->addWidget(clear_all_button);

  list_layout->addLayout(btn_layout);
  list_group->setLayout(list_layout);
  main_layout->addWidget(list_group);

  auto placement_hint = new QLabel(
    "备注：选择船类型后，使用工具栏“添加动态船”；单击使用默认航向，"
    "拖拽指定船艏方向。右键取消，Esc 退出。");
  placement_hint->setObjectName("placement_note");
  placement_hint->setWordWrap(true);
  main_layout->addWidget(placement_hint);

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
  getDisplayContext()->setProperty(
    "env_panel.virtual_ship", virtual_ship_button_->isChecked());

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
        updateTargetNames(ship_list_, msg->data);
      });

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
  if (!config_client_->service_is_ready()) {
    last_config_.reset();
    return;
  }

  auto req =
      std::make_shared<usv_interfaces::srv::SetDynamicShipConfig::Request>();
  req->heading_deg = heading_deg_;
  req->speed = speed_;
  req->shape = shape_;
  req->half_distance = half_distance_;
  if (last_config_ && *last_config_ == *req) {
    return;
  }
  last_config_ = req;
  config_client_->async_send_request(req,
    [this, req](rclcpp::Client<usv_interfaces::srv::SetDynamicShipConfig>::SharedFuture future) {
      if (!future.get()->success) {
        QMetaObject::invokeMethod(this, [this, req]() {
          if (last_config_ == req) {last_config_.reset();}
        }, Qt::QueuedConnection);
      }
    });
}

void DynamicShipPanel::onDeleteClicked()
{
  auto selected = ship_list_->currentItem();
  if (!selected) return;

  if (!delete_client_->service_is_ready()) return;

  auto req =
      std::make_shared<usv_interfaces::srv::DeleteDynamicShip::Request>();
  req->model_name = selected->text().toStdString();
  delete_client_->async_send_request(req);
}

void DynamicShipPanel::onClearAllClicked()
{
  if (!clear_client_->service_is_ready()) return;

  auto req =
      std::make_shared<usv_interfaces::srv::ClearDynamicShips::Request>();
  clear_client_->async_send_request(req);
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::DynamicShipPanel, rviz_common::Panel)
