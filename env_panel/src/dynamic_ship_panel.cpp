#include "env_panel/dynamic_ship_panel.hpp"

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QMetaObject>
#include <QRegularExpression>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <set>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace env_panel
{

DynamicShipPanel::DynamicShipPanel(QWidget * parent)
: rviz_common::Panel(parent),
  heading_deg_(0.0),
  speed_(3.0),
  half_distance_(100.0)
{
  elapsed_.start();
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
  speed_slider_->setRange(0, 200);
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

  model_combo_ = new QComboBox;
  model_combo_->addItem("等待后端船型目录…");
  form->addRow("船型：", model_combo_);
  QObject::connect(model_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
                   this, &DynamicShipPanel::onModelChanged);
  model_summary_ = new QLabel;
  model_summary_->setWordWrap(true);
  form->addRow(model_summary_);
  config_status_ = new QLabel("等待配置服务；确认成功后再点击/拖拽生成");
  config_status_->setWordWrap(true);
  form->addRow(config_status_);

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
  action_status_ = new QLabel;
  action_status_->setWordWrap(true);
  list_layout->addWidget(action_status_);
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
  auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "--remap",
       "__node:=rviz_dynamic_ship_config_panel_" +
         std::to_string(reinterpret_cast<std::uintptr_t>(this)), "--"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  delete_client_ =
      client_node_->create_client<usv_interfaces::srv::DeleteDynamicShip>(
          "/dynamic_ship/delete");
  clear_client_ =
      client_node_->create_client<usv_interfaces::srv::ClearDynamicShips>(
          "/dynamic_ship/clear");
  config_client_ =
      client_node_->create_client<usv_interfaces::srv::SetDynamicShipConfigV2>(
          "/dynamic_ship/set_config_v2");
  models_client_ = client_node_->create_client<usv_interfaces::srv::ListDynamicShipModels>(
      "/dynamic_ship/models");

  names_sub_ = client_node_->create_subscription<std_msgs::msg::String>(
      "/dynamic_ship/names", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        onNamesReceived(msg);
      });
  snapshot_sub_ = client_node_->create_subscription<usv_interfaces::msg::DynamicShipSnapshot>(
      "/dynamic_ship/snapshot", 10,
      [this](usv_interfaces::msg::DynamicShipSnapshot::SharedPtr msg) {
        QMetaObject::invokeMethod(this, [this, msg]() {
          QMap<QString, QString> metadata;
          for (const auto & ship : msg->ships) {
            metadata[QString::fromStdString(ship.model_name)] = QString::fromStdString(ship.model_id);
          }
          if (metadata != instance_models_) {
            instance_models_ = metadata;
            updateShipList();
          }
        }, Qt::QueuedConnection);
      });

  config_timer_->start();

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(client_node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });
}

DynamicShipPanel::~DynamicShipPanel()
{
  config_timer_->stop();
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
  configEdited();
}

void DynamicShipPanel::onSpeedChanged(int value)
{
  speed_ = static_cast<double>(value) / 10.0;
  speed_label_->setText(
      QString("%1 m/s").arg(speed_, 0, 'f', 1));
  configEdited();
}

void DynamicShipPanel::onLegDistChanged(int value)
{
  const double leg_distance = static_cast<double>(value) / 10.0;
  half_distance_ = leg_distance / 2.0;
  leg_dist_label_->setText(
      QString("%1 m").arg(leg_distance, 0, 'f', 1));
  configEdited();
}

void DynamicShipPanel::onModelChanged(int index)
{
  model_id_ = model_combo_->itemData(index).toString().toStdString();
  updateModelSummary();
  configEdited();
}

void DynamicShipPanel::configEdited()
{
  ++revision_;
  next_config_ms_ = 0;
  config_status_->setText("配置待确认；请先等待成功，再点击/拖拽生成");
  Q_EMIT configChanged();
}

void DynamicShipPanel::updateModelSummary()
{
  const auto found = std::find_if(models_.begin(), models_.end(), [this](const auto & model) {
    return model.model_id == model_id_;
  });
  if (found == models_.end()) {
    model_summary_->setText("所选船型不可用：" + QString::fromStdString(model_id_));
    return;
  }
  model_summary_->setText(QString("包络 %1 × %2 × %3 m；吃水 %4 m；建议 %5 m/s")
      .arg(found->length, 0, 'f', 1).arg(found->beam, 0, 'f', 1)
      .arg(found->height, 0, 'f', 1).arg(found->draft, 0, 'f', 1)
      .arg(found->recommended_speed, 0, 'f', 1));
}

void DynamicShipPanel::refreshCatalog()
{
  const auto now = elapsed_.elapsed();
  if (catalog_pending_ && now - catalog_sent_ms_ > 5000) {
    models_client_->remove_pending_request(catalog_request_id_);
    catalog_pending_ = false;
    ++catalog_token_;
    next_catalog_ms_ = now + 1000;
    config_status_->setText("目录查询超时；将自动重试");
  }
  if (catalog_pending_ || now < next_catalog_ms_) return;
  if (!models_client_->service_is_ready()) {
    next_catalog_ms_ = now + 1000;
    config_status_->setText("等待 /dynamic_ship/models；将自动重试");
    return;
  }
  catalog_pending_ = true;
  catalog_sent_ms_ = now;
  const auto token = ++catalog_token_;
  auto request = std::make_shared<usv_interfaces::srv::ListDynamicShipModels::Request>();
  auto pending = models_client_->async_send_request(request,
      [this, token](rclcpp::Client<usv_interfaces::srv::ListDynamicShipModels>::SharedFuture future) {
        auto response = future.get();
        QMetaObject::invokeMethod(this, [this, token, response]() {
          if (token != catalog_token_) return;
          catalog_pending_ = false;
          next_catalog_ms_ = elapsed_.elapsed() + 1000;
          std::set<std::string> ids;
          bool valid = response->success && !response->models.empty();
          for (const auto & model : response->models) {
            valid = valid && !model.model_id.empty() && ids.insert(model.model_id).second &&
              std::isfinite(model.length) && model.length > 0 &&
              std::isfinite(model.beam) && model.beam > 0 &&
              std::isfinite(model.height) && model.height > 0;
          }
          if (!valid) {
            catalog_ready_ = false;
            config_status_->setText("船型目录无效；将自动重试：" + QString::fromStdString(response->message));
            return;
          }
          models_ = response->models;
          const QSignalBlocker blocker(model_combo_);
          model_combo_->clear();
          for (const auto & model : models_) {
            model_combo_->addItem(QString::fromStdString(model.display_name),
                                  QString::fromStdString(model.model_id));
          }
          int index = model_combo_->findData(QString::fromStdString(model_id_));
          if (index < 0) {
            model_combo_->addItem("不可用：" + QString::fromStdString(model_id_),
                                  QString::fromStdString(model_id_));
            index = model_combo_->count() - 1;
          }
          model_combo_->setCurrentIndex(index);
          catalog_ready_ = true;
          next_catalog_ms_ = elapsed_.elapsed() + 10000;
          updateModelSummary();
          updateShipList();
        }, Qt::QueuedConnection);
      });
  catalog_request_id_ = pending.request_id;
}

void DynamicShipPanel::publishConfig()
{
  if (!client_node_) return;
  refreshCatalog();
  const auto now = elapsed_.elapsed();
  if (config_pending_ && now - config_sent_ms_ > 5000) {
    config_client_->remove_pending_request(config_request_id_);
    config_pending_ = false;
    applied_revision_ = 0;
    ++config_token_;
    next_config_ms_ = now + 1000;
    config_status_->setText("配置确认超时；将自动重试，请勿生成");
  }
  if (config_pending_ || !catalog_ready_ || now < next_config_ms_) return;
  if (!config_client_->service_is_ready()) {
    applied_revision_ = 0;
    config_status_->setText("等待 V2 配置服务；将自动重试，请勿生成");
    next_config_ms_ = now + 1000;
    return;
  }
  if (std::none_of(models_.begin(), models_.end(), [this](const auto & model) {
      return model.model_id == model_id_;
    })) {
    config_status_->setText("所选船型不在后端目录中；请重新选择");
    return;
  }
  // A periodic acknowledged refresh also restores the selection after a manager
  // restart. Never mark a configuration applied just because it was sent.
  if (applied_revision_ == revision_ && now - config_sent_ms_ < 5000) return;
  config_pending_ = true;
  config_sent_ms_ = now;
  const auto token = ++config_token_;
  const auto revision = revision_;

  auto req =
      std::make_shared<usv_interfaces::srv::SetDynamicShipConfigV2::Request>();
  req->heading_deg = heading_deg_;
  req->speed = speed_;
  req->model_id = model_id_;
  req->half_distance = half_distance_;
  auto pending = config_client_->async_send_request(req,
      [this, token, revision, id = model_id_](
          rclcpp::Client<usv_interfaces::srv::SetDynamicShipConfigV2>::SharedFuture future) {
        auto response = future.get();
        QMetaObject::invokeMethod(this, [this, token, revision, id, response]() {
          if (token != config_token_) return;
          config_pending_ = false;
          if (response->success && response->model_id == id) {
            applied_revision_ = revision;
            if (revision == revision_) config_status_->setText("配置已确认；可点击/拖拽生成新船");
            next_config_ms_ = 0;
          } else {
            applied_revision_ = 0;
            config_status_->setText("配置失败；自动重试：" + QString::fromStdString(response->message));
            next_config_ms_ = elapsed_.elapsed() + 1000;
          }
        }, Qt::QueuedConnection);
      });
  config_request_id_ = pending.request_id;
}

void DynamicShipPanel::onDeleteClicked()
{
  auto selected = ship_list_->currentItem();
  if (!selected) return;

  if (!delete_client_->service_is_ready()) {
    action_status_->setText("删除服务不可用");
    return;
  }

  auto req =
      std::make_shared<usv_interfaces::srv::DeleteDynamicShip::Request>();
  req->model_name = selected->data(Qt::UserRole).toString().toStdString();
  action_status_->setText("等待删除确认…");
  delete_client_->async_send_request(req,
      [this](rclcpp::Client<usv_interfaces::srv::DeleteDynamicShip>::SharedFuture future) {
        auto response = future.get();
        QMetaObject::invokeMethod(this, [this, response]() {
          action_status_->setText((response->success ? "删除成功：" : "删除失败：") +
                                 QString::fromStdString(response->message));
        }, Qt::QueuedConnection);
      });
}

void DynamicShipPanel::onClearAllClicked()
{
  if (!clear_client_->service_is_ready()) {
    action_status_->setText("清空服务不可用");
    return;
  }

  auto req =
      std::make_shared<usv_interfaces::srv::ClearDynamicShips::Request>();
  action_status_->setText("等待清空确认…");
  clear_client_->async_send_request(req,
      [this](rclcpp::Client<usv_interfaces::srv::ClearDynamicShips>::SharedFuture future) {
        auto response = future.get();
        QMetaObject::invokeMethod(this, [this, response]() {
          action_status_->setText((response->success ? "清空成功：" : "清空未完成：") +
                                 QString::fromStdString(response->message));
        }, Qt::QueuedConnection);
      });
}

void DynamicShipPanel::onNamesReceived(
    const std_msgs::msg::String::SharedPtr msg)
{
  const auto doc = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
  if (!doc.isArray()) return;
  std::vector<std::string> names;
  for (const auto & value : doc.array()) {
    if (!value.isString()) return;
    names.push_back(value.toString().toStdString());
  }
  QMetaObject::invokeMethod(this, [this, names]() {
    if (names != ship_names_) {
      ship_names_ = names;
      updateShipList();
    }
  }, Qt::QueuedConnection);
}

void DynamicShipPanel::updateShipList()
{
  const auto selected = ship_list_->currentItem();
  const auto selected_name = selected ? selected->data(Qt::UserRole).toString() : QString();
  ship_list_->clear();
  for (const auto & name : ship_names_) {
    const auto instance = QString::fromStdString(name);
    const auto generated = QRegularExpression("^dyn_target_([0-9]+)$").match(instance);
    const auto display_instance = generated.hasMatch() ?
      QString("动态船 %1").arg(generated.captured(1)) : instance;
    const auto id = instance_models_.value(instance);
    QString label = id;
    for (const auto & model : models_) {
      if (QString::fromStdString(model.model_id) == id) {
        label = QString::fromStdString(model.display_name);
        break;
      }
    }
    auto item = new QListWidgetItem(display_instance + (label.isEmpty() ? "" : " — " + label), ship_list_);
    item->setData(Qt::UserRole, instance);  // Never send a decorated display name to delete.
    if (instance == selected_name) ship_list_->setCurrentItem(item);
  }
}

void DynamicShipPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("HeadingDeg", heading_deg_);
  config.mapSetValue("Speed", speed_);
  config.mapSetValue("HalfDistance", half_distance_);
  config.mapSetValue("ModelId", QString::fromStdString(model_id_));
}

void DynamicShipPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  float value;
  if (config.mapGetFloat("HeadingDeg", &value) && std::isfinite(value)) {
    heading_slider_->setValue(static_cast<int>(std::clamp(value, 0.f, 359.f)));
  }
  if (config.mapGetFloat("Speed", &value) && std::isfinite(value)) {
    speed_slider_->setValue(static_cast<int>(std::clamp(value, 0.f, 20.f) * 10));
  }
  if (config.mapGetFloat("HalfDistance", &value) && std::isfinite(value)) {
    leg_dist_slider_->setValue(static_cast<int>(std::clamp(value, 1.f, 500.f) * 20));
  }
  QString id;
  if (config.mapGetString("ModelId", &id) && !id.isEmpty()) model_id_ = id.toStdString();
  if (catalog_ready_) {
    const QSignalBlocker blocker(model_combo_);
    auto index = model_combo_->findData(QString::fromStdString(model_id_));
    if (index < 0) {
      model_combo_->addItem("不可用：" + QString::fromStdString(model_id_), QString::fromStdString(model_id_));
      index = model_combo_->count() - 1;
    }
    model_combo_->setCurrentIndex(index);
    updateModelSummary();
  }
  configEdited();
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::DynamicShipPanel, rviz_common::Panel)
