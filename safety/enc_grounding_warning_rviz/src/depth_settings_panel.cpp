#include "enc_grounding_warning_rviz/depth_settings_panel.hpp"

#include <cmath>
#include <set>

#include "enc_grounding_warning_rviz/depth_point_tool.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/tool_manager.hpp"

namespace enc_grounding_warning_rviz
{
namespace
{
QDoubleSpinBox * numberEdit(const char * name, double low, double high, double value)
{
  auto * edit = new QDoubleSpinBox;
  edit->setObjectName(name);
  edit->setRange(low, high);
  edit->setDecimals(3);
  edit->setValue(value);
  return edit;
}
}  // namespace

DepthSettingsPanel::DepthSettingsPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(2, 2, 2, 2);
  auto * ns_row = new QHBoxLayout;
  ns_row->addWidget(new QLabel("Namespace:"));
  ns_edit_ = new QLineEdit(QString::fromStdString(ns_));
  ns_edit_->setMinimumWidth(0);
  ns_row->addWidget(ns_edit_, 1);
  root->addLayout(ns_row);
  connect(ns_edit_, &QLineEdit::editingFinished, this, &DepthSettingsPanel::onApplyNamespace);

  auto * form = new QFormLayout;
  radius_edit_ = numberEdit("depthRadius", 0.001, 1.0e9, 10.0);
  depth_edit_ = numberEdit("regionDepth", 0.0, 11000.0, 0.5);
  const auto add_field = [form](const QString & label, QDoubleSpinBox * edit) {
    auto * row = new QHBoxLayout;
    row->addWidget(edit, 1);
    row->addWidget(new QLabel("米"));
    form->addRow(label, row);
  };
  add_field("区域半径", radius_edit_);
  add_field("区域水深", depth_edit_);
  root->addLayout(form);
  root->addWidget(new QLabel("已添加区域（删除后立即恢复）", this));
  regions_table_ = new QTableWidget(0, 3, this);
  regions_table_->setObjectName("depthRegions");
  regions_table_->setHorizontalHeaderLabels({"区域", "半径（米）", "水深（米）"});
  regions_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  regions_table_->horizontalHeader()->setMinimumSectionSize(55);
  regions_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  regions_table_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  regions_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  regions_table_->setMinimumHeight(100);
  root->addWidget(regions_table_, 1);
  remove_region_button_ = new QPushButton("删除选中区域");
  remove_region_button_->setObjectName("removeDepthRegion");
  remove_all_button_ = new QPushButton("删除全部区域");
  remove_all_button_->setObjectName("removeAllDepthRegions");
  remove_all_button_->setToolTip("清空当前船舶的全部自定义水深区域，恢复原始水深。");
  auto * remove_buttons = new QHBoxLayout;
  remove_buttons->addWidget(remove_region_button_);
  remove_buttons->addWidget(remove_all_button_);
  root->addLayout(remove_buttons);
  connect(remove_region_button_, &QPushButton::clicked, this, &DepthSettingsPanel::onRemoveRegion);
  connect(remove_all_button_, &QPushButton::clicked, this, [this]() {
    regions_table_->selectAll();
    onRemoveRegion();
  });
  connect(regions_table_, &QTableWidget::itemSelectionChanged, this, &DepthSettingsPanel::updateControls);
  status_label_ = new QLabel("等待水深设置服务…", this);
  status_label_->setObjectName("depthSettingsStatus");
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);
  updateControls();
}

DepthSettingsPanel::~DepthSettingsPanel()
{
  if (timer_) {timer_->stop();}
  // QWidget teardown can emit editingFinished/selection signals after our
  // node and other C++ members have been destroyed. Disconnect them first.
  QObject::disconnect(ns_edit_, nullptr, this, nullptr);
  QObject::disconnect(regions_table_, nullptr, this, nullptr);
}

void DepthSettingsPanel::onInitialize()
{
  if (timer_) {timer_->stop(); delete timer_;}
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "--remap", "__node:=rviz_depth_settings_panel", "--"});
  node_ = std::make_shared<rclcpp::Node>("_", options);
  updateConnections();
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() {
    rclcpp::spin_some(node_);
    const auto now = std::chrono::steady_clock::now();
    if (request_pending_ && now >= request_deadline_) {
      client_->remove_pending_request(request_id_);
      if (pending_operation_ == Operation::Add && !pending_regions_.empty()) {
        pending_regions_.pop_front();
      }
      request_pending_ = false;
      settings_loaded_ = false;
      query_after_ = now + std::chrono::seconds(2);
      status_label_->setText("操作超时，正在核实已应用的区域…");
    }
    if (!request_pending_ && !settings_loaded_ && now >= query_after_) {
      query_after_ = now + std::chrono::seconds(2);
      requestSettings(Operation::Read);
    }
    processNextPoint();
    updateControls();
  });
  timer_->start(50);
}

void DepthSettingsPanel::onApplyNamespace()
{
  auto ns = ns_edit_->text().trimmed().toStdString();
  while (!ns.empty() && ns.front() == '/') {ns.erase(0, 1);}
  while (!ns.empty() && ns.back() == '/') {ns.pop_back();}
  if (ns.empty()) {ns = "usv_1";}
  ns_edit_->setText(QString::fromStdString(ns));
  if (ns == ns_) {return;}
  ns_ = ns;
  updateConnections();
  Q_EMIT configChanged();
}

void DepthSettingsPanel::updateConnections()
{
  if (!node_) {return;}
  ++generation_;
  if (client_ && request_pending_) {client_->remove_pending_request(request_id_);}
  request_pending_ = false;
  settings_loaded_ = false;
  pending_regions_.clear();
  query_after_ = std::chrono::steady_clock::now();
  client_ = node_->create_client<Service>("/" + ns_ + "/safety/configure_depth");
  const auto generation = generation_;
  settings_sub_ = node_->create_subscription<Settings>(
    "/" + ns_ + "/safety/depth_settings", rclcpp::QoS(1).reliable().transient_local(),
    [this, generation](Settings::ConstSharedPtr message) {
      if (generation == generation_ && !request_pending_) {
        showSettings(*message);
        status_label_->setText(current_settings_.shoals.empty() ? "暂无自定义区域，使用原始水深。" :
          QString("当前 %1 个区域，已应用并自动保存。").arg(current_settings_.shoals.size()));
        updateControls();
      }
    });
  point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/" + ns_ + "/safety/depth_point", 10,
    [this, generation](geometry_msgs::msg::PointStamped::ConstSharedPtr message) {
      if (generation == generation_) {receivePoint(*message);}
    });
  regions_table_->setRowCount(0);
  status_label_->setText("等待水深设置服务…");
  bindPointTool();
  updateControls();
}

void DepthSettingsPanel::bindPointTool()
{
  auto * context = getDisplayContext();
  if (!context || !context->getToolManager()) {return;}
  auto * manager = context->getToolManager();
  DepthPointTool * tool = nullptr;
  for (int i = 0; i < manager->numTools(); ++i) {
    tool = dynamic_cast<DepthPointTool *>(manager->getTool(i));
    if (tool) {break;}
  }
  if (!tool) {
    tool = dynamic_cast<DepthPointTool *>(manager->addTool("enc_grounding_warning_rviz/DepthPointTool"));
  }
  if (tool) {tool->setTopic("/" + ns_ + "/safety/depth_point");}
}

void DepthSettingsPanel::receivePoint(const geometry_msgs::msg::PointStamped & point)
{
  if (!settings_loaded_ || !client_->service_is_ready()) {
    status_label_->setText("水深服务未就绪，请稍后重新打点。");
    return;
  }
  if (point.header.frame_id != "map" || !std::isfinite(point.point.x) ||
    !std::isfinite(point.point.y) || std::abs(point.point.x) > 1.0e9 || std::abs(point.point.y) > 1.0e9)
  {
    status_label_->setText("取点无效，请使用 map 坐标系中的有效点。");
    return;
  }
  if (current_settings_.shoals.size() + pending_regions_.size() >= 128) {
    status_label_->setText("最多支持 128 个区域，请删除部分区域后再打点。");
    return;
  }
  enc_grounding_warning_msgs::msg::DepthShoal region;
  region.x = point.point.x;
  region.y = point.point.y;
  region.radius = radius_edit_->value();
  region.depth_m = depth_edit_->value();
  pending_regions_.push_back(region);
  processNextPoint();
}

void DepthSettingsPanel::processNextPoint()
{
  if (request_pending_ || !settings_loaded_ || pending_regions_.empty() ||
    !client_->service_is_ready()) {return;}
  auto settings = current_settings_;
  settings.enabled = true;
  settings.shoals.push_back(pending_regions_.front());
  requestSettings(Operation::Add, settings);
}

void DepthSettingsPanel::onRemoveRegion()
{
  if (request_pending_ || !settings_loaded_ || !pending_regions_.empty()) {return;}
  std::set<int> rows;
  for (const auto & index : regions_table_->selectionModel()->selectedRows()) {rows.insert(index.row());}
  if (rows.empty()) {return;}
  auto settings = current_settings_;
  settings.shoals.clear();
  for (size_t i = 0; i < current_settings_.shoals.size(); ++i) {
    if (rows.count(static_cast<int>(i)) == 0) {settings.shoals.push_back(current_settings_.shoals[i]);}
  }
  settings.enabled = !settings.shoals.empty();
  requestSettings(Operation::Remove, settings);
}

void DepthSettingsPanel::updateControls()
{
  const bool ready = client_ && client_->service_is_ready() && settings_loaded_;
  radius_edit_->setEnabled(ready);
  depth_edit_->setEnabled(ready);
  regions_table_->setEnabled(ready && !request_pending_ && pending_regions_.empty());
  remove_region_button_->setEnabled(regions_table_->isEnabled() &&
    !regions_table_->selectionModel()->selectedRows().empty());
  remove_all_button_->setEnabled(regions_table_->isEnabled() && !current_settings_.shoals.empty());
}

void DepthSettingsPanel::showSettings(const Settings & settings)
{
  QSignalBlocker block(regions_table_);
  current_settings_ = settings;
  if (!current_settings_.enabled) {current_settings_.shoals.clear();}
  regions_table_->setRowCount(0);
  for (const auto & region : current_settings_.shoals) {
    const int row = regions_table_->rowCount();
    regions_table_->insertRow(row);
    regions_table_->setItem(row, 0, new QTableWidgetItem(QString("区域 %1").arg(row + 1)));
    regions_table_->setItem(row, 1, new QTableWidgetItem(QString::number(region.radius, 'f', 3)));
    regions_table_->setItem(row, 2, new QTableWidgetItem(QString::number(region.depth_m, 'f', 3)));
  }
  settings_loaded_ = true;
}

void DepthSettingsPanel::requestSettings(Operation operation, const Settings & settings)
{
  if (!client_ || !client_->service_is_ready() || request_pending_) {return;}
  auto request = std::make_shared<Service::Request>();
  request->apply = operation != Operation::Read;
  request->settings = settings;
  request_pending_ = true;
  pending_operation_ = operation;
  request_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  status_label_->setText(operation == Operation::Read ? "正在读取已添加区域…" : "正在应用并保存…");
  status_label_->setStyleSheet("");
  updateControls();
  const auto generation = generation_;
  const auto pending = client_->async_send_request(request,
    [this, generation, operation](rclcpp::Client<Service>::SharedFuture future) {
      if (generation != generation_) {return;}
      request_pending_ = false;
      if (operation == Operation::Add && !pending_regions_.empty()) {pending_regions_.pop_front();}
      try {
        const auto response = future.get();
        if (response->success) {
          showSettings(response->settings);
          status_label_->setText(current_settings_.shoals.empty() ? "暂无自定义区域，使用原始水深。" :
            QString("当前 %1 个区域，已应用并自动保存。").arg(current_settings_.shoals.size()));
        } else {
          status_label_->setText(QString::fromStdString(response->message));
          status_label_->setStyleSheet("color: red;");
        }
      } catch (const std::exception & error) {
        status_label_->setText(QString("操作失败：%1").arg(error.what()));
        settings_loaded_ = false;
      }
      updateControls();
    });
  request_id_ = pending.request_id;
}

void DepthSettingsPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString ns;
  if (config.mapGetString("Namespace", &ns)) {ns_edit_->setText(ns); onApplyNamespace();}
}

void DepthSettingsPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Namespace", QString::fromStdString(ns_));
}

}  // namespace enc_grounding_warning_rviz

PLUGINLIB_EXPORT_CLASS(enc_grounding_warning_rviz::DepthSettingsPanel, rviz_common::Panel)
