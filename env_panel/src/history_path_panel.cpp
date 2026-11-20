#include <chrono>
#include <cstdint>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace env_panel
{
// A settings client only: trajectory generation and clearing remain in history_path.py.
class HistoryPathPanel : public rviz_common::Panel
{
public:
  explicit HistoryPathPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent)
  {
    duration_ = new QDoubleSpinBox(this);
    duration_->setObjectName("history_duration");
    duration_->setRange(0.1, 1e9);
    duration_->setDecimals(1);
    duration_->setSuffix(" 秒");
    duration_->setEnabled(false);
    apply_ = new QPushButton("应用", this);
    apply_->setObjectName("history_apply");
    refresh_ = new QPushButton("读取当前值", this);
    refresh_->setObjectName("history_refresh");
    apply_->setEnabled(false);
    refresh_->setEnabled(false);
    status_ = new QLabel("等待历史航迹节点", this);
    status_->setObjectName("history_status");
    status_->setWordWrap(true);
    auto * form = new QFormLayout;
    form->addRow("历史保留时长", duration_);
    auto * buttons = new QHBoxLayout;
    buttons->addWidget(apply_);
    buttons->addWidget(refresh_);
    auto * layout = new QVBoxLayout(this);
    layout->addLayout(form);
    layout->addLayout(buttons);
    layout->addWidget(status_);
    layout->addWidget(new QLabel("新规划任务自动清空上一段航迹。", this));
    layout->addStretch();
    connect(apply_, &QPushButton::clicked, this, [this]() {apply();});
    connect(refresh_, &QPushButton::clicked, this, [this]() {refresh();});
    connect(&timer_, &QTimer::timeout, this, [this]() {tick();});
  }

  void onInitialize() override
  {
    // Use the RViz ROS context, but avoid inheriting its node-name/namespace remaps.
    node_ = std::make_shared<rclcpp::Node>(
      "history_settings_" + std::to_string(reinterpret_cast<uintptr_t>(this)),
      rclcpp::NodeOptions().use_global_arguments(false));
    get_client_ = node_->create_client<Get>("/history_path/get_parameters");
    set_client_ = node_->create_client<Set>("/history_path/set_parameters");
    timer_.start(100);
  }

private:
  using Get = rcl_interfaces::srv::GetParameters;
  using Set = rcl_interfaces::srv::SetParameters;

  void refresh()
  {
    auto request = std::make_shared<Get::Request>();
    request->names = {"history_duration_sec"};
    auto pending = get_client_->async_send_request(request);
    request_id_ = pending.request_id;
    get_future_ = pending.future.share();
    elapsed_.restart();
    apply_->setEnabled(false);
    refresh_->setEnabled(false);
    status_->setText("正在读取…");
  }

  void apply()
  {
    auto request = std::make_shared<Set::Request>();
    request->parameters = {
      rclcpp::Parameter("history_duration_sec", duration_->value()).to_parameter_msg()};
    auto pending = set_client_->async_send_request(request);
    request_id_ = pending.request_id;
    set_future_ = pending.future.share();
    elapsed_.restart();
    apply_->setEnabled(false);
    refresh_->setEnabled(false);
    status_->setText("正在应用…");
  }

  void tick()
  {
    // ROS responses are handled on the Qt thread; no worker thread or blocking service call.
    rclcpp::spin_some(node_);
    using namespace std::chrono_literals;
    if (get_future_.valid()) {
      if (get_future_.wait_for(0s) == std::future_status::ready) {
        const auto values = get_future_.get()->values;
        loaded_ = !values.empty() &&
          values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        if (loaded_) {
          duration_->setValue(values[0].double_value);
        }
        status_->setText(loaded_ ? "已读取当前保留时长" : "未找到历史保留时长参数");
        get_future_ = {};
      } else if (elapsed_.elapsed() > 3000) {
        get_client_->remove_pending_request(request_id_);
        get_future_ = {};
        status_->setText("读取超时，请重试");
      }
    } else if (set_future_.valid()) {
      if (set_future_.wait_for(0s) == std::future_status::ready) {
        const auto results = set_future_.get()->results;
        const bool success = !results.empty() && results[0].successful;
        status_->setText(success ? "已应用" : "应用失败，请读取当前值后重试");
        set_future_ = {};
      } else if (elapsed_.elapsed() > 3000) {
        set_client_->remove_pending_request(request_id_);
        set_future_ = {};
        status_->setText("应用超时，请读取当前值确认");
      }
    }
    const bool ready = get_client_->service_is_ready() && set_client_->service_is_ready();
    const bool idle = !get_future_.valid() && !set_future_.valid();
    duration_->setEnabled(ready && loaded_);
    apply_->setEnabled(ready && idle && loaded_);
    refresh_->setEnabled(ready && idle);
    if (!ready && idle) {
      loaded_ = false;
      status_->setText("等待历史航迹节点");
    } else if (ready && idle && !loaded_) {
      refresh();
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Get>::SharedPtr get_client_;
  rclcpp::Client<Set>::SharedPtr set_client_;
  rclcpp::Client<Get>::SharedFuture get_future_;
  rclcpp::Client<Set>::SharedFuture set_future_;
  int64_t request_id_{0};
  QElapsedTimer elapsed_;
  QTimer timer_;
  bool loaded_{false};
  QDoubleSpinBox * duration_;
  QPushButton * apply_;
  QPushButton * refresh_;
  QLabel * status_;
};
}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::HistoryPathPanel, rviz_common::Panel)
