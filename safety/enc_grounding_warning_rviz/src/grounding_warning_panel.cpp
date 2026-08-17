#include "enc_grounding_warning_rviz/grounding_warning_panel.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <limits>
#include <string>

namespace enc_grounding_warning_rviz
{

GroundingWarningPanel::GroundingWarningPanel(QWidget * parent)
: rviz_common::Panel(parent),
  ns_("usv_1"),
  timer_(nullptr)
{
  auto * root = new QVBoxLayout;
  root->setContentsMargins(2, 2, 2, 2);
  root->setSpacing(2);

  auto * ns_row = new QHBoxLayout;
  ns_row->addWidget(new QLabel("Namespace:"));
  ns_edit_ = new QLineEdit(QString::fromStdString(ns_));
  ns_row->addWidget(ns_edit_);
  apply_button_ = new QPushButton("Apply");
  ns_row->addWidget(apply_button_);
  root->addLayout(ns_row);

  auto * ukc_box = new QGroupBox("UKC");
  auto * ukc_layout = new QVBoxLayout;
  ukc_label_ = new QLabel("waiting for /safety/ukc_state ...");
  ukc_label_->setWordWrap(true);
  ukc_layout->addWidget(ukc_label_);
  ukc_box->setLayout(ukc_layout);
  root->addWidget(ukc_box);

  auto * alert_box = new QGroupBox("Grounding Alert");
  auto * alert_layout = new QVBoxLayout;
  alert_label_ = new QLabel("no alert");
  alert_label_->setWordWrap(true);
  alert_layout->addWidget(alert_label_);
  alert_box->setLayout(alert_layout);
  root->addWidget(alert_box);

  auto * grid_box = new QGroupBox("Depth Grid");
  auto * grid_layout = new QVBoxLayout;
  grid_label_ = new QLabel("waiting for /safety/depth_grid ...");
  grid_label_->setWordWrap(true);
  grid_layout->addWidget(grid_label_);
  grid_box->setLayout(grid_layout);
  root->addWidget(grid_box);

  setLayout(root);
  setMaximumHeight(260);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

  connect(apply_button_, &QPushButton::clicked, this, &GroundingWarningPanel::onApplyNamespace);
}

GroundingWarningPanel::~GroundingWarningPanel() = default;

void GroundingWarningPanel::onInitialize()
{
  if (timer_) {
    timer_->stop();
    delete timer_;
    timer_ = nullptr;
  }

  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "--remap", "__node:=rviz_grounding_warning_panel", "--"});
  node_ = std::make_shared<rclcpp::Node>("_", options);
  updateSubscriptions();

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() {
    if (node_) {
      rclcpp::spin_some(node_);
    }
  });
  timer_->start(50);
}

void GroundingWarningPanel::updateSubscriptions()
{
  ukc_sub_.reset();
  alert_sub_.reset();
  grid_sub_.reset();

  const std::string ukc_topic = "/" + ns_ + "/safety/ukc_state";
  const std::string alert_topic = "/" + ns_ + "/safety/grounding_alerts";
  const std::string grid_topic = "/" + ns_ + "/safety/depth_grid";

  rclcpp::QoS qos(10);

  ukc_sub_ = node_->create_subscription<enc_grounding_warning_msgs::msg::UKCState>(
    ukc_topic, qos,
    [this](const enc_grounding_warning_msgs::msg::UKCState::SharedPtr msg) {
      QMetaObject::invokeMethod(this, [this, msg]() {
        const std::string risk = [&]() {
          switch (msg->risk_level) {
            case 0: return "SAFE";
            case 1: return "CAUTION";
            case 2: return "WARNING";
            case 3: return "DANGER";
            case 4: return "GROUNDED";
            default: return "UNKNOWN";
          }
        }();
        ukc_label_->setText(
          QString(
            "Risk: %1\nChart depth: %2 m\nDynamic draft: %3 m\n"
            "UKC: %4 m (required %5 m)\nUncertainty: %6 m\nSafety depth: %7 m")
            .arg(QString::fromStdString(risk))
            .arg(msg->chart_depth_m)
            .arg(msg->dynamic_draft_m)
            .arg(msg->ukc_m)
            .arg(msg->ukc_required_m)
            .arg(msg->uncertainty_m)
            .arg(msg->safety_depth_m));
        setRiskStyle(ukc_label_, msg->risk_level);
      }, Qt::QueuedConnection);
    });

  alert_sub_ = node_->create_subscription<enc_grounding_warning_msgs::msg::GroundingAlert>(
    alert_topic, qos,
    [this](const enc_grounding_warning_msgs::msg::GroundingAlert::SharedPtr msg) {
      QMetaObject::invokeMethod(this, [this, msg]() {
        alert_label_->setText(
          QString(
            "Level: %1  Type: %2\nMin UKC: %3 m\nDistance: %4 m  Time: %5 s\n"
            "Danger: (%6, %7)\n%8")
            .arg(msg->level)
            .arg(msg->type)
            .arg(msg->min_ukc_m)
            .arg(msg->distance_to_danger_m)
            .arg(msg->time_to_danger_s)
            .arg(msg->danger_x)
            .arg(msg->danger_y)
            .arg(QString::fromStdString(msg->description)));
      }, Qt::QueuedConnection);
    });

  grid_sub_ = node_->create_subscription<enc_grounding_warning_msgs::msg::DepthGrid>(
    grid_topic, qos,
    [this](const enc_grounding_warning_msgs::msg::DepthGrid::SharedPtr msg) {
      QMetaObject::invokeMethod(this, [this, msg]() {
        float min_depth = std::numeric_limits<float>::max();
        bool valid = false;
        for (const auto & d : msg->depth_m) {
          if (d > -1000.0f) {
            min_depth = std::min(min_depth, d);
            valid = true;
          }
        }
        grid_label_->setText(
          QString("Size: %1 x %2 @ %3 m\nMin depth: %4 m")
            .arg(msg->width)
            .arg(msg->height)
            .arg(msg->resolution)
            .arg(valid ? min_depth : 0.0f));
      }, Qt::QueuedConnection);
    });
}

void GroundingWarningPanel::onApplyNamespace()
{
  ns_ = ns_edit_->text().trimmed().toStdString();
  if (ns_.empty()) {
    ns_ = "usv_1";
    ns_edit_->setText(QString::fromStdString(ns_));
  }
  updateSubscriptions();
}

void GroundingWarningPanel::setRiskStyle(QLabel * label, uint8_t risk)
{
  const char * color = "black";
  switch (risk) {
    case 0: color = "green"; break;
    case 1: color = "orange"; break;
    case 2: color = "orange"; break;
    case 3: color = "red"; break;
    case 4: color = "darkred"; break;
    default: color = "gray"; break;
  }
  label->setStyleSheet(QString("color: %1; font-weight: bold;").arg(color));
}

void GroundingWarningPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString ns;
  if (config.mapGetString("Namespace", &ns)) {
    ns_ = ns.toStdString();
    ns_edit_->setText(ns);
    updateSubscriptions();
  }
}

void GroundingWarningPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Namespace", QString::fromStdString(ns_));
}

}  // namespace enc_grounding_warning_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  enc_grounding_warning_rviz::GroundingWarningPanel,
  rviz_common::Panel)
