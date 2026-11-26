// Copyright 2026 CCZH.
//
// RViz2 本船状态面板实现。

#include "env_panel/own_ship_status_panel.hpp"

#include <cmath>
#include <chrono>

#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace env_panel
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMsToKn = 1.9438444924406;

double yawFromQuat(double x, double y, double z, double w)
{
  // yaw (Z) from quaternion
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double rollFromQuat(double x, double y, double z, double w)
{
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  return std::atan2(sinr_cosp, cosr_cosp);
}

double pitchFromQuat(double x, double y, double z, double w)
{
  const double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0) {
    return std::copysign(kPi / 2.0, sinp);
  }
  return std::asin(sinp);
}

}  // namespace

OwnShipStatusPanel::OwnShipStatusPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setupUi();
}

OwnShipStatusPanel::~OwnShipStatusPanel()
{
  if (timer_) {
    timer_->stop();
  }
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void OwnShipStatusPanel::setupUi()
{
  auto * root = new QVBoxLayout;
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(6);

  setStyleSheet(
    "QGroupBox { font-weight: 600; margin-top: 8px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }"
    "QLabel[role='value'] { font-family: monospace; min-width: 140px; }");

  auto * ns_row = new QHBoxLayout;
  ns_row->addWidget(new QLabel("命名空间"));
  ns_edit_ = new QLineEdit(QString::fromStdString(ns_));
  ns_edit_->setPlaceholderText("usv_1");
  ns_row->addWidget(ns_edit_, 1);
  root->addLayout(ns_row);

  status_label_ = new QLabel("等待 ROS…");
  status_label_->setStyleSheet("color: #888;");
  root->addWidget(status_label_);

  auto * scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto * content = new QWidget;
  auto * content_layout = new QVBoxLayout(content);
  content_layout->setContentsMargins(0, 0, 0, 0);
  content_layout->setSpacing(6);

  auto make_group = [&](const QString & title) {
      auto * box = new QGroupBox(title, content);
      auto * form = new QFormLayout;
      form->setContentsMargins(10, fontMetrics().height() + 6, 10, 8);
      form->setHorizontalSpacing(12);
      form->setVerticalSpacing(4);
      form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
      box->setLayout(form);
      content_layout->addWidget(box);
      return form;
    };

  auto * motion = make_group("运动");
  sog_label_ = addRow(motion, "对地航速 SOG");
  sog_kn_label_ = addRow(motion, "对地航速");
  surge_label_ = addRow(motion, "纵向速度 u");
  cmd_u_label_ = addRow(motion, "指令速度");
  yaw_rate_label_ = addRow(motion, "转向角速度 r");
  cmd_r_label_ = addRow(motion, "指令角速度");
  heading_label_ = addRow(motion, "航向 yaw");
  cog_label_ = addRow(motion, "航迹向 COG");
  sideslip_label_ = addRow(motion, "漂角 β");
  steer_label_ = addRow(motion, "等效舵角");

  auto * thrust = make_group("推力 / 舵");
  thrust_l_label_ = addRow(thrust, "左推力");
  thrust_r_label_ = addRow(thrust, "右推力");
  thrust_sum_label_ = addRow(thrust, "推力合计");
  thrust_diff_label_ = addRow(thrust, "推力差 (R−L)");
  az_l_label_ = addRow(thrust, "左舵角/方位");
  az_r_label_ = addRow(thrust, "右舵角/方位");

  auto * pose = make_group("位姿");
  pose_label_ = addRow(pose, "位置 (odom)");
  gps_label_ = addRow(pose, "GPS");
  attitude_label_ = addRow(pose, "横摇/纵摇");

  auto * hull = make_group("船体");
  draft_label_ = addRow(hull, "平均吃水");
  trim_heel_label_ = addRow(hull, "纵倾 / 横倾");

  auto * sys = make_group("系统");
  nav_ready_label_ = addRow(sys, "导航就绪");
  cte_label_ = addRow(sys, "循线横向误差");
  age_label_ = addRow(sys, "数据时效");

  content_layout->addStretch(1);
  scroll->setWidget(content);
  root->addWidget(scroll, 1);
  setLayout(root);

  connect(ns_edit_, &QLineEdit::editingFinished, this, &OwnShipStatusPanel::onNamespaceEdited);

  timer_ = new QTimer(this);
  timer_->setInterval(100);
  connect(timer_, &QTimer::timeout, this, &OwnShipStatusPanel::refreshUi);
}

QLabel * OwnShipStatusPanel::addRow(QFormLayout * form, const QString & name)
{
  auto * value = new QLabel("—");
  value->setProperty("role", "value");
  value->setTextInteractionFlags(Qt::TextSelectableByMouse);
  form->addRow(name, value);
  return value;
}

void OwnShipStatusPanel::onInitialize()
{
  setupRos();
  timer_->start();
}

void OwnShipStatusPanel::setupRos()
{
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "--remap", "__node:=rviz_own_ship_status_panel", "--"});
  node_ = std::make_shared<rclcpp::Node>("_", options);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {executor_->spin();});
  resubscribe();
  status_label_->setText(QString("订阅中 · ns=/%1").arg(QString::fromStdString(ns_)));
  status_label_->setStyleSheet("color: #2e7d32; font-weight: 600;");
}

void OwnShipStatusPanel::resubscribe()
{
  odom_sub_.reset();
  cmd_sub_.reset();
  gps_sub_.reset();
  imu_sub_.reset();
  thrust_l_sub_.reset();
  thrust_r_sub_.reset();
  pos_l_sub_.reset();
  pos_r_sub_.reset();
  draft_sub_.reset();
  ready_sub_.reset();
  cte_sub_.reset();

  if (!node_) {
    return;
  }

  const std::string prefix = "/" + ns_;
  const auto sensor_qos = rclcpp::SensorDataQoS();

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    prefix + "/odom", sensor_qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      Snapshot local;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        local = snap_;
      }
      local.odom_valid = true;
      local.rx_odom = nowSeconds();
      local.x = msg->pose.pose.position.x;
      local.y = msg->pose.pose.position.y;
      local.z = msg->pose.pose.position.z;
      const auto & q = msg->pose.pose.orientation;
      local.yaw_rad = yawFromQuat(q.x, q.y, q.z, q.w);
      if (!local.imu_valid) {
        local.roll_rad = rollFromQuat(q.x, q.y, q.z, q.w);
        local.pitch_rad = pitchFromQuat(q.x, q.y, q.z, q.w);
      }
      local.u = msg->twist.twist.linear.x;
      local.v = msg->twist.twist.linear.y;
      local.yaw_rate = msg->twist.twist.angular.z;
      local.sog = std::hypot(local.u, local.v);
      if (local.sog > 0.05) {
        local.cog_rad = std::atan2(local.v, local.u) + local.yaw_rad;
      } else {
        local.cog_rad = local.yaw_rad;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      snap_ = local;
    });

  cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    prefix + "/cmd_vel", 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      snap_.cmd_valid = true;
      snap_.rx_cmd = nowSeconds();
      snap_.cmd_u = msg->linear.x;
      snap_.cmd_r = msg->angular.z;
    });

  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    prefix + "/sensors/gps/gps_sensor/data", sensor_qos,
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      snap_.gps_valid = true;
      snap_.rx_gps = nowSeconds();
      snap_.lat = msg->latitude;
      snap_.lon = msg->longitude;
      snap_.alt = msg->altitude;
    });

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    prefix + "/sensors/imu/imu_sensor/data", sensor_qos,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      const auto & q = msg->orientation;
      const double n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (n2 < 1e-9) {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      snap_.imu_valid = true;
      snap_.roll_rad = rollFromQuat(q.x, q.y, q.z, q.w);
      snap_.pitch_rad = pitchFromQuat(q.x, q.y, q.z, q.w);
    });

  thrust_l_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    prefix + "/thrusters/left/thrust", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      snap_.thrust_valid = true;
      snap_.thrust_left = msg->data;
    });
  thrust_r_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    prefix + "/thrusters/right/thrust", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      snap_.thrust_valid = true;
      snap_.thrust_right = msg->data;
    });
  pos_l_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    prefix + "/thrusters/left/pos", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      snap_.pos_left = msg->data;
    });
  pos_r_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    prefix + "/thrusters/right/pos", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      snap_.pos_right = msg->data;
    });

  draft_sub_ = node_->create_subscription<usv_interfaces::msg::HullDraft>(
    prefix + "/hull_draft", 10,
    [this](const usv_interfaces::msg::HullDraft::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      snap_.draft_valid = true;
      snap_.draft_avg = msg->average_draft;
      snap_.trim = msg->trim;
      snap_.heel = msg->heel;
    });

  ready_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    prefix + "/navigation/ready", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      snap_.nav_ready_valid = true;
      snap_.nav_ready = msg->data;
    });

  // Optional; ignore if publisher absent.
  cte_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/line_follow_tuner/cross_track", 10,
    [this](const std_msgs::msg::Float32::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      snap_.cte_valid = true;
      snap_.cross_track = msg->data;
    });
}

void OwnShipStatusPanel::onNamespaceEdited()
{
  const QString text = ns_edit_->text().trimmed();
  if (text.isEmpty()) {
    ns_edit_->setText(QString::fromStdString(ns_));
    return;
  }
  QString cleaned = text;
  while (cleaned.startsWith('/')) {
    cleaned.remove(0, 1);
  }
  ns_edit_->setText(cleaned);
  const std::string next = cleaned.toStdString();
  if (next == ns_) {
    return;
  }
  ns_ = next;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snap_ = Snapshot{};
  }
  resubscribe();
  status_label_->setText(QString("已切换 · ns=/%1").arg(cleaned));
}

void OwnShipStatusPanel::refreshUi()
{
  Snapshot s;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    s = snap_;
  }
  const double now = nowSeconds();

  if (s.odom_valid) {
    sog_label_->setText(fmt(s.sog, 2, "m/s"));
    sog_kn_label_->setText(fmt(s.sog * kMsToKn, 2, "kn"));
    surge_label_->setText(fmt(s.u, 2, "m/s"));
    yaw_rate_label_->setText(
      QString("%1 rad/s  (%2 °/s)")
      .arg(s.yaw_rate, 0, 'f', 3)
      .arg(radToDeg(s.yaw_rate), 0, 'f', 1));
    heading_label_->setText(
      QString("%1°  (%2 rad)")
      .arg(radToDeg(s.yaw_rad), 0, 'f', 1)
      .arg(s.yaw_rad, 0, 'f', 3));
    cog_label_->setText(QString("%1°").arg(radToDeg(wrapPi(s.cog_rad)), 0, 'f', 1));
    const double beta = (s.sog > 0.05) ? std::atan2(s.v, s.u) : 0.0;
    sideslip_label_->setText(QString("%1°").arg(radToDeg(beta), 0, 'f', 1));
    // 差速船等效舵角：用 r 与 u 估一个小角度，便于直观看转向强度。
    const double steer = (std::abs(s.u) > 0.2) ? std::atan2(s.yaw_rate * 5.0, s.u) : 0.0;
    steer_label_->setText(QString("%1°").arg(radToDeg(steer), 0, 'f', 1));
    pose_label_->setText(
      QString("x=%1  y=%2  z=%3")
      .arg(s.x, 0, 'f', 1).arg(s.y, 0, 'f', 1).arg(s.z, 0, 'f', 2));
  } else {
    sog_label_->setText("—");
    sog_kn_label_->setText("—");
    surge_label_->setText("—");
    yaw_rate_label_->setText("—");
    heading_label_->setText("—");
    cog_label_->setText("—");
    sideslip_label_->setText("—");
    steer_label_->setText("—");
    pose_label_->setText("—");
  }

  if (s.cmd_valid) {
    cmd_u_label_->setText(fmt(s.cmd_u, 2, "m/s"));
    cmd_r_label_->setText(
      QString("%1 rad/s  (%2 °/s)")
      .arg(s.cmd_r, 0, 'f', 3)
      .arg(radToDeg(s.cmd_r), 0, 'f', 1));
  } else {
    cmd_u_label_->setText("—");
    cmd_r_label_->setText("—");
  }

  if (s.thrust_valid) {
    thrust_l_label_->setText(fmt(s.thrust_left, 0, "N"));
    thrust_r_label_->setText(fmt(s.thrust_right, 0, "N"));
    thrust_sum_label_->setText(fmt(s.thrust_left + s.thrust_right, 0, "N"));
    thrust_diff_label_->setText(fmt(s.thrust_right - s.thrust_left, 0, "N"));
    az_l_label_->setText(QString("%1°").arg(radToDeg(s.pos_left), 0, 'f', 1));
    az_r_label_->setText(QString("%1°").arg(radToDeg(s.pos_right), 0, 'f', 1));
  } else {
    thrust_l_label_->setText("—");
    thrust_r_label_->setText("—");
    thrust_sum_label_->setText("—");
    thrust_diff_label_->setText("—");
    az_l_label_->setText("—");
    az_r_label_->setText("—");
  }

  if (s.gps_valid) {
    gps_label_->setText(
      QString("%1°, %2°  alt=%3 m")
      .arg(s.lat, 0, 'f', 6)
      .arg(s.lon, 0, 'f', 6)
      .arg(s.alt, 0, 'f', 1));
  } else {
    gps_label_->setText("—");
  }

  if (s.imu_valid || s.odom_valid) {
    attitude_label_->setText(
      QString("φ=%1°  θ=%2°")
      .arg(radToDeg(s.roll_rad), 0, 'f', 1)
      .arg(radToDeg(s.pitch_rad), 0, 'f', 1));
  } else {
    attitude_label_->setText("—");
  }

  if (s.draft_valid) {
    draft_label_->setText(fmt(s.draft_avg, 3, "m"));
    trim_heel_label_->setText(
      QString("trim=%1 m  heel=%2 m")
      .arg(s.trim, 0, 'f', 3)
      .arg(s.heel, 0, 'f', 3));
  } else {
    draft_label_->setText("—");
    trim_heel_label_->setText("—");
  }

  if (s.nav_ready_valid) {
    nav_ready_label_->setText(s.nav_ready ? "true" : "false");
    nav_ready_label_->setStyleSheet(
      s.nav_ready ? "color: #2e7d32; font-weight: 600;" : "color: #c62828; font-weight: 600;");
  } else {
    nav_ready_label_->setText("—");
    nav_ready_label_->setStyleSheet("");
  }

  if (s.cte_valid) {
    cte_label_->setText(fmt(s.cross_track, 2, "m"));
  } else {
    cte_label_->setText("—");
  }

  age_label_->setText(
    QString("odom %1 · cmd %2 · gps %3")
    .arg(s.odom_valid ? ageText(s.rx_odom, now) : "—")
    .arg(s.cmd_valid ? ageText(s.rx_cmd, now) : "—")
    .arg(s.gps_valid ? ageText(s.rx_gps, now) : "—"));
}

void OwnShipStatusPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString ns;
  if (config.mapGetString("namespace", &ns) && !ns.isEmpty()) {
    ns_edit_->setText(ns);
    onNamespaceEdited();
  }
}

void OwnShipStatusPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("namespace", ns_edit_ ? ns_edit_->text() : QString::fromStdString(ns_));
}

double OwnShipStatusPanel::nowSeconds()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

double OwnShipStatusPanel::wrapPi(double a)
{
  while (a > kPi) {a -= 2.0 * kPi;}
  while (a < -kPi) {a += 2.0 * kPi;}
  return a;
}

double OwnShipStatusPanel::radToDeg(double a)
{
  return a * 180.0 / kPi;
}

QString OwnShipStatusPanel::fmt(double value, int decimals, const QString & unit)
{
  return QString("%1 %2").arg(value, 0, 'f', decimals).arg(unit);
}

QString OwnShipStatusPanel::ageText(double stamp, double now)
{
  if (stamp <= 0.0) {
    return "—";
  }
  const double age = std::max(0.0, now - stamp);
  if (age < 0.5) {
    return QString("%1 ms").arg(static_cast<int>(age * 1000.0));
  }
  return QString("%1 s").arg(age, 0, 'f', 1);
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::OwnShipStatusPanel, rviz_common::Panel)
