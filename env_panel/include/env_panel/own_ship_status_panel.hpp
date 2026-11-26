// Copyright 2026 CCZH.
//
// RViz2 本船状态面板：实时显示航速、航向/转向、推力与船体姿态等。

#ifndef ENV_PANEL__OWN_SHIP_STATUS_PANEL_HPP_
#define ENV_PANEL__OWN_SHIP_STATUS_PANEL_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QString>
#include <QTimer>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <usv_interfaces/msg/hull_draft.hpp>

namespace env_panel
{

class OwnShipStatusPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit OwnShipStatusPanel(QWidget * parent = nullptr);
  ~OwnShipStatusPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void refreshUi();
  void onNamespaceEdited();

private:
  struct Snapshot
  {
    bool odom_valid{false};
    bool cmd_valid{false};
    bool gps_valid{false};
    bool imu_valid{false};
    bool draft_valid{false};
    bool thrust_valid{false};
    bool nav_ready{false};
    bool nav_ready_valid{false};
    bool cte_valid{false};

    double rx_odom{0.0};
    double rx_cmd{0.0};
    double rx_gps{0.0};

    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw_rad{0.0};

    double u{0.0};          ///< body surge (odom twist.linear.x)
    double v{0.0};          ///< body sway
    double sog{0.0};        ///< speed over ground
    double yaw_rate{0.0};   ///< actual r
    double cog_rad{0.0};    ///< course over ground from velocity

    double cmd_u{0.0};
    double cmd_r{0.0};

    double lat{0.0};
    double lon{0.0};
    double alt{0.0};

    double roll_rad{0.0};
    double pitch_rad{0.0};

    double thrust_left{0.0};
    double thrust_right{0.0};
    double pos_left{0.0};
    double pos_right{0.0};

    double draft_avg{0.0};
    double trim{0.0};
    double heel{0.0};

    double cross_track{0.0};
  };

  void setupUi();
  void setupRos();
  void resubscribe();
  static double nowSeconds();
  static double wrapPi(double a);
  static double radToDeg(double a);
  static QString fmt(double value, int decimals, const QString & unit);
  static QString ageText(double stamp, double now);
  QLabel * addRow(QFormLayout * form, const QString & name);

  std::string ns_{"usv_1"};

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr thrust_l_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr thrust_r_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pos_l_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pos_r_sub_;
  rclcpp::Subscription<usv_interfaces::msg::HullDraft>::SharedPtr draft_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cte_sub_;

  mutable std::mutex mutex_;
  Snapshot snap_;

  QTimer * timer_{nullptr};
  QLineEdit * ns_edit_{nullptr};
  QLabel * status_label_{nullptr};

  QLabel * sog_label_{nullptr};
  QLabel * sog_kn_label_{nullptr};
  QLabel * surge_label_{nullptr};
  QLabel * cmd_u_label_{nullptr};
  QLabel * yaw_rate_label_{nullptr};
  QLabel * cmd_r_label_{nullptr};
  QLabel * heading_label_{nullptr};
  QLabel * cog_label_{nullptr};
  QLabel * sideslip_label_{nullptr};
  QLabel * steer_label_{nullptr};

  QLabel * thrust_l_label_{nullptr};
  QLabel * thrust_r_label_{nullptr};
  QLabel * thrust_sum_label_{nullptr};
  QLabel * thrust_diff_label_{nullptr};
  QLabel * az_l_label_{nullptr};
  QLabel * az_r_label_{nullptr};

  QLabel * pose_label_{nullptr};
  QLabel * gps_label_{nullptr};
  QLabel * attitude_label_{nullptr};

  QLabel * draft_label_{nullptr};
  QLabel * trim_heel_label_{nullptr};

  QLabel * nav_ready_label_{nullptr};
  QLabel * cte_label_{nullptr};
  QLabel * age_label_{nullptr};
};

}  // namespace env_panel

#endif  // ENV_PANEL__OWN_SHIP_STATUS_PANEL_HPP_
