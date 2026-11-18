#include <gtest/gtest.h>

#include <QApplication>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QThread>

#include <chrono>
#include <functional>

#include "pluginlib/class_loader.hpp"
#include "rviz_common/tool.hpp"
#include "enc_grounding_warning_rviz/depth_settings_panel.hpp"
#include "enc_grounding_warning_rviz/grounding_warning_panel.hpp"

TEST(DepthPanel, MapClicksApplyImmediatelyAndDeletionRestoresWithoutConfirmation)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QTemporaryDir logs;
  qputenv("ROS_LOG_DIR", logs.path().toUtf8());
  int argc = 1;
  char name[] = "depth_panel_test";
  char * argv[] = {name, nullptr};
  QApplication application(argc, argv);
  rclcpp::init(argc, argv);
  {
    pluginlib::ClassLoader<rviz_common::Panel> panels("rviz_common", "rviz_common::Panel");
    EXPECT_TRUE(panels.isClassAvailable("enc_grounding_warning_rviz/GroundingWarningPanel"));
    EXPECT_TRUE(panels.isClassAvailable("enc_grounding_warning_rviz/DepthSettingsPanel"));
    pluginlib::ClassLoader<rviz_common::Tool> tools("rviz_common", "rviz_common::Tool");
    ASSERT_TRUE(tools.isClassAvailable("enc_grounding_warning_rviz/DepthPointTool"));
    auto tool = tools.createSharedInstance("enc_grounding_warning_rviz/DepthPointTool");
    EXPECT_EQ(tool->getShortcutKey(), 'w');
    using Service = enc_grounding_warning_msgs::srv::ConfigureDepth;
    auto server = std::make_shared<rclcpp::Node>("depth_panel_test_service", "/depth_panel_test");
    enc_grounding_warning_msgs::msg::DepthSettings saved;
    saved.base_depth_m = 4.5;
    int applies = 0;
    bool reject_next = false;
    auto service = server->create_service<Service>("safety/configure_depth",
      [&](const Service::Request::SharedPtr request, Service::Response::SharedPtr response) {
        if (request->apply) {
          ++applies;
          if (reject_next) {
            reject_next = false;
            response->success = false;
            response->settings = saved;
            response->message = "保存失败";
            return;
          }
          saved = request->settings;
          saved.enabled = !saved.shoals.empty();
        }
        response->success = true;
        response->settings = saved;
      });
    auto point_pub = server->create_publisher<geometry_msgs::msg::PointStamped>("safety/depth_point", 10);
    enc_grounding_warning_rviz::GroundingWarningPanel warning;
    EXPECT_TRUE(warning.findChildren<QDoubleSpinBox *>().empty());
    enc_grounding_warning_rviz::DepthSettingsPanel panel;
    rviz_common::Config config;
    config.mapSetValue("Namespace", "depth_panel_test");
    warning.load(config);
    panel.load(config);
    panel.onInitialize();
    panel.resize(360, 480);
    panel.show();
    ASSERT_EQ(panel.findChildren<QDoubleSpinBox *>().size(), 2);
    // Deletion actions only; no add/sync/confirmation buttons.
    ASSERT_EQ(panel.findChildren<QPushButton *>().size(), 2);
    auto * radius = panel.findChild<QDoubleSpinBox *>("depthRadius");
    auto * depth = panel.findChild<QDoubleSpinBox *>("regionDepth");
    EXPECT_TRUE(radius->suffix().isEmpty());
    EXPECT_TRUE(depth->suffix().isEmpty());
    int external_units = 0;
    for (auto * label : panel.findChildren<QLabel *>()) {
      if (label->text() == "米") {++external_units;}
    }
    EXPECT_EQ(external_units, 2);
    auto * table = panel.findChild<QTableWidget *>("depthRegions");
    auto * remove = panel.findChild<QPushButton *>("removeDepthRegion");
    auto * remove_all = panel.findChild<QPushButton *>("removeAllDepthRegions");
    auto * status = panel.findChild<QLabel *>("depthSettingsStatus");
    auto until = [&](const std::function<bool()> & condition) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
      while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(server);
        application.processEvents();
        if (condition()) {return true;}
        QThread::msleep(10);
      }
      return false;
    };
    ASSERT_TRUE(until([&]() {return radius->isEnabled() && point_pub->get_subscription_count() > 0;}));
    EXPECT_EQ(table->rowCount(), 0);
    EXPECT_FALSE(remove->isEnabled());
    EXPECT_FALSE(remove_all->isEnabled());
    EXPECT_EQ(table->editTriggers(), QAbstractItemView::NoEditTriggers);
    radius->setValue(15.0);
    depth->setValue(6.0);  // User depths may also be deeper than the original map.
    EXPECT_EQ(applies, 0);
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = "map";
    point.point.x = -20.0;
    point.point.y = 15.0;
    point_pub->publish(point);
    ASSERT_TRUE(until([&]() {return applies == 1 && table->isEnabled();}));
    ASSERT_EQ(saved.shoals.size(), 1u);
    EXPECT_EQ(table->rowCount(), 1);
    EXPECT_DOUBLE_EQ(saved.shoals[0].x, -20.0);
    EXPECT_DOUBLE_EQ(saved.shoals[0].y, 15.0);
    EXPECT_DOUBLE_EQ(saved.shoals[0].radius, 15.0);
    EXPECT_DOUBLE_EQ(saved.shoals[0].depth_m, 6.0);

    // Rapid consecutive map clicks are queued rather than lost during a request.
    radius->setValue(12.0);
    depth->setValue(0.7);
    point.point.x = 10.0;
    point_pub->publish(point);
    point.point.x = 20.0;
    point_pub->publish(point);
    ASSERT_TRUE(until([&]() {return applies == 3 && table->isEnabled();}));
    ASSERT_EQ(saved.shoals.size(), 3u);
    EXPECT_EQ(table->rowCount(), 3);
    EXPECT_DOUBLE_EQ(saved.shoals[1].x, 10.0);
    EXPECT_DOUBLE_EQ(saved.shoals[2].x, 20.0);
    table->selectRow(1);
    remove->click();
    ASSERT_TRUE(until([&]() {return applies == 4 && table->isEnabled();}));
    ASSERT_EQ(saved.shoals.size(), 2u);
    EXPECT_EQ(table->rowCount(), 2);
    EXPECT_DOUBLE_EQ(saved.shoals[1].x, 20.0);

    point.header.frame_id = "odom";
    point_pub->publish(point);
    ASSERT_TRUE(until([&]() {return status->text().contains("无效");}));
    EXPECT_EQ(applies, 4);
    point.header.frame_id = "map";
    point.point.x = 99.0;
    reject_next = true;
    point_pub->publish(point);
    ASSERT_TRUE(until([&]() {return applies == 5 && table->isEnabled();}));
    EXPECT_EQ(saved.shoals.size(), 2u);
    EXPECT_EQ(table->rowCount(), 2);  // Failed additions must not appear as active.
    point_pub->publish(point);
    ASSERT_TRUE(until([&]() {return applies == 6 && table->isEnabled();}));
    EXPECT_EQ(table->rowCount(), 3);
    table->selectRow(0);
    reject_next = true;
    remove->click();
    ASSERT_TRUE(until([&]() {return applies == 7 && table->isEnabled();}));
    EXPECT_EQ(table->rowCount(), 3);  // Failed deletions must remain visible.
    table->clearSelection();
    EXPECT_TRUE(remove_all->isEnabled());
    reject_next = true;
    remove_all->click();
    ASSERT_TRUE(until([&]() {return applies == 8 && table->isEnabled();}));
    EXPECT_EQ(table->rowCount(), 3);  // A failed bulk deletion preserves all regions.
    table->clearSelection();
    remove_all->click();
    ASSERT_TRUE(until([&]() {return applies == 9 && table->isEnabled();}));
    EXPECT_FALSE(saved.enabled);
    EXPECT_TRUE(saved.shoals.empty());
    EXPECT_EQ(table->rowCount(), 0);
    EXPECT_FALSE(remove_all->isEnabled());
    EXPECT_TRUE(status->text().contains("原始水深"));
    if (qEnvironmentVariableIsSet("DEPTH_PANEL_SCREENSHOT")) {
      EXPECT_TRUE(panel.grab().save(qgetenv("DEPTH_PANEL_SCREENSHOT")));
    }
  }
  rclcpp::shutdown();
}
