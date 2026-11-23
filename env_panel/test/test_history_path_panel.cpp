#include <QApplication>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QPushButton>
#include <QThread>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

TEST(HistoryPathSettings, LoadsPluginReadsBackendAndAppliesWithoutChangingOtherParameters)
{
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
  ASSERT_TRUE(loader.isClassAvailable("env_panel/HistoryPathSettings"));
  auto panel = loader.createSharedInstance("env_panel/HistoryPathSettings");
  panel->onInitialize();
  auto * duration = panel->findChild<QDoubleSpinBox *>("history_duration");
  auto * apply = panel->findChild<QPushButton *>("history_apply");
  auto * refresh = panel->findChild<QPushButton *>("history_refresh");
  auto * status = panel->findChild<QLabel *>("history_status");
  ASSERT_NE(duration, nullptr);
  ASSERT_NE(apply, nullptr);
  ASSERT_NE(refresh, nullptr);
  ASSERT_NE(status, nullptr);
  EXPECT_FALSE(apply->isEnabled());

  auto backend = std::make_shared<rclcpp::Node>("history_path");
  backend->declare_parameter("history_duration_sec", 120.0);
  backend->declare_parameter("max_points", 10000);
  auto wait_until = [&](auto predicate) {
      QElapsedTimer timeout;
      timeout.start();
      while (!predicate() && timeout.elapsed() < 5000) {
        rclcpp::spin_some(backend);
        QApplication::processEvents();
        QThread::msleep(10);
      }
      return predicate();
    };
  ASSERT_TRUE(wait_until([&]() {return apply->isEnabled();}));
  EXPECT_DOUBLE_EQ(duration->value(), 120.0);
  EXPECT_DOUBLE_EQ(backend->get_parameter("history_duration_sec").as_double(), 120.0);
  duration->setValue(45.0);
  EXPECT_DOUBLE_EQ(backend->get_parameter("history_duration_sec").as_double(), 120.0);
  apply->click();
  ASSERT_TRUE(wait_until([&]() {
      return backend->get_parameter("history_duration_sec").as_double() == 45.0 &&
             apply->isEnabled();
    }));
  EXPECT_EQ(backend->get_parameter("max_points").as_int(), 10000);

  backend->set_parameter(rclcpp::Parameter("history_duration_sec", 90.0));
  refresh->click();
  ASSERT_TRUE(wait_until([&]() {return duration->value() == 90.0 && apply->isEnabled();}));
  if (qEnvironmentVariableIsSet("HISTORY_PANEL_PREVIEW")) {
    panel->resize(340, 190);
    panel->show();
    QApplication::processEvents();
    panel->grab().save(qEnvironmentVariable("HISTORY_PANEL_PREVIEW"));
  }
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
