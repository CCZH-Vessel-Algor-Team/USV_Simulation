#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <QApplication>
#include <gtest/gtest.h>
#include "env_panel/dynamic_ship_panel.hpp"

namespace env_panel
{
class DynamicShipPanelTestPeer
{
public:
  static std::string id(const DynamicShipPanel & panel) {return panel.model_id_;}
  static bool applied(const DynamicShipPanel & panel)
  {return panel.applied_revision_ == panel.revision_;}
  static int count(const DynamicShipPanel & panel) {return panel.model_combo_->count();}
  static void select(DynamicShipPanel & panel, const std::string & id)
  {panel.model_combo_->setCurrentIndex(panel.model_combo_->findData(QString::fromStdString(id)));}
  static void timeout(DynamicShipPanel & panel)
  {
    panel.config_pending_ = true;
    panel.applied_revision_ = 0;
    panel.config_sent_ms_ = panel.elapsed_.elapsed() - 6000;
    panel.publishConfig();
  }
  static QString status(const DynamicShipPanel & panel) {return panel.config_status_->text();}
  static QString actionStatus(const DynamicShipPanel & panel) {return panel.action_status_->text();}
  static void clearAll(DynamicShipPanel & panel) {panel.clear_all_button_->click();}
  static bool clearReady(const DynamicShipPanel & panel)
  {return panel.clear_client_->service_is_ready();}
  static void displayAndDelete(DynamicShipPanel & panel)
  {
    panel.instance_models_["instance_a"] = "huge_test_model";
    panel.instance_models_["dyn_target_7"] = "huge_test_model";
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "[\"instance_a\",\"dyn_target_7\"]";
    panel.onNamesReceived(msg);
    QApplication::processEvents();
    ASSERT_EQ(panel.ship_list_->count(), 2);
    EXPECT_TRUE(panel.ship_list_->item(1)->text().startsWith("动态船 7 — "));
    EXPECT_EQ(panel.ship_list_->item(1)->data(Qt::UserRole).toString(), "dyn_target_7");
    panel.ship_list_->setCurrentRow(0);
    EXPECT_TRUE(panel.ship_list_->currentItem()->text().contains("测试大船"));
    EXPECT_EQ(panel.ship_list_->currentItem()->data(Qt::UserRole).toString(), "instance_a");
    panel.onDeleteClicked();
  }
};
}  // namespace env_panel

using env_panel::DynamicShipPanel;
using Peer = env_panel::DynamicShipPanelTestPeer;

static bool waitFor(const std::function<bool()> & condition, double seconds = 8.)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    QApplication::processEvents();
    if (condition()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

TEST(DynamicShipPanel, SavesAndRestoresPerInstanceSelectionBeforeCatalogArrives)
{
  DynamicShipPanel first, second;
  rviz_common::Config input;
  input.mapSetValue("ModelId", "huge_test_model");
  input.mapSetValue("HeadingDeg", 123.);
  input.mapSetValue("Speed", 1.2);
  input.mapSetValue("HalfDistance", 300.);
  first.load(input);
  rviz_common::Config saved;
  first.save(saved);
  second.load(saved);
  EXPECT_EQ(Peer::id(first), "huge_test_model");
  EXPECT_EQ(Peer::id(second), "huge_test_model");
  float value = 0;
  ASSERT_TRUE(saved.mapGetFloat("Speed", &value));
  EXPECT_NEAR(value, 1.2, .001);
  ASSERT_TRUE(saved.mapGetFloat("HalfDistance", &value));
  EXPECT_NEAR(value, 300., .001);
  EXPECT_FALSE(Peer::applied(first));
}

TEST(DynamicShipPanel, KeepsPlacementModesAndManageButtonsWithoutManualRetry)
{
  DynamicShipPanel panel;
  const auto buttons = panel.findChildren<QPushButton *>();
  ASSERT_EQ(buttons.size(), 4);
  EXPECT_EQ(buttons[0]->objectName(), "sim_ship_mode");
  EXPECT_EQ(buttons[1]->objectName(), "virtual_ship_mode");
  EXPECT_EQ(buttons[2]->text(), "删除所选");
  EXPECT_EQ(buttons[3]->text(), "全部清除");
  for (const auto * label : panel.findChildren<QLabel *>()) {
    EXPECT_FALSE(label->text().contains("只影响以后生成的船"));
  }
  for (const auto * button : panel.findChildren<QPushButton *>()) {
    EXPECT_FALSE(button->text().contains("重新获取目录"));
    EXPECT_FALSE(button->text().contains("重试配置"));
  }
}

TEST(DynamicShipPanel, QueriesBackendRetriesFailuresAndManagesShips)
{
  auto backend = std::make_shared<rclcpp::Node>("dynamic_ship_panel_test_backend");
  std::atomic<int> huge_requests{0};
  std::atomic<bool> clear_success{false};
  std::mutex mutex;
  std::string deleted;
  auto models = backend->create_service<usv_interfaces::srv::ListDynamicShipModels>(
    "/dynamic_ship/models", [](
      const std::shared_ptr<usv_interfaces::srv::ListDynamicShipModels::Request>,
      const std::shared_ptr<usv_interfaces::srv::ListDynamicShipModels::Response> response) {
      response->success = true;
      response->default_model_id = "legacy_10m";
      for (const auto & id : {"legacy_10m", "huge_test_model"}) {
        usv_interfaces::msg::DynamicShipModelInfo model;
        model.model_id = id;
        model.display_name = id == std::string("legacy_10m") ? "默认目标船" : "测试大船";
        model.length = 399.; model.beam = 58.; model.height = 44.5;
        response->models.push_back(model);
      }
    });
  auto config = backend->create_service<usv_interfaces::srv::SetDynamicShipConfig>(
    "/dynamic_ship/set_config", [&huge_requests](
      const std::shared_ptr<usv_interfaces::srv::SetDynamicShipConfig::Request> request,
      const std::shared_ptr<usv_interfaces::srv::SetDynamicShipConfig::Response> response) {
      response->model_id = request->model_id;
      response->success = request->model_id != "huge_test_model" || ++huge_requests > 1;
      response->message = response->success ? "applied" : "intentional retry test";
    });
  auto remove = backend->create_service<usv_interfaces::srv::DeleteDynamicShip>(
    "/dynamic_ship/delete", [&mutex, &deleted](
      const std::shared_ptr<usv_interfaces::srv::DeleteDynamicShip::Request> request,
      const std::shared_ptr<usv_interfaces::srv::DeleteDynamicShip::Response> response) {
      std::lock_guard<std::mutex> lock(mutex);
      deleted = request->model_name;
      response->success = true;
    });
  auto clear = backend->create_service<usv_interfaces::srv::ClearDynamicShips>(
    "/dynamic_ship/clear", [&clear_success](
      const std::shared_ptr<usv_interfaces::srv::ClearDynamicShips::Request>,
      const std::shared_ptr<usv_interfaces::srv::ClearDynamicShips::Response> response) {
      response->success = clear_success.load();
      response->message = response->success ? "Cleared 1 of 1 ships" : "intentional clear failure";
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(backend);
  std::thread thread([&executor]() {executor.spin();});
  // RAII even when an ASSERT aborts the test body.
  struct Stop {
    rclcpp::Executor & executor;
    std::thread & thread;
    ~Stop() {executor.cancel(); if (thread.joinable()) thread.join();}
  } stop{executor, thread};
  DynamicShipPanel panel;
  panel.onInitialize();
  ASSERT_TRUE(waitFor([&panel]() {return Peer::count(panel) == 2 && Peer::applied(panel);}));
  Peer::select(panel, "huge_test_model");
  EXPECT_FALSE(Peer::applied(panel));
  ASSERT_TRUE(waitFor([&panel]() {return Peer::status(panel).contains("intentional retry test");}));
  EXPECT_FALSE(Peer::applied(panel));
  ASSERT_TRUE(waitFor([&panel]() {return Peer::applied(panel);}));
  EXPECT_TRUE(Peer::status(panel).isEmpty());
  EXPECT_GE(huge_requests.load(), 2);
  Peer::timeout(panel);
  EXPECT_FALSE(Peer::applied(panel));
  EXPECT_TRUE(Peer::status(panel).contains("超时"));
  ASSERT_TRUE(waitFor([&panel]() {return Peer::applied(panel);}));
  Peer::displayAndDelete(panel);
  ASSERT_TRUE(waitFor([&mutex, &deleted]() {
    std::lock_guard<std::mutex> lock(mutex);
    return deleted == "instance_a";
  }));
  ASSERT_TRUE(waitFor([&panel]() {return Peer::clearReady(panel);}));
  ASSERT_TRUE(waitFor([&panel]() {return Peer::actionStatus(panel).startsWith("删除成功：");}));
  Peer::clearAll(panel);
  ASSERT_TRUE(waitFor([&panel]() {
    return Peer::actionStatus(panel).startsWith("清空未完成：");
  }));
  clear_success.store(true);
  Peer::clearAll(panel);
  EXPECT_FALSE(Peer::actionStatus(panel).isEmpty());
  ASSERT_TRUE(waitFor([&panel]() {return Peer::actionStatus(panel).isEmpty();}));
  // A new panel is not incorrectly treated as already configured by static
  // last-values left behind by an older panel instance.
  DynamicShipPanel restored;
  rviz_common::Config saved;
  panel.save(saved);
  restored.load(saved);
  restored.onInitialize();
  ASSERT_TRUE(waitFor([&restored]() {return Peer::applied(restored);}));
  EXPECT_EQ(Peer::id(restored), "huge_test_model");
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
