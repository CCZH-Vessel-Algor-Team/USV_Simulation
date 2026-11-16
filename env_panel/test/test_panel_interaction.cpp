#include "env_panel/dynamic_ship_panel.hpp"
#include "env_panel/storm_field_panel.hpp"

#include <QLayout>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool.hpp>

namespace
{
// The real mode buttons use this context's QObject properties; no ROS node is started.
class TestContext : public rviz_common::DisplayContext
{
public:
  Ogre::SceneManager * getSceneManager() const override {return nullptr;}
  rviz_common::WindowManagerInterface * getWindowManager() const override {return nullptr;}
  std::shared_ptr<rviz_common::interaction::SelectionManagerIface> getSelectionManager() const override {return {};}
  std::shared_ptr<rviz_common::interaction::HandlerManagerIface> getHandlerManager() const override {return {};}
  std::shared_ptr<rviz_common::interaction::ViewPickerIface> getViewPicker() const override {return {};}
  rviz_common::FrameManagerIface * getFrameManager() const override {return nullptr;}
  QString getFixedFrame() const override {return "map";}
  uint64_t getFrameCount() const override {return 0;}
  rviz_common::DisplayFactory * getDisplayFactory() const override {return nullptr;}
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr getRosNodeAbstraction() const override {return {};}
  void handleChar(QKeyEvent *, rviz_common::RenderPanel *) override {}
  void handleMouseEvent(const rviz_common::ViewportMouseEvent &) override {}
  rviz_common::ToolManager * getToolManager() const override {return nullptr;}
  rviz_common::ViewManager * getViewManager() const override {return nullptr;}
  rviz_common::transformation::TransformationManager * getTransformationManager() override {return nullptr;}
  rviz_common::DisplayGroup * getRootDisplayGroup() const override {return nullptr;}
  uint32_t getDefaultVisibilityBit() const override {return 0;}
  rviz_common::BitAllocator * visibilityBits() override {return nullptr;}
  void setStatus(const QString &) override {}
  QString getHelpPath() const override {return {};}
  std::shared_ptr<rclcpp::Clock> getClock() override {return {};}
  void lockRender() override {}
  void unlockRender() override {}
  void queueRender() override {}
};

class ModePanelHarness : public env_panel::DynamicShipPanel
{
  void onInitialize() override {}  // Test widget signals without starting service clients.
};

TEST(PanelInteraction, ShipTypeButtonsAreExclusiveAndUpdatePlacementMode)
{
  TestContext context;
  ModePanelHarness panel;
  panel.initialize(&context);
  auto sim = panel.findChild<QPushButton *>("sim_ship_mode");
  auto virt = panel.findChild<QPushButton *>("virtual_ship_mode");
  ASSERT_NE(sim, nullptr);
  ASSERT_NE(virt, nullptr);
  EXPECT_EQ(sim->text(), "仿真船");
  EXPECT_EQ(virt->text(), "虚拟船");
  EXPECT_TRUE(sim->isChecked());
  EXPECT_FALSE(virt->isChecked());
  EXPECT_EQ(panel.findChild<QComboBox *>("spawn_mode"), nullptr);
  virt->click();
  EXPECT_FALSE(sim->isChecked());
  EXPECT_TRUE(virt->isChecked());
  EXPECT_TRUE(context.property("env_panel.virtual_ship").toBool());
  virt->click();
  EXPECT_TRUE(virt->isChecked());
  sim->click();
  EXPECT_TRUE(sim->isChecked());
  EXPECT_FALSE(virt->isChecked());
  EXPECT_FALSE(context.property("env_panel.virtual_ship").toBool());
  sim->click();
  EXPECT_TRUE(sim->isChecked());
}

TEST(PanelInteraction, PlacementNotesStayAtBottomOfBothPanels)
{
  env_panel::DynamicShipPanel ship;
  env_panel::StormFieldPanel storm;
  for (auto panel : {static_cast<rviz_common::Panel *>(&ship),
      static_cast<rviz_common::Panel *>(&storm)})
  {
    auto note = panel->findChild<QLabel *>("placement_note");
    ASSERT_NE(note, nullptr);
    EXPECT_EQ(panel->layout()->itemAt(panel->layout()->count() - 1)->widget(), note);
    EXPECT_TRUE(note->wordWrap());
    EXPECT_TRUE(note->text().contains("单击"));
    EXPECT_TRUE(note->text().contains("拖拽"));
  }
}

TEST(PanelInteraction, OnlyOneShipPlacementToolIsRegistered)
{
  pluginlib::ClassLoader<rviz_common::Tool> loader("rviz_common", "rviz_common::Tool");
  EXPECT_TRUE(loader.isClassAvailable("env_panel/DynamicShipTool"));
  EXPECT_FALSE(loader.isClassAvailable("env_panel/VirtualShipPoint"));
  EXPECT_TRUE(loader.isClassAvailable("env_panel/StormPoint"));
  auto ship = loader.createSharedInstance("env_panel/DynamicShipTool");
  auto storm = loader.createSharedInstance("env_panel/StormPoint");
  EXPECT_EQ(ship->getShortcutKey(), 'd');
  EXPECT_EQ(storm->getShortcutKey(), 's');
}
}  // namespace
