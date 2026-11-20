#include "env_panel/target_list.hpp"

#include <thread>

#include <QCoreApplication>
#include <gtest/gtest.h>

namespace
{
void flushNames()
{
  QCoreApplication::sendPostedEvents(nullptr, QEvent::MetaCall);
}

TEST(TargetNames, DecodesJsonNamesOnGuiThread)
{
  QListWidget list;
  std::thread callback([&list]() {
    env_panel::updateTargetNames(&list, R"(["target\"1","storm\\2","\u8239"])");
  });
  callback.join();
  EXPECT_EQ(list.count(), 0);
  flushNames();
  ASSERT_EQ(list.count(), 3);
  EXPECT_EQ(list.item(0)->text(), "target\"1");
  EXPECT_EQ(list.item(1)->text(), "storm\\2");
  EXPECT_EQ(list.item(2)->text(), QString::fromUtf8("船"));
}

TEST(TargetNames, KeepsSelectionUntilSelectedTargetDisappears)
{
  QListWidget list;
  env_panel::updateTargetNames(&list, R"(["a","b"])");
  flushNames();
  list.setCurrentRow(1);
  auto selected = list.currentItem();
  env_panel::updateTargetNames(&list, R"(["a","b"])");
  flushNames();
  EXPECT_EQ(list.currentItem(), selected);
  env_panel::updateTargetNames(&list, R"(["b","c"])");
  flushNames();
  ASSERT_NE(list.currentItem(), nullptr);
  EXPECT_EQ(list.currentItem()->text(), "b");
  env_panel::updateTargetNames(&list, R"(["c"])");
  flushNames();
  EXPECT_EQ(list.currentItem(), nullptr);
}

TEST(TargetNames, RejectsMalformedListsAndAcceptsClear)
{
  QListWidget list;
  list.addItem("existing");
  for (const auto invalid : {"bad json", "{}", "[null]", "[1]", "[\"a\",false]"}) {
    env_panel::updateTargetNames(&list, invalid);
    flushNames();
    ASSERT_EQ(list.count(), 1);
    EXPECT_EQ(list.item(0)->text(), "existing");
  }
  env_panel::updateTargetNames(&list, "[]");
  flushNames();
  EXPECT_EQ(list.count(), 0);
}

TEST(TargetNames, CancelsPendingUpdatesWhenWidgetIsDestroyed)
{
  auto list = new QListWidget;
  env_panel::updateTargetNames(list, R"(["pending"])");
  delete list;
  flushNames();
}
}  // namespace
