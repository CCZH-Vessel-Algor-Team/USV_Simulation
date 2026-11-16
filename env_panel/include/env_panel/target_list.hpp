#ifndef ENV_PANEL__TARGET_LIST_HPP_
#define ENV_PANEL__TARGET_LIST_HPP_

#include <string>

#include <QJsonArray>
#include <QJsonDocument>
#include <QListWidget>
#include <QMetaObject>

namespace env_panel
{

// ROS callbacks may run off the GUI thread. Queue all widget access on its owner.
inline void updateTargetNames(QListWidget * list, const std::string & json)
{
  const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(json));
  if (!document.isArray()) {
    return;
  }
  QStringList names;
  for (const auto value : document.array()) {
    if (!value.isString()) {
      return;
    }
    names.append(value.toString());
  }
  QMetaObject::invokeMethod(list, [list, names]() {
    QStringList current;
    for (int i = 0; i < list->count(); ++i) {
      current.append(list->item(i)->text());
    }
    if (current == names) {
      return;
    }
    const auto selected = list->currentItem() ? list->currentItem()->text() : QString();
    list->clear();
    list->addItems(names);
    const auto matches = list->findItems(selected, Qt::MatchExactly);
    if (!matches.isEmpty()) {
      list->setCurrentItem(matches.front());
    }
  }, Qt::QueuedConnection);
}

}  // namespace env_panel

#endif  // ENV_PANEL__TARGET_LIST_HPP_
