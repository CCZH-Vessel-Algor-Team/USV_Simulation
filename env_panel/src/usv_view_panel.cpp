#include "env_panel/usv_view_panel.hpp"

#include <string>

#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QDebug>
#include <QVariant>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>

namespace env_panel
{

namespace
{

constexpr char kOwnShipFrame[] = "usv_1/base_link";

// TopDownOrtho scale is pixels per metre. These values deliberately start
// with a wide field of view; RViz keeps native pan/rotate/zoom interaction
// after the one-shot preset has been applied.
constexpr double kGlobalTopDownScale = 1.0;
constexpr double kFollowTopDownScale = 4.0;

constexpr double kThirdPersonDistance = 120.0;
constexpr double kFollowPitch = 0.78;
constexpr double kRearYaw = 3.14159265358979323846;

}  // namespace

UsvViewPanel::UsvViewPanel(QWidget * parent)
: rviz_common::Panel(parent),
  view_manager_(nullptr),
  global_top_down_button_(nullptr),
  follow_top_down_button_(nullptr),
  third_person_button_(nullptr)
{
  setupUi();
}

void UsvViewPanel::setupUi()
{
  auto * main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(12, 10, 12, 12);
  main_layout->setSpacing(10);

  setStyleSheet(
    "QGroupBox { font-weight: 600; margin-top: 10px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
    "QPushButton { min-height: 30px; padding: 4px 10px; }");

  auto * view_group = new QGroupBox("快速视角");
  auto * button_layout = new QGridLayout;
  button_layout->setContentsMargins(12, fontMetrics().height() + 8, 12, 10);
  button_layout->setHorizontalSpacing(10);
  button_layout->setVerticalSpacing(8);

  global_top_down_button_ = new QPushButton("全局俯视");
  follow_top_down_button_ = new QPushButton("跟随俯视");
  third_person_button_ = new QPushButton("斜后跟随");

  global_top_down_button_->setToolTip(
    "定位到 map 全局远景，之后可自由平移和缩放");
  follow_top_down_button_->setToolTip("以本船为中心的北向俯视远景");
  third_person_button_->setToolTip("跟随本船位置和航向的斜后远景");

  button_layout->addWidget(global_top_down_button_, 0, 0);
  button_layout->addWidget(follow_top_down_button_, 0, 1);
  button_layout->addWidget(third_person_button_, 0, 2);
  view_group->setLayout(button_layout);
  main_layout->addWidget(view_group);

  auto * target_label = new QLabel(
    QString("跟随目标：%1").arg(kOwnShipFrame));
  target_label->setWordWrap(true);
  main_layout->addWidget(target_label);

  main_layout->addStretch(1);

  connect(
    global_top_down_button_, &QPushButton::clicked,
    this, &UsvViewPanel::showGlobalTopDown);
  connect(
    follow_top_down_button_, &QPushButton::clicked,
    this, &UsvViewPanel::showFollowTopDown);
  connect(
    third_person_button_, &QPushButton::clicked,
    this, &UsvViewPanel::showThirdPersonFollow);

  setLayout(main_layout);
}

void UsvViewPanel::onInitialize()
{
  auto * display_context = getDisplayContext();
  view_manager_ = display_context ? display_context->getViewManager() : nullptr;
  const bool ready = view_manager_ != nullptr;
  global_top_down_button_->setEnabled(ready);
  follow_top_down_button_->setEnabled(ready);
  third_person_button_->setEnabled(ready);
  if (!ready) {
    qWarning() << "UsvViewPanel: RViz 视角管理器不可用";
  }
}

rviz_common::ViewController * UsvViewPanel::switchController(
  const QString & class_id)
{
  if (!view_manager_) {
    qWarning() << "UsvViewPanel: RViz 视角管理器不可用";
    return nullptr;
  }

  view_manager_->setCurrentViewControllerType(class_id);
  return view_manager_->getCurrent();
}

bool UsvViewPanel::setViewProperty(
  rviz_common::ViewController * controller,
  const QString & name,
  const QVariant & value)
{
  if (!controller) {
    return false;
  }
  auto * property = controller->subProp(name);
  if (!property || property->getName() != name) {
    return false;
  }
  property->setValue(value);
  return true;
}

bool UsvViewPanel::setVectorProperty(
  rviz_common::ViewController * controller,
  const QString & name,
  double x, double y, double z)
{
  if (!controller) {
    return false;
  }
  auto * vector_property = controller->subProp(name);
  if (!vector_property || vector_property->getName() != name) {
    return false;
  }
  vector_property->subProp("X")->setValue(x);
  vector_property->subProp("Y")->setValue(y);
  vector_property->subProp("Z")->setValue(z);
  return true;
}

void UsvViewPanel::showGlobalTopDown()
{
  auto * display_context = getDisplayContext();
  if (!display_context || !display_context->getFrameManager()) {
    qWarning() << "UsvViewPanel: 全局俯视切换失败：TF 管理器不可用";
    return;
  }

  Ogre::Vector3 own_ship_position;
  Ogre::Quaternion own_ship_orientation;
  if (!display_context->getFrameManager()->getTransform(
      std::string(kOwnShipFrame), own_ship_position, own_ship_orientation))
  {
    qWarning() << "UsvViewPanel: 全局俯视切换失败：本船 TF 不可用";
    return;
  }

  const QString fixed_frame = display_context->getFixedFrame();
  auto * controller = switchController("rviz_default_plugins/TopDownOrtho");
  bool ok = controller != nullptr;
  ok &= setViewProperty(controller, "Target Frame", fixed_frame);
  ok &= setViewProperty(controller, "Scale", kGlobalTopDownScale);
  ok &= setViewProperty(controller, "Angle", 0.0);
  ok &= setViewProperty(controller, "X", own_ship_position.x);
  ok &= setViewProperty(controller, "Y", own_ship_position.y);
  if (!ok) {
    qWarning() << "UsvViewPanel: 全局俯视属性设置失败";
  }
}

void UsvViewPanel::showFollowTopDown()
{
  auto * controller = switchController("rviz_default_plugins/TopDownOrtho");
  bool ok = controller != nullptr;
  ok &= setViewProperty(controller, "Target Frame", QString(kOwnShipFrame));
  ok &= setViewProperty(controller, "Scale", kFollowTopDownScale);
  ok &= setViewProperty(controller, "Angle", 0.0);
  ok &= setViewProperty(controller, "X", 0.0);
  ok &= setViewProperty(controller, "Y", 0.0);
  if (!ok) {
    qWarning() << "UsvViewPanel: 跟随俯视属性设置失败";
  }
}

void UsvViewPanel::showThirdPersonFollow()
{
  auto * controller = switchController("rviz_default_plugins/ThirdPersonFollower");
  bool ok = controller != nullptr;
  ok &= setViewProperty(controller, "Target Frame", QString(kOwnShipFrame));
  ok &= setVectorProperty(controller, "Focal Point", 0.0, 0.0, 0.0);
  ok &= setViewProperty(controller, "Distance", kThirdPersonDistance);
  ok &= setViewProperty(controller, "Pitch", kFollowPitch);
  ok &= setViewProperty(controller, "Yaw", kRearYaw);
  if (!ok) {
    qWarning() << "UsvViewPanel: 斜后跟随属性设置失败";
  }
}

}  // namespace env_panel

PLUGINLIB_EXPORT_CLASS(env_panel::UsvViewPanel, rviz_common::Panel)
