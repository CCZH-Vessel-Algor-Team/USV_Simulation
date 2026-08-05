#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/common/Console.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

namespace vrx
{
class TargetShipController final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    const auto linkName = sdf->Get<std::string>("link_name", "base_link").first;
    this->link_ = gz::sim::Link(model.LinkByName(ecm, linkName));
    if (!this->link_.Valid(ecm)) {
      gzerr << "TargetShipController link not found: " << linkName << std::endl;
      return;
    }
    gz::sim::enableComponent<gz::sim::components::WorldPose>(
      ecm, this->link_.Entity(), true);
    gz::sim::enableComponent<gz::sim::components::WorldLinearVelocity>(
      ecm, this->link_.Entity(), true);
    gz::sim::enableComponent<gz::sim::components::WorldAngularVelocity>(
      ecm, this->link_.Entity(), true);

    this->kpLinear_ = sdf->Get<double>("kp_linear", 1500.0).first;
    this->kpHeading_ = sdf->Get<double>("kp_heading", 60000.0).first;
    this->kdYaw_ = sdf->Get<double>("kd_yaw", 25000.0).first;
    this->maxForce_ = sdf->Get<double>("max_force", 8000.0).first;
    this->maxTorque_ = sdf->Get<double>("max_torque", 10000.0).first;
    const auto topic = sdf->Get<std::string>(
      "topic", "/model/" + model.Name(ecm) + "/cmd_vel").first;

    if (!this->node_.Subscribe(topic, &TargetShipController::OnCommand, this)) {
      gzerr << "Unable to subscribe to target ship command topic: "
            << topic << std::endl;
    }
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || !this->link_.Valid(ecm)) {
      return;
    }

    const auto pose = this->link_.WorldPose(ecm);
    const auto linearVelocity = this->link_.WorldLinearVelocity(ecm);
    const auto angularVelocity = this->link_.WorldAngularVelocity(ecm);
    if (!pose || !linearVelocity || !angularVelocity) {
      return;
    }

    gz::math::Vector3d desiredLinear;
    double desiredHeading = 0.0;
    double desiredYawRate = 0.0;
    bool headingControl = false;
    bool commandFresh = false;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      desiredLinear = this->desiredLinearBody_;
      desiredHeading = this->desiredHeading_;
      desiredYawRate = this->desiredYawRate_;
      headingControl = this->headingControl_;
      commandFresh = std::chrono::steady_clock::now() - this->lastCommand_ <
        std::chrono::seconds(1);
    }
    if (!commandFresh) {
      desiredLinear.Set(0.0, 0.0, 0.0);
      headingControl = false;
      desiredYawRate = 0.0;
    }

    double headingError = 0.0;
    if (headingControl) {
      const double currentHeading = pose->Rot().Yaw();
      headingError = std::atan2(
        std::sin(desiredHeading - currentHeading),
        std::cos(desiredHeading - currentHeading));
    }

    const auto linearBody = pose->Rot().Inverse() * *linearVelocity;
    const double desiredSurge =
      headingControl && std::abs(headingError) > 0.15 ? 0.0 : desiredLinear.X();
    const double surgeForce = this->kpLinear_ *
      (desiredSurge - linearBody.X());
    auto force = pose->Rot() * gz::math::Vector3d(surgeForce, 0.0, 0.0);
    const double forceLength = force.Length();
    if (forceLength > this->maxForce_) {
      force *= this->maxForce_ / forceLength;
    }

    double yawTorque = 0.0;
    if (headingControl) {
      yawTorque = this->kpHeading_ * headingError -
        this->kdYaw_ * angularVelocity->Z();
    } else {
      yawTorque = this->kdYaw_ * (desiredYawRate - angularVelocity->Z());
    }
    yawTorque = std::clamp(yawTorque, -this->maxTorque_, this->maxTorque_);
    const gz::math::Vector3d torque(0.0, 0.0, yawTorque);

    if (force.IsFinite() && torque.IsFinite()) {
      this->link_.AddWorldWrench(ecm, force, torque);
    }
  }

private:
  void OnCommand(const gz::msgs::Twist & msg)
  {
    const gz::math::Vector3d linear(
      msg.linear().x(), msg.linear().y(), 0.0);
    if (!linear.IsFinite() || !std::isfinite(msg.angular().x()) ||
      !std::isfinite(msg.angular().y()) || !std::isfinite(msg.angular().z()))
    {
      return;
    }
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->desiredLinearBody_ = linear;
    this->headingControl_ = msg.angular().x() > 0.5;
    this->desiredHeading_ = msg.angular().y();
    this->desiredYawRate_ = msg.angular().z();
    this->lastCommand_ = std::chrono::steady_clock::now();
  }

  gz::sim::Link link_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  std::mutex mutex_;
  gz::math::Vector3d desiredLinearBody_{0.0, 0.0, 0.0};
  bool headingControl_{false};
  double desiredHeading_{0.0};
  double desiredYawRate_{0.0};
  std::chrono::steady_clock::time_point lastCommand_{};
  double kpLinear_{5000.0};
  double kpHeading_{60000.0};
  double kdYaw_{25000.0};
  double maxForce_{8000.0};
  double maxTorque_{120000.0};
};
}  // namespace vrx

GZ_ADD_PLUGIN(
  vrx::TargetShipController,
  gz::sim::System,
  vrx::TargetShipController::ISystemConfigure,
  vrx::TargetShipController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::TargetShipController, "vrx::TargetShipController")
