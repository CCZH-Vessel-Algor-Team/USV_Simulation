// Copyright 2026 CCZH Vessel Algorithm Team
// Licensed under the Apache License, Version 2.0.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vrx_gz/Wavefield.hh>

#include "usv_vessel_devices/MastControl.hh"
#include "usv_vessel_devices/ThrusterTiltControl.hh"
#include "usv_vessel_devices/ThrusterTiltLimits.hh"
#include "usv_vessel_devices/PropulsionPhysics.hh"
#include "usv_vessel_devices/msg/thruster_tilt_state.hpp"
#include "usv_vessel_devices/srv/set_thruster_tilt.hpp"

namespace usv_vessel_devices
{
class ThrusterTilt final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate, public gz::sim::ISystemReset
{
  using SetTilt = srv::SetThrusterTilt;
  struct Outboard
  {
    std::string name, jointName, inputTopic;
    gz::sim::Entity joint{gz::sim::kNullEntity}, propJoint{gz::sim::kNullEntity};
    gz::sim::Entity propLink{gz::sim::kNullEntity}, engineLink{gz::sim::kNullEntity};
    gz::sim::Entity tiltLink{gz::sim::kNullEntity};
    gz::transport::Node::Publisher output, speedOutput;
    double position{std::numeric_limits<double>::quiet_NaN()};
    double velocity{std::numeric_limits<double>::quiet_NaN()};
    double target{0.0}, command{0.0}, applied{0.0};
    double lockedTarget{0.0};
    double lastPublished{std::numeric_limits<double>::quiet_NaN()};
    std::chrono::steady_clock::duration lastPublishTime{0};
    std::chrono::steady_clock::duration zeroSince{0};
    bool ready{false}, moving{false}, thrustEnabled{false};
    bool locked{false};
    double reference{0.0}, referenceVelocity{0.0}, integral{0.0};
    double immersion{0.0}, waterThrust{0.0}, effort{0.0}, propSpeed{0.0};
    bool referenceInitialized{false}, saturated{false};
  };

public:
  ~ThrusterTilt() override
  {
    for (const auto &side : sides_)
      if (!side.inputTopic.empty()) transport_.Unsubscribe(side.inputTopic);
    stopping_ = true;
    if (executor_) executor_->cancel();
    if (thread_.joinable()) thread_.join();
    if (context_ && context_->is_valid()) context_->shutdown("model removed");
  }

  void Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    try
    {
      model_ = gz::sim::Model(entity);
      ns_ = sdf->Get<std::string>("namespace");
      upper_ = sdf->Get<double>("max_angle");
      speed_ = sdf->Get<double>("speed");
      tolerance_ = sdf->Get<double>("tolerance");
      effortLimit_ = sdf->Get<double>("effort", 10000.0).first;
      fluidLevel_ = sdf->Get<double>("fluid_level", 0.0).first;
      diameter_ = sdf->Get<double>("propeller_diameter", 0.2).first;
      if (!ValidTiltLimit(upper_) ||
        !std::isfinite(speed_) || speed_ <= 0.0 ||
        !std::isfinite(tolerance_) || tolerance_ <= 0.0 || tolerance_ >= upper_ ||
        !std::isfinite(effortLimit_) || effortLimit_ <= 0.0 ||
        !std::isfinite(fluidLevel_) || !std::isfinite(diameter_) || diameter_ <= 0.0)
        throw std::invalid_argument("invalid thruster tilt limits, speed or tolerance");

      context_ = std::make_shared<rclcpp::Context>();
      context_->init(0, nullptr);
      rclcpp::NodeOptions options;
      options.context(context_).use_global_arguments(false);
      node_ = std::make_shared<rclcpp::Node>("thruster_tilt", "/" + ns_ + "/devices", options);
      publisher_ = node_->create_publisher<msg::ThrusterTiltState>(
        "thrusters/state", rclcpp::QoS(1).reliable().transient_local());
      diagnostics_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "thrusters/physics", rclcpp::QoS(1));
      wavefield_.Load(sdf);
      transport_.Subscribe(wavefield_.Topic(), &ThrusterTilt::OnWavefield, this);
      const std::array<std::string, 2> names{"left", "right"};
      for (size_t i = 0; i < sides_.size(); ++i)
      {
        auto &side = sides_[i];
        side.name = names[i];
        side.jointName = ns_ + "/" + side.name + "_tilt_joint";
        side.inputTopic = "/" + ns_ + "/thrusters/" + side.name + "/thrust";
        side.output = transport_.Advertise<gz::msgs::Double>(
          "/" + ns_ + "/devices/thrusters/" + side.name + "/thrust_applied");
        side.speedOutput = transport_.Advertise<gz::msgs::Double>(
          "/" + ns_ + "/devices/thrusters/" + side.name + "/thrust_applied/ang_vel");
        if (!side.output || !side.speedOutput || !transport_.Subscribe(side.inputTopic,
            std::function<void(const gz::msgs::Double &)>([this, i](const gz::msgs::Double &message)
            {
              std::lock_guard<std::mutex> lock(mutex_);
              // Match the Gazebo native force command limits, independently of tilt.
              sides_[i].command = std::isfinite(message.data()) ?
                std::clamp(message.data(), -1000.0, 8529.75) : 0.0;
            })))
          throw std::runtime_error("could not create thruster tilt transport endpoints");
        CreateCommands(side.name, {i});
      }
      CreateCommands("both", {0, 1});
      timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this]() {Publish();});
      rclcpp::ExecutorOptions executorOptions;
      executorOptions.context = context_;
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executorOptions);
      executor_->add_node(node_);
      thread_ = std::thread([this]()
        {
          while (!stopping_ && context_->is_valid())
            executor_->spin_once(std::chrono::milliseconds(100));
        });
      configured_ = true;
      RCLCPP_INFO(node_->get_logger(), "Independent left/right outboard tilt enabled for %s", ns_.c_str());
    }
    catch (const std::exception &e)
    {
      // Native thrusters listen only to internal outputs: failure remains thrust-off.
      gzerr << "ThrusterTilt configuration failed: " << e.what() << std::endl;
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &side : sides_)
    {
      if (side.joint == gz::sim::kNullEntity)
      {
        side.joint = model_.JointByName(ecm, side.jointName);
        if (side.joint != gz::sim::kNullEntity)
        {
          gz::sim::Joint(side.joint).EnablePositionCheck(ecm);
          gz::sim::Joint(side.joint).EnableVelocityCheck(ecm);
        }
      }
      if (side.propJoint == gz::sim::kNullEntity)
        side.propJoint = model_.JointByName(ecm, ns_ + "/" + side.name + "_engine_propeller_joint");
      if (side.propLink == gz::sim::kNullEntity)
        side.propLink = model_.LinkByName(ecm, ns_ + "/" + side.name + "_propeller_link");
      if (side.engineLink == gz::sim::kNullEntity)
        side.engineLink = model_.LinkByName(ecm, ns_ + "/" + side.name + "_engine_link");
      if (side.tiltLink == gz::sim::kNullEntity)
        side.tiltLink = model_.LinkByName(ecm, ns_ + "/" + side.name + "_tilt_link");
      ReadFeedback(side, ecm, info.paused);
      if (!info.paused) UpdatePropulsion(side, info, ecm);
      SendThrust(side, info.simTime);
      if (side.ready && !info.paused)
      {
        UpdateServo(side, info, ecm);
      }
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    simNanos_ = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    for (auto &side : sides_) ReadFeedback(side, ecm, info.paused);
  }

  void Reset(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &) override
  {
    if (!configured_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &side : sides_)
    {
      side.target = side.command = 0.0;
      side.referenceInitialized = false;
      side.reference = side.referenceVelocity = side.integral = 0.0;
      side.immersion = side.waterThrust = side.effort = side.propSpeed = 0.0;
      side.ready = side.moving = side.thrustEnabled = false;
      side.position = side.velocity = std::numeric_limits<double>::quiet_NaN();
      side.lastPublished = std::numeric_limits<double>::quiet_NaN();
      SendThrust(side, info.simTime);
    }
    waveStartTime_ = -1.0;
  }

private:
  void OnWavefield(const gz::msgs::Param &message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    wavefield_.Load(message, simTime_, gz::math::Vector3d::Zero);
  }

  void UpdatePropulsion(Outboard &side, const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm)
  {
    if (!side.thrustEnabled) return;
    simTime_ = std::chrono::duration<double>(info.simTime).count();
    if (waveStartTime_ < 0.0 || simTime_ < waveStartTime_) waveStartTime_ = simTime_;
    const auto pose = gz::sim::worldPose(side.propLink, ecm);
    const auto axis = pose.Rot().RotateVector(gz::math::Vector3d::UnitX).Normalized();
    const double wave = wavefield_.ComputeDepthSimply(pose.Pos(), simTime_, waveStartTime_);
    const double depth = fluidLevel_ + wave - pose.Pos().Z();
    const double span = 0.5 * diameter_ * std::sqrt(std::max(0.0, 1.0 - axis.Z() * axis.Z()));
    side.immersion = SubmergedDiskFraction(depth, span);
    side.waterThrust = side.command * side.immersion;
    side.propSpeed = PropellerSpeed(side.command, 1000.0, 0.004422, diameter_);
    gz::sim::Joint(side.propJoint).SetVelocity(ecm, {side.propSpeed});
    gz::sim::Link(side.propLink).AddWorldWrench(ecm, axis * side.waterThrust,
      gz::math::Vector3d::Zero);
  }

  void UpdateServo(Outboard &side, const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm)
  {
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (!(dt > 0.0) || !std::isfinite(dt)) return;
    if (!side.referenceInitialized)
    {
      side.reference = side.position;
      side.referenceVelocity = 0.0;
      side.referenceInitialized = true;
    }
    const double error = side.target - side.reference;
    const double desiredVelocity = std::clamp(4.0 * error, -speed_, speed_);
    const double acceleration = 0.5;
    side.referenceVelocity = std::clamp(desiredVelocity,
      side.referenceVelocity - acceleration * dt, side.referenceVelocity + acceleration * dt);
    const double step = side.referenceVelocity * dt;
    if (std::abs(step) >= std::abs(error) && step * error >= 0.0)
    {
      side.reference = side.target;
      side.referenceVelocity = 0.0;
    }
    else side.reference += step;

    // Feed-forward moments of actual applied force and gravity about the
    // tilt hinge. This uses the same simulation step as the thrust calculation.
    if (side.tiltLink == gz::sim::kNullEntity) return;
    // The tilt link origin coincides with the hinge. Joint poses are relative
    // to the child in the ECM and are not a reliable world-space hinge pose.
    const auto hinge = gz::sim::worldPose(side.tiltLink, ecm);
    const auto hingeAxis = hinge.Rot().RotateVector(gz::math::Vector3d::UnitY).Normalized();
    double load = 0.0;
    if (side.propLink != gz::sim::kNullEntity)
    {
      const auto prop = gz::sim::worldPose(side.propLink, ecm);
      const auto force = prop.Rot().RotateVector(gz::math::Vector3d::UnitX) * side.waterThrust;
      load += (prop.Pos() - hinge.Pos()).Cross(force).Dot(hingeAxis);
    }
    const auto world = gz::sim::worldEntity(ecm);
    const auto gravity = ecm.Component<gz::sim::components::Gravity>(world);
    if (gravity)
    {
      for (const auto link : {side.engineLink, side.propLink})
      {
        const auto inertia = ecm.Component<gz::sim::components::Inertial>(link);
        if (!inertia) continue;
        const auto pose = gz::sim::worldPose(link, ecm);
        const auto com = pose.Pos() + pose.Rot().RotateVector(inertia->Data().Pose().Pos());
        const auto weight = gravity->Data() * inertia->Data().MassMatrix().Mass();
        load += (com - hinge.Pos()).Cross(weight).Dot(hingeAxis);
      }
    }
    const double positionError = side.reference - side.position;
    const double candidateIntegral = std::clamp(side.integral + positionError * dt, -2.0, 2.0);
    const double raw = -load + 800.0 * positionError +
      150.0 * (side.referenceVelocity - side.velocity) + 100.0 * candidateIntegral;
    side.effort = std::clamp(raw, -effortLimit_, effortLimit_);
    side.saturated = std::abs(raw) > effortLimit_;
    if (!side.saturated || raw * positionError < 0.0) side.integral = candidateIntegral;
    gz::sim::Joint(side.joint).SetForce(ecm, {side.effort});
  }

  void CreateCommands(const std::string &name, const std::vector<size_t> &indices)
  {
    const auto base = "thrusters/" + name + "/tilt";
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::Float64>(
      base + "/cmd", rclcpp::QoS(1), [this, indices](std_msgs::msg::Float64::ConstSharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto error = AcceptTarget(indices, message->data);
        if (!error.empty()) RCLCPP_WARN(node_->get_logger(), "Tilt command rejected: %s", error.c_str());
      }));
    services_.push_back(node_->create_service<SetTilt>(base + "/set_position",
      [this, indices](const std::shared_ptr<SetTilt::Request> request,
        std::shared_ptr<SetTilt::Response> response)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto error = AcceptTarget(indices, request->position);
        response->success = error.empty();
        response->message = error.empty() ? "Accepted; observe devices/thrusters/state" : error;
      }));
  }

  // Called under mutex. Validate all sides before changing either one.
  std::string AcceptTarget(const std::vector<size_t> &indices, double target)
  {
    if (!ValidTiltTarget(target, upper_))
      return "Position must be finite and within [0, " + std::to_string(upper_) + "] rad";
    for (const auto i : indices)
      if (!sides_[i].ready) return "Waiting for " + sides_[i].name + " tilt joint physics feedback";
    for (const auto i : indices)
    {
      sides_[i].target = target;
    }
    return {};
  }

  void ReadFeedback(Outboard &side, const gz::sim::EntityComponentManager &ecm, bool paused)
  {
    const auto joint = gz::sim::Joint(side.joint);
    const auto position = joint.Position(ecm);
    const auto velocity = joint.Velocity(ecm);
    side.position = position && !position->empty() ? position->front() :
      std::numeric_limits<double>::quiet_NaN();
    side.velocity = velocity && !velocity->empty() ? velocity->front() :
      std::numeric_limits<double>::quiet_NaN();
    side.ready = std::isfinite(side.position) && std::isfinite(side.velocity);
    side.moving = !paused && side.ready && std::abs(side.velocity) > 0.001;
    side.thrustEnabled = side.propLink != gz::sim::kNullEntity &&
      side.propJoint != gz::sim::kNullEntity;
  }

  void SendThrust(Outboard &side, std::chrono::steady_clock::duration simTime)
  {
    const double command = side.command;
    if (command != 0.0 || side.lastPublished != 0.0) side.zeroSince = simTime;
    // Changes (especially inhibition) are sent immediately; refresh at 20 Hz
    // simulation time so late native subscribers also receive the current value.
    if (command != side.lastPublished || simTime < side.lastPublishTime ||
      simTime - side.lastPublishTime >= std::chrono::milliseconds(50))
    {
      gz::msgs::Double message;
      message.set_data(command);
      side.output.Publish(message);
      gz::msgs::Double speedMessage;
      speedMessage.set_data(side.propSpeed);
      side.speedOutput.Publish(speedMessage);
      side.lastPublished = command;
      side.lastPublishTime = simTime;
      side.applied = command;
    }
  }

  void Publish()
  {
    msg::ThrusterTiltState state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state.header.stamp = rclcpp::Time(simNanos_, RCL_ROS_TIME);
      state.header.frame_id = ns_ + "/base_link";
      state.ready = sides_[0].ready && sides_[1].ready;
      state.status = state.ready ? "ready" : "waiting for tilt joint physics feedback";
      for (const auto &side : sides_)
      {
        state.thruster_names.push_back(side.name);
        state.position.push_back(side.position);
        state.target.push_back(side.target);
        state.moving.push_back(side.moving);
        state.at_target.push_back(side.ready && std::abs(side.position - side.target) <= tolerance_ &&
          std::abs(side.velocity) <= 0.001);
        state.thrust_enabled.push_back(side.thrustEnabled);
        state.applied_thrust.push_back(side.applied);
      }
    }
    publisher_->publish(state);
    std_msgs::msg::Float64MultiArray physics;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto &side : sides_)
      {
        physics.data.insert(physics.data.end(), {side.command, side.propSpeed,
          side.immersion, side.waterThrust, side.effort, side.saturated ? 1.0 : 0.0});
      }
    }
    diagnostics_->publish(physics);
  }

  gz::sim::Model model_;
  std::string ns_;
  std::array<Outboard, 2> sides_;
  double upper_{kMaxThrusterTiltAngle}, speed_{0.25}, tolerance_{0.005};
  double effortLimit_{10000.0}, fluidLevel_{0.0}, diameter_{0.2};
  double simTime_{0.0}, waveStartTime_{-1.0};
  vrx::Wavefield wavefield_;
  int64_t simNanos_{0};
  bool configured_{false};
  std::atomic<bool> stopping_{false};
  std::mutex mutex_;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread thread_;
  rclcpp::Publisher<msg::ThrusterTiltState>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subscriptions_;
  std::vector<rclcpp::Service<SetTilt>::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr timer_;
  // Destroy transport callbacks before any of the state they reference.
  gz::transport::Node transport_;
};
}  // namespace usv_vessel_devices

GZ_ADD_PLUGIN(usv_vessel_devices::ThrusterTilt, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
  gz::sim::ISystemPostUpdate, gz::sim::ISystemReset)
