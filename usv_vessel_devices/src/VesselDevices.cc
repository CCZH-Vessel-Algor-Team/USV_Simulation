// Copyright 2026 CCZH Vessel Algorithm Team
// Licensed under the Apache License, Version 2.0.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <rclcpp/rclcpp.hpp>
#include <sdf/Material.hh>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "usv_vessel_devices/MastControl.hh"
#include "usv_vessel_devices/LampNames.hh"
#include "usv_vessel_devices/VisualName.hh"
#include "usv_vessel_devices/msg/device_state.hpp"
#include "usv_vessel_devices/srv/set_mast_position.hpp"

namespace usv_vessel_devices
{
class VesselDevices final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate, public gz::sim::ISystemReset
{
  using SetBool = std_srvs::srv::SetBool;
  using SetMast = srv::SetMastPosition;
  struct Lamp
  {
    std::string name;
    std::string link;
    std::string visualName{"lamp"};
    gz::math::Color color;
    gz::sim::Entity visual{gz::sim::kNullEntity};
    bool requested{false};
    bool applied{false};
    bool dirty{true};
  };

public:
  ~VesselDevices() override
  {
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
      jointName_ = sdf->Get<std::string>("mast_joint");
      upper_ = sdf->Get<double>("mast_max_angle");
      speed_ = sdf->Get<double>("mast_speed");
      tolerance_ = sdf->Get<double>("mast_tolerance");
      if (!std::isfinite(upper_) || upper_ <= 0.0 || upper_ > M_PI ||
        !std::isfinite(speed_) || speed_ <= 0.0 ||
        !std::isfinite(tolerance_) || tolerance_ <= 0.0 || tolerance_ >= upper_)
        throw std::invalid_argument("invalid mast limits, speed or tolerance");

      auto config = sdf->Clone();
      if (!config->HasElement("lamp")) throw std::invalid_argument("missing lamps");
      for (auto lamp = config->GetElement("lamp"); lamp;
        lamp = lamp->GetNextElement("lamp"))
      {
        Lamp item;
        item.name = lamp->Get<std::string>("name");
        item.link = lamp->Get<std::string>("link");
        if (lamp->HasElement("visual")) item.visualName = lamp->Get<std::string>("visual");
        item.color = lamp->Get<gz::math::Color>("color");
        lamps_.push_back(item);
      }
      std::vector<std::string> names;
      for (const auto &lamp : lamps_) names.push_back(lamp.name);
      if (!ValidLampNames(names))
        throw std::invalid_argument("expected the five legacy lamps followed by unique valid extra lamp names");

      // Private context per model: never initialize/shutdown Gazebo's global ROS context.
      context_ = std::make_shared<rclcpp::Context>();
      context_->init(0, nullptr);
      rclcpp::NodeOptions options;
      options.context(context_).use_global_arguments(false);
      node_ = std::make_shared<rclcpp::Node>("vessel_devices", "/" + ns_ + "/devices", options);
      publisher_ = node_->create_publisher<msg::DeviceState>(
        "state", rclcpp::QoS(1).reliable().transient_local());
      for (size_t i = 0; i < lamps_.size(); ++i)
      {
        const auto base = "lights/" + lamps_[i].name;
        lightSubscriptions_.push_back(node_->create_subscription<std_msgs::msg::Bool>(
          base + "/cmd", rclcpp::QoS(1), [this, i](std_msgs::msg::Bool::ConstSharedPtr m)
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!ready_) {RCLCPP_WARN(node_->get_logger(), "Lights not ready"); return;}
            lamps_[i].requested = m->data;
            lamps_[i].dirty = true;
          }));
        lightServices_.push_back(node_->create_service<SetBool>(base + "/set",
          [this, i](const std::shared_ptr<SetBool::Request> request,
            std::shared_ptr<SetBool::Response> response)
          {
            std::lock_guard<std::mutex> lock(mutex_);
            response->success = ready_;
            response->message = ready_ ? "Accepted; observe devices/state" : status_;
            if (!ready_) return;
            lamps_[i].requested = request->data;
            lamps_[i].dirty = true;
          }));
      }
      mastSubscription_ = node_->create_subscription<std_msgs::msg::Float64>(
        "mast/cmd", rclcpp::QoS(1), [this](std_msgs::msg::Float64::ConstSharedPtr m)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (ready_ && ValidTarget(m->data, upper_)) target_ = m->data;
          else RCLCPP_WARN(node_->get_logger(), "Mast command rejected: not ready or outside limits");
        });
      mastService_ = node_->create_service<SetMast>("mast/set_position",
        [this](const std::shared_ptr<SetMast::Request> request,
          std::shared_ptr<SetMast::Response> response)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          response->success = ready_ && ValidTarget(request->position, upper_);
          response->message = !ready_ ? status_ : (!response->success ?
            "Position must be finite and within [0, " + std::to_string(upper_) + "] rad" :
            "Accepted; observe mast_at_target in devices/state");
          if (response->success) target_ = request->position;
        });
      // Wall-clock executor keeps services and status responsive while Gazebo is paused.
      timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this]() {Publish();});
      rclcpp::ExecutorOptions executorOptions;
      executorOptions.context = context_;
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executorOptions);
      executor_->add_node(node_);
      thread_ = std::thread([this]()
        {
          // A bounded spin also handles deletion immediately after Configure,
          // before the thread has entered its first executor wait.
          while (!stopping_ && context_->is_valid())
            executor_->spin_once(std::chrono::milliseconds(100));
        });
      configured_ = true;
      RCLCPP_INFO(node_->get_logger(), "%zu lights and folding mast enabled for %s", lamps_.size(), ns_.c_str());
    }
    catch (const std::exception &e)
    {
      gzerr << "VesselDevices configuration failed: " << e.what() << std::endl;
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (joint_ == gz::sim::kNullEntity)
    {
      joint_ = model_.JointByName(ecm, jointName_);
      if (joint_ != gz::sim::kNullEntity)
      {
        gz::sim::Joint(joint_).EnablePositionCheck(ecm);
        gz::sim::Joint(joint_).EnableVelocityCheck(ecm);
      }
    }
    for (auto &lamp : lamps_)
    {
      if (lamp.visual == gz::sim::kNullEntity)
      {
        const auto link = model_.LinkByName(ecm, lamp.link);
        for (const auto visual : ecm.ChildrenByComponents(link, gz::sim::components::Visual()))
        {
          const auto name = ecm.Component<gz::sim::components::Name>(visual);
          if (name && MatchesVisualName(name->Data(), lamp.visualName))
          {
            lamp.visual = visual;
            break;
          }
        }
      }
      if (lamp.visual == gz::sim::kNullEntity || !lamp.dirty) continue;
      const auto c = lamp.requested ? lamp.color : gz::math::Color(0.025f, 0.025f, 0.025f, 1.f);
      sdf::Material material;
      material.SetAmbient(c);
      material.SetDiffuse(c);
      material.SetSpecular(gz::math::Color::Black);
      material.SetEmissive(lamp.requested ? lamp.color : gz::math::Color::Black);
      // Material persists for late GUI clients; VisualCmd updates sensor rendering too.
      ecm.SetComponentData<gz::sim::components::Material>(lamp.visual, material);
      ecm.SetChanged(lamp.visual, gz::sim::components::Material::typeId,
        gz::sim::ComponentState::OneTimeChange);
      gz::msgs::Visual command;
      command.set_id(lamp.visual);
      command.set_name(ecm.Component<gz::sim::components::Name>(lamp.visual)->Data());
      *command.mutable_material() = gz::sim::convert<gz::msgs::Material>(material);
      ecm.SetComponentData<gz::sim::components::VisualCmd>(lamp.visual, command);
      ecm.SetChanged(lamp.visual, gz::sim::components::VisualCmd::typeId,
        gz::sim::ComponentState::OneTimeChange);
      lamp.applied = lamp.requested;
      lamp.dirty = false;
    }
    if (joint_ != gz::sim::kNullEntity && !info.paused)
    {
      auto joint = gz::sim::Joint(joint_);
      const auto position = joint.Position(ecm);
      const auto actualVelocity = joint.Velocity(ecm);
      const double dt = std::chrono::duration<double>(info.dt).count();
      const double velocity = position && !position->empty() ?
        MastVelocity(position->front(), target_, speed_, tolerance_, dt) : 0.0;
      const double effort = actualVelocity && !actualVelocity->empty() ?
        MastEffort(velocity, actualVelocity->front(), dt, integral_) : 0.0;
      joint.SetForce(ecm, {effort});
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    simNanos_ = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    const auto position = gz::sim::Joint(joint_).Position(ecm);
    const auto velocity = gz::sim::Joint(joint_).Velocity(ecm);
    position_ = position && !position->empty() ? position->front() :
      std::numeric_limits<double>::quiet_NaN();
    const bool validVelocity = velocity && !velocity->empty() && std::isfinite(velocity->front());
    moving_ = !info.paused && validVelocity && std::abs(velocity->front()) > 0.001;
    ready_ = std::isfinite(position_) && validVelocity;
    status_ = ready_ ? "ready" : "waiting for mast joint physics feedback";
    for (const auto &lamp : lamps_)
      if (lamp.visual == gz::sim::kNullEntity)
      {
        ready_ = false;
        status_ = "waiting for lamp visual: " + lamp.link;
        break;
      }
  }

  void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = 0.0;
    integral_ = 0.0;
    ready_ = false;
    status_ = "waiting for physics after reset";
    for (auto &lamp : lamps_) {lamp.requested = false; lamp.dirty = true;}
  }

private:
  void Publish()
  {
    msg::DeviceState state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state.header.stamp = rclcpp::Time(simNanos_, RCL_ROS_TIME);
      state.header.frame_id = ns_ + "/base_link";
      state.ready = ready_;
      state.status = status_;
      for (const auto &lamp : lamps_)
      {
        state.light_names.push_back(lamp.name);
        state.lights_on.push_back(lamp.applied);
      }
      state.mast_position = position_;
      state.mast_target = target_;
      state.mast_at_target = ready_ && std::abs(position_ - target_) <= tolerance_;
      state.mast_moving = moving_;
    }
    publisher_->publish(state);
  }

  gz::sim::Model model_;
  gz::sim::Entity joint_{gz::sim::kNullEntity};
  std::string ns_, jointName_;
  double upper_{M_PI / 2}, speed_{0.3}, tolerance_{0.005}, target_{0.0};
  double integral_{0.0};
  double position_{std::numeric_limits<double>::quiet_NaN()};
  int64_t simNanos_{0};
  bool configured_{false}, ready_{false}, moving_{false};
  std::atomic<bool> stopping_{false};
  std::string status_{"waiting for simulation entities"};
  std::vector<Lamp> lamps_;
  std::mutex mutex_;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread thread_;
  rclcpp::Publisher<msg::DeviceState>::SharedPtr publisher_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> lightSubscriptions_;
  std::vector<rclcpp::Service<SetBool>::SharedPtr> lightServices_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr mastSubscription_;
  rclcpp::Service<SetMast>::SharedPtr mastService_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace usv_vessel_devices

GZ_ADD_PLUGIN(usv_vessel_devices::VesselDevices, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
  gz::sim::ISystemPostUpdate, gz::sim::ISystemReset)
