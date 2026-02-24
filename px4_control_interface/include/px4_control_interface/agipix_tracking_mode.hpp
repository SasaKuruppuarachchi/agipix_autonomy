#pragma once

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <autonomous_flight/msg/target.hpp>

#include <px4_control_interface/agi_controller.hpp>

namespace px4_control_interface
{

class AgipixTrackingMode : public px4_ros2::ModeBase
{
public:
  explicit AgipixTrackingMode(rclcpp::Node & node)
  : ModeBase(node, px4_ros2::ModeBase::Settings{"AgiPix DDS Tracking", false})
  {
    _trajectory_sp = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    _target_topic = node.declare_parameter<std::string>("target_topic", "/autonomous_flight/target_state");
    _target_timeout_s = node.declare_parameter<double>("target_timeout_s", 0.2);
    _use_input_yaw = node.declare_parameter<bool>("use_input_yaw", true);

    const std::string controller_type =
      node.declare_parameter<std::string>("middle_level_controller", "cascaded_pid");
    const Eigen::Vector3f kp_pos = vectorParamToEigen(
      node.declare_parameter<std::vector<double>>("controller.kp_pos", {2.0, 2.0, 2.5}),
      {2.f, 2.f, 2.5f});
    const Eigen::Vector3f kd_vel = vectorParamToEigen(
      node.declare_parameter<std::vector<double>>("controller.kd_vel", {1.2, 1.2, 1.6}),
      {1.2f, 1.2f, 1.6f});
    const Eigen::Vector3f ki_pos = vectorParamToEigen(
      node.declare_parameter<std::vector<double>>("controller.ki_pos", {0.0, 0.0, 0.0}),
      {0.f, 0.f, 0.f});
    const float integral_limit = static_cast<float>(
      node.declare_parameter<double>("controller.integral_limit", 0.5));

    if (controller_type == "cascaded_pid") {
      _controller = std::make_unique<CascadedPidAgiController>(kp_pos, kd_vel, ki_pos, integral_limit);
    } else {
      _controller = std::make_unique<PassThroughAgiController>();
    }

    RCLCPP_INFO(
      node.get_logger(),
      "[px4_control_interface]: middle_level_controller=%s",
      _controller->name().c_str());

    _target_sub = node.create_subscription<autonomous_flight::msg::Target>(
      _target_topic,
      rclcpp::QoS(10),
      [this](const autonomous_flight::msg::Target::SharedPtr msg) {
        std::scoped_lock<std::mutex> lock(_target_mutex);
        _last_target = *msg;
        _last_target_rx = this->node().get_clock()->now();
        _target_received = true;
      });
  }

  void onActivate() override
  {
    _hold_position_ned = _local_position->positionNed();
    AgiControllerState state;
    state.position_ned = _local_position->positionNed();
    state.velocity_ned = _local_position->velocityNed();
    state.acceleration_ned = _local_position->accelerationNed();
    state.yaw_ned = _local_position->heading();
    _controller->reset(state);
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    autonomous_flight::msg::Target target;
    bool has_fresh_target = false;

    {
      std::scoped_lock<std::mutex> lock(_target_mutex);
      if (_target_received) {
        const double age = (node().get_clock()->now() - _last_target_rx).seconds();
        has_fresh_target = age <= _target_timeout_s;
        target = _last_target;
      }
    }

    AgiControllerState state;
    state.position_ned = _local_position->positionNed();
    state.velocity_ned = _local_position->velocityNed();
    state.acceleration_ned = _local_position->accelerationNed();
    state.yaw_ned = _local_position->heading();

    AgiControllerReference reference;
    if (has_fresh_target) {
      reference.position_ned = enuToNed(target.position.x, target.position.y, target.position.z);
      reference.velocity_ned = enuToNed(target.velocity.x, target.velocity.y, target.velocity.z);
      reference.acceleration_ned = enuToNed(target.acceleration.x, target.acceleration.y, target.acceleration.z);
      reference.yaw_ned = _use_input_yaw ? enuYawToNed(target.yaw) : state.yaw_ned;
      reference.type_mask = target.type_mask;
      _hold_position_ned = reference.position_ned;
    } else {
      reference.position_ned = _hold_position_ned;
      reference.velocity_ned = Eigen::Vector3f::Zero();
      reference.acceleration_ned = Eigen::Vector3f::Zero();
      reference.yaw_ned = state.yaw_ned;
      reference.type_mask = autonomous_flight::msg::Target::IGNORE_ACC_VEL;
    }

    _controller->setReference(reference);
    const autonomous_flight::msg::Target controlled_target = _controller->update(state, dt_s);

    const bool ignore_acc_vel = controlled_target.type_mask == autonomous_flight::msg::Target::IGNORE_ACC_VEL;
    const bool ignore_acc = controlled_target.type_mask == autonomous_flight::msg::Target::IGNORE_ACC;
    const bool velocity_priority = controlled_target.type_mask == autonomous_flight::msg::Target::IGNORE_POS_ACC;

    px4_ros2::TrajectorySetpoint sp;
    if (velocity_priority) {
      sp.withHorizontalVelocity(Eigen::Vector2f{
        static_cast<float>(controlled_target.velocity.x),
        static_cast<float>(controlled_target.velocity.y)});
      sp.withPositionZ(static_cast<float>(controlled_target.position.z));
      sp.withVelocityZ(static_cast<float>(controlled_target.velocity.z));
    } else {
      sp.withPosition(Eigen::Vector3f{
        static_cast<float>(controlled_target.position.x),
        static_cast<float>(controlled_target.position.y),
        static_cast<float>(controlled_target.position.z)});
      if (!ignore_acc_vel) {
        sp.withVelocity(Eigen::Vector3f{
          static_cast<float>(controlled_target.velocity.x),
          static_cast<float>(controlled_target.velocity.y),
          static_cast<float>(controlled_target.velocity.z)});
      }
      if (!ignore_acc_vel && !ignore_acc) {
        sp.withAcceleration(Eigen::Vector3f{
          static_cast<float>(controlled_target.acceleration.x),
          static_cast<float>(controlled_target.acceleration.y),
          static_cast<float>(controlled_target.acceleration.z)});
      }
    }
    if (_use_input_yaw) {
      sp.withYaw(controlled_target.yaw);
    }

    _trajectory_sp->update(sp);
  }

private:
  static Eigen::Vector3f enuToNed(float x_enu, float y_enu, float z_enu)
  {
    // ENU -> NED: N=y, E=x, D=-z
    return {y_enu, x_enu, -z_enu};
  }

  static float enuYawToNed(float yaw_enu)
  {
    // yaw_ned = pi/2 - yaw_enu (wrapped to [-pi, pi])
    constexpr float kHalfPi = 1.57079632679f;
    const float raw = kHalfPi - yaw_enu;
    return std::atan2(std::sin(raw), std::cos(raw));
  }

  static Eigen::Vector3f vectorParamToEigen(
    const std::vector<double> & values,
    const std::array<float, 3> & fallback)
  {
    if (values.size() < 3) {
      return {fallback[0], fallback[1], fallback[2]};
    }
    return {
      static_cast<float>(values[0]),
      static_cast<float>(values[1]),
      static_cast<float>(values[2])};
  }

  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_sp;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_position;

  rclcpp::Subscription<autonomous_flight::msg::Target>::SharedPtr _target_sub;
  std::string _target_topic;
  double _target_timeout_s{0.2};
  bool _use_input_yaw{true};
  std::unique_ptr<AgiController> _controller;

  std::mutex _target_mutex;
  autonomous_flight::msg::Target _last_target{};
  rclcpp::Time _last_target_rx{0};
  bool _target_received{false};

  Eigen::Vector3f _hold_position_ned{0.f, 0.f, 0.f};
};

class AgipixTrackingExecutor : public px4_ros2::ModeExecutorBase
{
public:
  AgipixTrackingExecutor(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
  : ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}, owned_mode), _node(node)
  {}

  void onActivate() override
  {
    RCLCPP_INFO(
      _node.get_logger(),
      "[px4_control_interface]: Executor activated, starting sequence");

    runState(State::TakingOff, px4_ros2::Result::Success);
  }

  void onDeactivate(DeactivateReason reason) override
  {
    RCLCPP_INFO(
      _node.get_logger(),
      "[px4_control_interface]: Executor deactivated (reason=%d)",
      static_cast<int>(reason));
  }

private:

  enum class State
  {
    Reset,
    TakingOff,
    TrackingMode,
    RTL,
    WaitUntilDisarmed,
  };

  void runState(State state, px4_ros2::Result previous_result)
  {
    if (previous_result != px4_ros2::Result::Success) {
      RCLCPP_ERROR(
        _node.get_logger(),
        "[px4_control_interface]: Previous state failed: %s",
        px4_ros2::resultToString(previous_result));
      return;
    }

    switch (state) {
      case State::Reset:
        break;

      case State::TakingOff:
        takeoff([this](px4_ros2::Result result) {runState(State::TrackingMode, result);});
        break;

      case State::TrackingMode:
        scheduleMode(
          ownedMode().id(), [this](px4_ros2::Result result) {runState(State::RTL, result);});
        break;

      case State::RTL:
        rtl([this](px4_ros2::Result result) {runState(State::WaitUntilDisarmed, result);});
        break;

      case State::WaitUntilDisarmed:
        waitUntilDisarmed([this](px4_ros2::Result result) {
          RCLCPP_INFO(
            _node.get_logger(),
            "[px4_control_interface]: Mission sequence complete (%s)",
            px4_ros2::resultToString(result));
        });
        break;
    }
  }

  rclcpp::Node & _node;
  // No additional parameters required; safety/arming handled by the interface library
};

}  // namespace px4_control_interface
