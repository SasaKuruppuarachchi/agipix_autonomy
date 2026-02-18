#pragma once

#include <Eigen/Core>

#include <cmath>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <autonomous_flight/msg/target.hpp>

namespace AutoFlight
{

class flightBaseTrackingMode : public px4_ros2::ModeBase
{
public:
  explicit flightBaseTrackingMode(rclcpp::Node & node)
  : ModeBase(node, px4_ros2::ModeBase::Settings{"AgiPix FB Tracking", false})
  {
    _trajectory_sp = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    _target_topic = node.declare_parameter<std::string>("target_topic", "/autonomous_flight/target_state");
    _target_timeout_s = node.declare_parameter<double>("target_timeout_s", 0.2);
    _use_input_yaw = node.declare_parameter<bool>("use_input_yaw", true);

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
  }

  void onDeactivate() override {}

  void updateSetpoint(float /*dt_s*/) override
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

    px4_ros2::TrajectorySetpoint sp;

    if (has_fresh_target) {
      const Eigen::Vector3f pos_ned = enuToNed(target.position.x, target.position.y, target.position.z);
      const Eigen::Vector3f vel_ned = enuToNed(target.velocity.x, target.velocity.y, target.velocity.z);
      const Eigen::Vector3f acc_ned = enuToNed(target.acceleration.x, target.acceleration.y, target.acceleration.z);

      const bool ignore_acc_vel = target.type_mask == autonomous_flight::msg::Target::IGNORE_ACC_VEL;
      const bool ignore_acc = target.type_mask == autonomous_flight::msg::Target::IGNORE_ACC;

      sp.withPosition(pos_ned);
      if (!ignore_acc_vel) {
        sp.withVelocity(vel_ned);
      }
      if (!ignore_acc_vel && !ignore_acc) {
        sp.withAcceleration(acc_ned);
      }
      if (_use_input_yaw) {
        sp.withYaw(enuYawToNed(target.yaw));
      }

      _hold_position_ned = pos_ned;
    } else {
      sp.withPosition(_hold_position_ned);
    }

    _trajectory_sp->update(sp);
  }

private:
  static Eigen::Vector3f enuToNed(float x_enu, float y_enu, float z_enu)
  {
    return {y_enu, x_enu, -z_enu};
  }

  static float enuYawToNed(float yaw_enu)
  {
    constexpr float kHalfPi = 1.57079632679f;
    const float raw = kHalfPi - yaw_enu;
    return std::atan2(std::sin(raw), std::cos(raw));
  }

  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_sp;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_position;

  rclcpp::Subscription<autonomous_flight::msg::Target>::SharedPtr _target_sub;
  std::string _target_topic;
  double _target_timeout_s{0.2};
  bool _use_input_yaw{true};

  std::mutex _target_mutex;
  autonomous_flight::msg::Target _last_target{};
  rclcpp::Time _last_target_rx{0};
  bool _target_received{false};

  Eigen::Vector3f _hold_position_ned{0.f, 0.f, 0.f};
};

class flightBaseExecutor : public px4_ros2::ModeExecutorBase
{
public:
  flightBaseExecutor(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
  : ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}, owned_mode), _node(node)
  {}

  enum class State
  {
    Reset,
    TakingOff,
    TrackingMode,
    RTL,
    WaitUntilDisarmed,
  };

  void onActivate() override
  {
    runState(State::TakingOff, px4_ros2::Result::Success);
  }

  void onDeactivate(DeactivateReason /*reason*/) override {}

  void runState(State state, px4_ros2::Result previous_result)
  {
    if (previous_result != px4_ros2::Result::Success) {
      RCLCPP_ERROR(
        _node.get_logger(),
        "[AutoFlight]: State failed: %s",
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
            "[AutoFlight]: Mode-executor sequence complete (%s)",
            px4_ros2::resultToString(result));
        });
        break;
    }
  }

private:
  rclcpp::Node & _node;
};

}  // namespace AutoFlight
