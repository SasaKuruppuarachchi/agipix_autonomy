#pragma once

#include <Eigen/Core>

#include <cmath>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <tracking_controller/msg/target.hpp>

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

    _target_sub = node.create_subscription<tracking_controller::msg::Target>(
      _target_topic,
      rclcpp::QoS(10),
      [this](const tracking_controller::msg::Target::SharedPtr msg) {
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
    tracking_controller::msg::Target target;
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

      sp.withPosition(pos_ned).withVelocity(vel_ned).withAcceleration(acc_ned);
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

  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_sp;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_position;

  rclcpp::Subscription<tracking_controller::msg::Target>::SharedPtr _target_sub;
  std::string _target_topic;
  double _target_timeout_s{0.2};
  bool _use_input_yaw{true};

  std::mutex _target_mutex;
  tracking_controller::msg::Target _last_target{};
  rclcpp::Time _last_target_rx{0};
  bool _target_received{false};

  Eigen::Vector3f _hold_position_ned{0.f, 0.f, 0.f};
};

}  // namespace px4_control_interface
