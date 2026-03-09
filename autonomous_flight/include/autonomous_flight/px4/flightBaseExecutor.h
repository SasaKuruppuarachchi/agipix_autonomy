#pragma once

#include <Eigen/Core>

#include <cmath>
#include <mutex>
#include <string>
#include <algorithm>
#include <cctype>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/vehicle_state/vehicle_status.hpp>

#include <autonomous_flight/msg/target.hpp>
#include <autonomous_flight/px4/target_qos.h>

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

    _target_topic = getOrDeclareParam<std::string>(node, "target_topic", "/autonomous_flight/target_state");
    _target_timeout_s = getOrDeclareParam<double>(node, "target_timeout_s", 0.2);
    _target_qos_depth = std::max(1, getOrDeclareParam<int>(node, "target_qos_depth", 1));
    _target_qos_reliability =
      getOrDeclareParam<std::string>(node, "target_qos_reliability", "best_effort");
    _target_qos_durability = getOrDeclareParam<std::string>(node, "target_qos_durability", "volatile");
    _use_input_yaw = getOrDeclareParam<bool>(node, "use_input_yaw", true);
    _builtin_profile = getOrDeclareParam<std::string>(node, "builtin_profile", "external_target");
    _circle_radius_m = static_cast<float>(getOrDeclareParam<double>(node, "circle_radius", 2.0));
    _circle_velocity_m_s = static_cast<float>(getOrDeclareParam<double>(node, "velocity", 0.5));
    _circle_use_tangent_yaw = getOrDeclareParam<bool>(node, "yaw_control", false);
    _publish_target_rx_marker = getOrDeclareParam<bool>(node, "publish_target_rx_marker", false);
    _target_rx_marker_topic =
      getOrDeclareParam<std::string>(node, "target_rx_marker_topic", "/autonomous_flight/target_state_rx_marker");
    _target_rx_marker_frame_id = getOrDeclareParam<std::string>(node, "target_rx_marker_frame_id", "drone0/map");
    _target_rx_marker_scale_m =
      static_cast<float>(getOrDeclareParam<double>(node, "target_rx_marker_scale", 0.6));
    _debug_mode = getOrDeclareParam<bool>(node, "debug_mode", false);

    std::transform(
      _builtin_profile.begin(), _builtin_profile.end(), _builtin_profile.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});

    _target_cb_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions target_sub_options;
    target_sub_options.callback_group = _target_cb_group;

    target_sub_options.event_callbacks.incompatible_qos_callback =
      [this](rclcpp::QOSRequestedIncompatibleQoSInfo & info) {
        RCLCPP_WARN(
          this->node().get_logger(),
          "[AutoFlight]: /autonomous_flight/target_state incompatible QoS (last_policy_kind=%d, total_count=%d).",
          info.last_policy_kind,
          info.total_count);
      };

    rclcpp::QoS target_qos = buildTargetQos();

    _target_sub = node.create_subscription<autonomous_flight::msg::Target>(
      _target_topic,
      target_qos,
      std::bind(&flightBaseTrackingMode::targetCallback, this, std::placeholders::_1),
      target_sub_options);

    if (_publish_target_rx_marker) {
      _target_rx_marker_pub = node.create_publisher<visualization_msgs::msg::Marker>(
        _target_rx_marker_topic,
        rclcpp::QoS(10));
    }

    if (_debug_mode) {
      RCLCPP_INFO(
        node.get_logger(),
        "[AutoFlight][debug]: tracking mode config target_topic=%s timeout=%.2f profile=%s qos=(depth=%d, reliability=%s, durability=%s)",
        _target_topic.c_str(),
        _target_timeout_s,
        _builtin_profile.c_str(),
        _target_qos_depth,
        _target_qos_reliability.c_str(),
        _target_qos_durability.c_str());
    }
  }

  void onActivate() override
  {
    refreshHoldFromLocalPosition();
    _hover_hold_latched = false;
    _activation_time = node().get_clock()->now();

    if (_debug_mode) {
      RCLCPP_INFO(
        node().get_logger(),
        "[AutoFlight][debug]: mode activate hold_ned=(%.2f, %.2f, %.2f) hold_yaw=%.2f",
        _hold_position_ned.x(),
        _hold_position_ned.y(),
        _hold_position_ned.z(),
        _hold_yaw_ned);
    }
  }

  void onDeactivate() override {}

  void updateSetpoint(float /*dt_s*/) override
  {
    px4_ros2::TrajectorySetpoint sp;

    if (applyBuiltinProfileSetpoint(sp)) {
      _trajectory_sp->update(sp);
      return;
    }

    autonomous_flight::msg::Target target;
    if (getFreshTarget(target)) {
      applyExternalTargetSetpoint(sp, target);
      _hover_hold_latched = false;

      if (_debug_mode) {
        RCLCPP_INFO_THROTTLE(
          node().get_logger(),
          *node().get_clock(),
          1000,
          "[AutoFlight][debug]: external target used hold_ned=(%.2f, %.2f, %.2f)",
          _hold_position_ned.x(),
          _hold_position_ned.y(),
          _hold_position_ned.z());
      }
    } else {
      if (!_hover_hold_latched) {
        refreshHoldFromLocalPosition();
        _hover_hold_latched = true;

        if (_debug_mode) {
          RCLCPP_INFO(
            node().get_logger(),
            "[AutoFlight][debug]: hover hold latched from local position ned=(%.2f, %.2f, %.2f)",
            _hold_position_ned.x(),
            _hold_position_ned.y(),
            _hold_position_ned.z());
        }
      }
      RCLCPP_WARN_THROTTLE(
        node().get_logger(),
        *node().get_clock(),
        2000,
        "[AutoFlight]: No fresh /autonomous_flight/target_state. Holding hover setpoint.");
      applyHoverSetpoint(sp);
    }

    _trajectory_sp->update(sp);
  }

private:
  template<typename T>
  static T getOrDeclareParam(rclcpp::Node & node, const std::string & name, const T & default_value)
  {
    T value{};
    if (!node.get_parameter(name, value)) {
      value = node.declare_parameter<T>(name, default_value);
    }
    return value;
  }

  void targetCallback(const autonomous_flight::msg::Target::SharedPtr msg)
  {
    const rclcpp::Time now = this->node().get_clock()->now();
    const bool has_stamp = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
    rclcpp::Time msg_stamp = now;
    if (has_stamp) {
      msg_stamp = rclcpp::Time(msg->header.stamp);
    }

    std::scoped_lock<std::mutex> lock(_target_mutex);

    // Drop out-of-order samples if source publishes stamped targets.
    if (has_stamp && _last_target_stamp_valid && msg_stamp < _last_target_stamp) {
      RCLCPP_WARN_THROTTLE(
        this->node().get_logger(),
        *this->node().get_clock(),
        2000,
        "[AutoFlight]: Dropping out-of-order target sample (stamp went backwards).");
      return;
    }

    // Drop stale samples that can appear after transport backlog/reconnect.
    if (has_stamp) {
      const double age = (now - msg_stamp).seconds();
      if (age > std::max(0.05, _target_timeout_s * 2.0)) {
        RCLCPP_WARN_THROTTLE(
          this->node().get_logger(),
          *this->node().get_clock(),
          2000,
          "[AutoFlight]: Dropping stale target sample (age=%.3fs).",
          age);
        return;
      }
      _last_target_stamp = msg_stamp;
      _last_target_stamp_valid = true;
    }

    _last_target = *msg;
    _last_target_rx = now;
    _target_received = true;

    if (_publish_target_rx_marker && _target_rx_marker_pub) {
      publishTargetRxMarker(*msg);
    }

    if (_debug_mode) {
      RCLCPP_INFO_THROTTLE(
        this->node().get_logger(),
        *this->node().get_clock(),
        1000,
        "[AutoFlight][debug]: target rx pos_enu=(%.2f, %.2f, %.2f) yaw=%.2f",
        msg->position.x,
        msg->position.y,
        msg->position.z,
        msg->yaw);
    }
  }

  rclcpp::QoS buildTargetQos() const
  {
    return AutoFlight::buildTargetQos(
      _target_qos_depth,
      _target_qos_reliability,
      _target_qos_durability);
  }

  void publishTargetRxMarker(const autonomous_flight::msg::Target & target)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = _target_rx_marker_frame_id;
    marker.header.stamp = node().get_clock()->now();
    marker.ns = "target_state_rx";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = target.position.x;
    marker.pose.position.y = target.position.y;
    marker.pose.position.z = target.position.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.01f, _target_rx_marker_scale_m);
    marker.scale.y = std::max(0.01f, _target_rx_marker_scale_m);
    marker.scale.z = std::max(0.01f, _target_rx_marker_scale_m);
    marker.color.a = 1.0f;
    marker.color.r = 0.2f;
    marker.color.g = 0.6f;
    marker.color.b = 1.0f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.15);
    _target_rx_marker_pub->publish(marker);
  }

  bool applyBuiltinProfileSetpoint(px4_ros2::TrajectorySetpoint & sp)
  {
    if (_builtin_profile == "hover") {
      applyHoverSetpoint(sp);
      return true;
    }

    if (_builtin_profile == "circle") {
      applyCircleSetpoint(sp);
      return true;
    }

    return false;
  }

  void applyHoverSetpoint(px4_ros2::TrajectorySetpoint & sp)
  {
    sp.withPosition(_hold_position_ned);
    sp.withYaw(_hold_yaw_ned);
  }

  void applyCircleSetpoint(px4_ros2::TrajectorySetpoint & sp)
  {
    const float speed = std::max(0.05f, _circle_velocity_m_s);
    const float radius = std::max(0.2f, _circle_radius_m);
    const float omega = speed / radius;
    const float t = static_cast<float>((node().get_clock()->now() - _activation_time).seconds());

    const float c = std::cos(omega * t);
    const float s = std::sin(omega * t);

    const Eigen::Vector3f pos_ned{
      _hold_position_ned.x() + radius * c,
      _hold_position_ned.y() + radius * s,
      _hold_position_ned.z()};
    const Eigen::Vector3f vel_ned{
      -radius * omega * s,
      radius * omega * c,
      0.f};
    const Eigen::Vector3f acc_ned{
      -radius * omega * omega * c,
      -radius * omega * omega * s,
      0.f};

    sp.withPosition(pos_ned);
    sp.withVelocity(vel_ned);
    sp.withAcceleration(acc_ned);
    if (_circle_use_tangent_yaw) {
      sp.withYaw(std::atan2(vel_ned.y(), vel_ned.x()));
    } else {
      sp.withYaw(_hold_yaw_ned);
    }
  }

  bool getFreshTarget(autonomous_flight::msg::Target & target)
  {
    std::scoped_lock<std::mutex> lock(_target_mutex);
    if (!_target_received) {
      return false;
    }

    const double age = (node().get_clock()->now() - _last_target_rx).seconds();
    if (age > _target_timeout_s) {
      return false;
    }

    target = _last_target;
    return true;
  }

  void applyExternalTargetSetpoint(
    px4_ros2::TrajectorySetpoint & sp,
    const autonomous_flight::msg::Target & target)
  {
    const Eigen::Vector3f pos_ned = enuToNed(target.position.x, target.position.y, target.position.z);
    const Eigen::Vector3f vel_ned = enuToNed(target.velocity.x, target.velocity.y, target.velocity.z);
    const Eigen::Vector3f acc_ned = enuToNed(target.acceleration.x, target.acceleration.y, target.acceleration.z);

    const bool ignore_acc_vel = target.type_mask == autonomous_flight::msg::Target::IGNORE_ACC_VEL;
    const bool ignore_acc = target.type_mask == autonomous_flight::msg::Target::IGNORE_ACC;
    const bool velocity_priority = target.type_mask == autonomous_flight::msg::Target::IGNORE_POS_ACC;

    if (velocity_priority) {
      sp.withHorizontalVelocity(Eigen::Vector2f(vel_ned.x(), vel_ned.y()));
      sp.withPositionZ(pos_ned.z());
      if (enableHeightControl(target)) {
        sp.withVelocityZ(vel_ned.z());
      } else {
        sp.withVelocityZ(0.0f);
      }
    } else {
      sp.withPosition(pos_ned);
      if (!ignore_acc_vel) {
        sp.withVelocity(vel_ned);
      }
      if (!ignore_acc_vel && !ignore_acc) {
        sp.withAcceleration(acc_ned);
      }
    }
    if (_use_input_yaw) {
      sp.withYaw(enuYawToNed(target.yaw));
    }

    if (!velocity_priority) {
      _hold_position_ned = pos_ned;
    } else {
      _hold_position_ned.z() = pos_ned.z();
    }
    _hold_yaw_ned = _use_input_yaw ? enuYawToNed(target.yaw) : _hold_yaw_ned;
  }

  bool enableHeightControl(const autonomous_flight::msg::Target & target) const
  {
    return std::abs(target.velocity.z) > 1e-3;
  }

  void refreshHoldFromLocalPosition()
  {
    if (_local_position->positionXYValid() && _local_position->positionZValid()) {
      _hold_position_ned = _local_position->positionNed();
      _hold_yaw_ned = _local_position->heading();
    } else if (_debug_mode) {
      RCLCPP_WARN_THROTTLE(
        node().get_logger(),
        *node().get_clock(),
        2000,
        "[AutoFlight][debug]: local position not valid yet; keep previous hold setpoint.");
    }
  }

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
  rclcpp::CallbackGroup::SharedPtr _target_cb_group;

  rclcpp::Subscription<autonomous_flight::msg::Target>::SharedPtr _target_sub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _target_rx_marker_pub;
  std::string _target_topic;
  double _target_timeout_s{0.2};
  int _target_qos_depth{1};
  std::string _target_qos_reliability{"best_effort"};
  std::string _target_qos_durability{"volatile"};
  bool _use_input_yaw{true};
  std::string _builtin_profile{"external_target"};
  float _circle_radius_m{2.0f};
  float _circle_velocity_m_s{0.5f};
  bool _circle_use_tangent_yaw{false};
  bool _publish_target_rx_marker{false};
  std::string _target_rx_marker_topic{"/autonomous_flight/target_state_rx_marker"};
  std::string _target_rx_marker_frame_id{"map"};
  float _target_rx_marker_scale_m{0.2f};

  std::mutex _target_mutex;
  autonomous_flight::msg::Target _last_target{};
  rclcpp::Time _last_target_rx{0};
  rclcpp::Time _last_target_stamp{0};
  bool _last_target_stamp_valid{false};
  bool _target_received{false};
  bool _hover_hold_latched{false};
  bool _debug_mode{false};
  rclcpp::Time _activation_time{0};

  Eigen::Vector3f _hold_position_ned{0.f, 0.f, 0.f};
  float _hold_yaw_ned{0.f};
};

class flightBaseExecutor : public px4_ros2::ModeExecutorBase
{
public:
  flightBaseExecutor(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
  : ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
    _node(node)
  {
    _local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(owned_mode);
    _vehicle_status = std::make_shared<px4_ros2::VehicleStatus>(owned_mode);
    _skip_takeoff_if_flying = getOrDeclareParam<bool>(node, "skip_takeoff_if_flying", true);
    _flying_height_threshold_m =
      static_cast<float>(getOrDeclareParam<double>(node, "flying_height_threshold", 0.35));
    _debug_mode = getOrDeclareParam<bool>(node, "debug_mode", false);

    if (_debug_mode) {
      RCLCPP_INFO(
        _node.get_logger(),
        "[AutoFlight][debug]: executor config skip_takeoff_if_flying=%d threshold=%.2f",
        static_cast<int>(_skip_takeoff_if_flying),
        _flying_height_threshold_m);
    }
  }

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

    if (_debug_mode) {
      RCLCPP_INFO(
        _node.get_logger(),
        "[AutoFlight][debug]: entering state=%d (prev=%s)",
        static_cast<int>(state),
        px4_ros2::resultToString(previous_result));
    }

    switch (state) {
      case State::Reset:
        break;

      case State::TakingOff:
      {
        if (_skip_takeoff_if_flying && shouldSkipTakeoff()) {
          const float current_z_ned = _local_position->positionNed().z();
          const float current_altitude_m = std::abs(current_z_ned);
          const auto status = _vehicle_status->last();
          RCLCPP_WARN(
            _node.get_logger(),
            "[AutoFlight]: Skip executor takeoff because vehicle is already flying (armed=%d, nav_state=%u, takeoff_time=%llu, |z|=%.2f m, threshold=%.2f m).",
            static_cast<int>(_vehicle_status->armed()),
            static_cast<unsigned>(_vehicle_status->navState()),
            static_cast<unsigned long long>(status.takeoff_time),
            current_altitude_m,
            _flying_height_threshold_m);
          runState(State::TrackingMode, px4_ros2::Result::Success);
          break;
        }

        takeoff([this](px4_ros2::Result result) {runState(State::TrackingMode, result);});
        break;
      }

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
  template<typename T>
  static T getOrDeclareParam(rclcpp::Node & node, const std::string & name, const T & default_value)
  {
    T value{};
    if (!node.get_parameter(name, value)) {
      value = node.declare_parameter<T>(name, default_value);
    }
    return value;
  }

  bool isAirborneNavState(uint8_t nav_state) const
  {
    switch (nav_state) {
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND:
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ORBIT:
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSITION_SLOW:
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND:
        return true;
      default:
        return false;
    }
  }

  bool shouldSkipTakeoff() const
  {
    const bool is_armed = _vehicle_status->armed();
    if (!is_armed) {
      return false;
    }

    const auto status = _vehicle_status->last();
    const uint8_t nav_state = _vehicle_status->navState();

    // Main signal from commander once takeoff occurred.
    if (status.takeoff_time > 0) {
      return true;
    }

    // Secondary signal for active airborne flight states.
    if (isAirborneNavState(nav_state)) {
      return true;
    }

    // Final fallback if status lags but altitude clearly indicates airborne.
    const float current_altitude_m = std::abs(_local_position->positionNed().z());
    return current_altitude_m >= _flying_height_threshold_m;
  }

  rclcpp::Node & _node;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_position;
  std::shared_ptr<px4_ros2::VehicleStatus> _vehicle_status;
  bool _skip_takeoff_if_flying{true};
  float _flying_height_threshold_m{0.35f};
  bool _debug_mode{false};
};

}  // namespace AutoFlight
