#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <mutex>
#include <string>

#include <autonomous_flight/msg/target.hpp>

namespace px4_control_interface
{

struct AgiControllerState
{
  Eigen::Vector3f position_ned{0.f, 0.f, 0.f};
  Eigen::Vector3f velocity_ned{0.f, 0.f, 0.f};
  Eigen::Vector3f acceleration_ned{0.f, 0.f, 0.f};
  float yaw_ned{0.f};
};

struct AgiControllerReference
{
  Eigen::Vector3f position_ned{0.f, 0.f, 0.f};
  Eigen::Vector3f velocity_ned{0.f, 0.f, 0.f};
  Eigen::Vector3f acceleration_ned{0.f, 0.f, 0.f};
  float yaw_ned{0.f};
  uint8_t type_mask{autonomous_flight::msg::Target::IGNORE_ACC_VEL};
};

class AgiController
{
public:
  virtual ~AgiController() = default;

  virtual std::string name() const = 0;

  virtual void reset(const AgiControllerState & state) = 0;

  virtual void setReference(const AgiControllerReference & reference) = 0;

  virtual autonomous_flight::msg::Target update(const AgiControllerState & state, float dt_s) = 0;
};

class PassThroughAgiController : public AgiController
{
public:
  std::string name() const override
  {
    return "pass_through";
  }

  void reset(const AgiControllerState & /*state*/) override {}

  void setReference(const AgiControllerReference & reference) override
  {
    std::scoped_lock<std::mutex> lock(_reference_mutex);
    _reference = reference;
  }

  autonomous_flight::msg::Target update(const AgiControllerState & /*state*/, float /*dt_s*/) override
  {
    std::scoped_lock<std::mutex> lock(_reference_mutex);
    return toTargetMsg(_reference);
  }

protected:
  static autonomous_flight::msg::Target toTargetMsg(const AgiControllerReference & ref)
  {
    autonomous_flight::msg::Target out;
    out.type_mask = ref.type_mask;
    out.position.x = ref.position_ned.x();
    out.position.y = ref.position_ned.y();
    out.position.z = ref.position_ned.z();
    out.velocity.x = ref.velocity_ned.x();
    out.velocity.y = ref.velocity_ned.y();
    out.velocity.z = ref.velocity_ned.z();
    out.acceleration.x = ref.acceleration_ned.x();
    out.acceleration.y = ref.acceleration_ned.y();
    out.acceleration.z = ref.acceleration_ned.z();
    out.yaw = ref.yaw_ned;
    return out;
  }

  std::mutex _reference_mutex;
  AgiControllerReference _reference{};
};

class CascadedPidAgiController : public PassThroughAgiController
{
public:
  CascadedPidAgiController(
    const Eigen::Vector3f & kp_pos,
    const Eigen::Vector3f & kd_vel,
    const Eigen::Vector3f & ki_pos,
    float integral_limit)
  : _kp_pos(kp_pos), _kd_vel(kd_vel), _ki_pos(ki_pos), _integral_limit(std::max(0.f, integral_limit))
  {}

  std::string name() const override
  {
    return "cascaded_pid";
  }

  void reset(const AgiControllerState & /*state*/) override
  {
    std::scoped_lock<std::mutex> lock(_reference_mutex);
    _pos_error_integral.setZero();
  }

  autonomous_flight::msg::Target update(const AgiControllerState & state, float dt_s) override
  {
    AgiControllerReference ref;
    {
      std::scoped_lock<std::mutex> lock(_reference_mutex);
      ref = _reference;
    }

    const float safe_dt = dt_s > 1e-4f ? dt_s : 0.01f;

    Eigen::Vector3f velocity_ref = ref.velocity_ned;
    Eigen::Vector3f acceleration_ff = ref.acceleration_ned;

    if (ref.type_mask == autonomous_flight::msg::Target::IGNORE_ACC_VEL) {
      velocity_ref.setZero();
      acceleration_ff.setZero();
    } else if (ref.type_mask == autonomous_flight::msg::Target::IGNORE_ACC) {
      acceleration_ff.setZero();
    }

    const Eigen::Vector3f pos_error = ref.position_ned - state.position_ned;
    const Eigen::Vector3f vel_error = velocity_ref - state.velocity_ned;

    _pos_error_integral += pos_error * safe_dt;
    const Eigen::Vector3f limits = Eigen::Vector3f::Constant(_integral_limit);
    _pos_error_integral = _pos_error_integral.cwiseMax(-limits).cwiseMin(limits);

    const Eigen::Vector3f acceleration_cmd =
      acceleration_ff +
      _kp_pos.cwiseProduct(pos_error) +
      _kd_vel.cwiseProduct(vel_error) +
      _ki_pos.cwiseProduct(_pos_error_integral);

    autonomous_flight::msg::Target out;
    out.type_mask = 0U;  // emit full controlled state for the low-level setpoint writer
    out.position.x = ref.position_ned.x();
    out.position.y = ref.position_ned.y();
    out.position.z = ref.position_ned.z();
    out.velocity.x = velocity_ref.x();
    out.velocity.y = velocity_ref.y();
    out.velocity.z = velocity_ref.z();
    out.acceleration.x = acceleration_cmd.x();
    out.acceleration.y = acceleration_cmd.y();
    out.acceleration.z = acceleration_cmd.z();
    out.yaw = ref.yaw_ned;
    return out;
  }

private:
  Eigen::Vector3f _kp_pos;
  Eigen::Vector3f _kd_vel;
  Eigen::Vector3f _ki_pos;
  float _integral_limit{0.5f};
  Eigen::Vector3f _pos_error_integral{0.f, 0.f, 0.f};
};

}  // namespace px4_control_interface
