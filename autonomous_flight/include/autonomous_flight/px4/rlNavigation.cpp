#include <autonomous_flight/px4/rlNavigation.h>

#include <algorithm>
#include <cmath>

namespace AutoFlight {
namespace {

navigationRunner::Plane getORCAPlane(
  const navigationRunner::Vector3 & agent_pos,
  const navigationRunner::Vector3 & agent_vel,
  double agent_radius,
  const navigationRunner::Vector3 & obs_pos,
  const navigationRunner::Vector3 & obs_vel,
  double obs_radius,
  double time_horizon,
  double time_step)
{
  const double inv_time_horizon = 1.0 / std::max(1e-3, time_horizon);

  const navigationRunner::Vector3 relative_position = obs_pos - agent_pos;
  const navigationRunner::Vector3 relative_velocity = agent_vel - obs_vel;
  const double dist_sq = navigationRunner::absSq(relative_position);
  const double combined_radius = agent_radius + obs_radius;
  const double combined_radius_sq = navigationRunner::sqr(combined_radius);

  navigationRunner::Plane plane;
  navigationRunner::Vector3 u;

  if (dist_sq > combined_radius_sq) {
    const navigationRunner::Vector3 w = relative_velocity - inv_time_horizon * relative_position;
    const double w_length_sq = navigationRunner::absSq(w);
    const double dot_product = w * relative_position;

    if (dot_product < 0.0 && navigationRunner::sqr(dot_product) > combined_radius_sq * w_length_sq) {
      const double w_length = navigationRunner::abs(w);
      const navigationRunner::Vector3 unit_w = w / std::max(1e-6, w_length);
      plane.normal = unit_w;
      u = (combined_radius * inv_time_horizon - w_length) * unit_w;
    } else {
      const double a = dist_sq;
      const double b = relative_position * relative_velocity;
      const double c = navigationRunner::absSq(relative_velocity) -
        navigationRunner::absSq(navigationRunner::cross(relative_position, relative_velocity)) /
          std::max(1e-6, (dist_sq - combined_radius_sq));
      const double discriminant = std::max(0.0, navigationRunner::sqr(b) - a * c);
      const double t = (b + std::sqrt(discriminant)) / std::max(1e-6, a);
      const navigationRunner::Vector3 w2 = relative_velocity - t * relative_position;
      const double w2_length = navigationRunner::abs(w2);
      const navigationRunner::Vector3 unit_w2 = w2 / std::max(1e-6, w2_length);
      plane.normal = unit_w2;
      u = (combined_radius * t - w2_length) * unit_w2;
    }
  } else {
    const double inv_time_step = 1.0 / std::max(1e-3, time_step);
    const navigationRunner::Vector3 w = relative_velocity - inv_time_step * relative_position;
    const double w_length = navigationRunner::abs(w);
    const navigationRunner::Vector3 unit_w = w / std::max(1e-6, w_length);
    plane.normal = unit_w;
    u = (combined_radius * inv_time_step - w_length) * unit_w;
  }

  plane.point = agent_vel + 0.85 * u;
  return plane;
}

}  // namespace

rlNavigation::rlNavigation(const rclcpp::Node::SharedPtr & node)
: flightBase(node)
{
  initParam();
  initModules();
  registerPub();
}

void rlNavigation::initParam()
{
  node_->declare_parameter<double>("rl_nav.control_dt", 0.05);
  node_->get_parameter("rl_nav.control_dt", controlDt_);

  node_->declare_parameter<double>("rl_nav.vel_limit", 1.0);
  node_->get_parameter("rl_nav.vel_limit", velLimit_);

  node_->declare_parameter<double>("rl_nav.max_vertical_velocity", 0.5);
  node_->get_parameter("rl_nav.max_vertical_velocity", maxVerticalVel_);

  node_->declare_parameter<double>("rl_nav.goal_tolerance", 1.0);
  node_->get_parameter("rl_nav.goal_tolerance", goalTolerance_);

  node_->declare_parameter<double>("rl_nav.goal_slowdown_radius", 3.0);
  node_->get_parameter("rl_nav.goal_slowdown_radius", slowdownRadius_);

  node_->declare_parameter<bool>("rl_nav.use_yaw_control", true);
  node_->get_parameter("rl_nav.use_yaw_control", useYawControl_);

  node_->declare_parameter<bool>("rl_nav.height_control", false);
  node_->get_parameter("rl_nav.height_control", enableHeightControl_);

  node_->declare_parameter<int>("rl_nav.raycast_beams", 36);
  node_->get_parameter("rl_nav.raycast_beams", raycastBeams_);

  node_->declare_parameter<double>("rl_nav.raycast_range", 4.0);
  node_->get_parameter("rl_nav.raycast_range", raycastRange_);

  node_->declare_parameter<double>("rl_nav.obstacle_influence_radius", 2.0);
  node_->get_parameter("rl_nav.obstacle_influence_radius", obstacleInfluenceRadius_);

  node_->declare_parameter<double>("rl_nav.static_repulsion_gain", 0.35);
  node_->get_parameter("rl_nav.static_repulsion_gain", staticRepulsionGain_);

  node_->declare_parameter<double>("rl_nav.dynamic_repulsion_gain", 0.75);
  node_->get_parameter("rl_nav.dynamic_repulsion_gain", dynamicRepulsionGain_);

  node_->declare_parameter<bool>("rl_nav.use_safe_action", true);
  node_->get_parameter("rl_nav.use_safe_action", useSafeAction_);

  node_->declare_parameter<double>("rl_nav.safe_action_time_horizon", 1.0);
  node_->get_parameter("rl_nav.safe_action_time_horizon", safeTimeHorizon_);

  node_->declare_parameter<double>("rl_nav.safe_action_time_step", 0.05);
  node_->get_parameter("rl_nav.safe_action_time_step", safeTimeStep_);

  node_->declare_parameter<double>("rl_nav.safe_action_safety_distance", 0.25);
  node_->get_parameter("rl_nav.safe_action_safety_distance", safeDistance_);

  node_->declare_parameter<double>("rl_nav.robot_radius", 0.30);
  node_->get_parameter("rl_nav.robot_radius", robotRadius_);

  RCLCPP_INFO(
    node_->get_logger(),
    "[AutoFlight][rl_navigation]: dt=%.3f vel_limit=%.2f goal_tol=%.2f safe_action=%s",
    controlDt_, velLimit_, goalTolerance_, useSafeAction_ ? "true" : "false");
}

void rlNavigation::initModules()
{
  map_.reset(new mapManager::dynamicMap(node_));
  map_->initMap();
}

void rlNavigation::registerPub()
{
  rlVisPub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("rl_navigation/visualization", 10);
}

void rlNavigation::registerCallback()
{
  controlCbGroup_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  visCbGroup_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const auto control_period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(std::max(0.01, controlDt_)));

  controlTimer_ = node_->create_wall_timer(
    control_period,
    std::bind(&rlNavigation::controlCB, this),
    controlCbGroup_);

  visTimer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&rlNavigation::visCB, this),
    visCbGroup_);
}

bool rlNavigation::collectStaticRayHits(std::vector<Eigen::Vector3d> & hits) const
{
  hits.clear();
  if (!odomReceived_) {
    return false;
  }

  const Eigen::Vector3d start = currPos_;
  const double yaw = AutoFlight::rpy_from_quaternion(odom_.pose.pose.orientation);
  const int beams = std::max(8, raycastBeams_);
  constexpr double kTwoPi = 6.2831853071795864769;
  const double delta = kTwoPi / static_cast<double>(beams);

  for (int i = 0; i < beams; ++i) {
    const double a = yaw + i * delta;
    const Eigen::Vector3d dir(std::cos(a), std::sin(a), 0.0);
    Eigen::Vector3d end;
    if (map_->castRay(start, dir, end, raycastRange_, true)) {
      hits.push_back(end);
    }
  }

  return !hits.empty();
}

Eigen::Vector3d rlNavigation::computeRawVelocityCommand()
{
  const Eigen::Vector3d pos = currPos_;
  Eigen::Vector3d goal(goal_.pose.position.x, goal_.pose.position.y, goal_.pose.position.z);
  if (!enableHeightControl_) {
    goal(2) = pos(2);
  }

  Eigen::Vector3d to_goal = goal - pos;
  const double dist = to_goal.norm();
  if (dist < 1e-3) {
    return Eigen::Vector3d::Zero();
  }

  const double speed_scale = std::clamp(dist / std::max(0.1, slowdownRadius_), 0.15, 1.0);
  Eigen::Vector3d cmd = to_goal.normalized() * velLimit_ * speed_scale;

  std::vector<Eigen::Vector3d> static_hits;
  collectStaticRayHits(static_hits);
  for (const auto & hit : static_hits) {
    Eigen::Vector3d away = pos - hit;
    away(2) = 0.0;
    const double d = away.norm();
    if (d < 1e-3 || d > obstacleInfluenceRadius_) {
      continue;
    }
    const double gain = staticRepulsionGain_ * (1.0 / d - 1.0 / obstacleInfluenceRadius_);
    cmd += away.normalized() * gain;
  }

  std::vector<Eigen::Vector3d> obs_pos;
  std::vector<Eigen::Vector3d> obs_vel;
  std::vector<Eigen::Vector3d> obs_size;
  map_->getDynamicObstacles(obs_pos, obs_vel, obs_size);

  for (size_t i = 0; i < obs_pos.size(); ++i) {
    Eigen::Vector3d away = pos - obs_pos[i];
    away(2) = 0.0;
    const double body_radius = 0.5 * std::max(obs_size[i](0), obs_size[i](1)) + robotRadius_;
    const double influence = obstacleInfluenceRadius_ + body_radius;
    const double d = away.norm();
    if (d < 1e-3 || d > influence) {
      continue;
    }
    const double gain = dynamicRepulsionGain_ * (1.0 / std::max(d, body_radius) - 1.0 / influence);
    cmd += away.normalized() * gain;
  }

  const double xy_speed = std::hypot(cmd.x(), cmd.y());
  if (xy_speed > velLimit_) {
    const double s = velLimit_ / xy_speed;
    cmd.x() *= s;
    cmd.y() *= s;
  }

  if (!enableHeightControl_) {
    cmd.z() = 0.0;
  }
  cmd.z() = std::clamp(cmd.z(), -maxVerticalVel_, maxVerticalVel_);

  return cmd;
}

Eigen::Vector3d rlNavigation::applySafeAction(
  const Eigen::Vector3d & preferred_velocity,
  const std::vector<Eigen::Vector3d> & dynamic_obs_pos,
  const std::vector<Eigen::Vector3d> & dynamic_obs_vel,
  const std::vector<Eigen::Vector3d> & dynamic_obs_size,
  const std::vector<Eigen::Vector3d> & static_hits)
{
  using navigationRunner::Plane;
  using navigationRunner::Vector3;

  const Eigen::Vector3d pos = currPos_;
  const Eigen::Vector3d vel = currVel_;

  Vector3 agent_pos(pos.x(), pos.y(), pos.z());
  Vector3 agent_vel(vel.x(), vel.y(), vel.z());
  Vector3 pref_vel(preferred_velocity.x(), preferred_velocity.y(), preferred_velocity.z());

  std::vector<Plane> planes;
  planes.reserve(dynamic_obs_pos.size() + static_hits.size() + 2);

  for (size_t i = 0; i < dynamic_obs_pos.size(); ++i) {
    const double obs_r = 0.5 * std::max(dynamic_obs_size[i](0), dynamic_obs_size[i](1));
    planes.push_back(getORCAPlane(
      agent_pos,
      agent_vel,
      robotRadius_ + safeDistance_,
      Vector3(dynamic_obs_pos[i].x(), dynamic_obs_pos[i].y(), dynamic_obs_pos[i].z()),
      Vector3(dynamic_obs_vel[i].x(), dynamic_obs_vel[i].y(), dynamic_obs_vel[i].z()),
      std::max(0.05, obs_r),
      safeTimeHorizon_,
      safeTimeStep_));
  }

  for (const auto & p : static_hits) {
    planes.push_back(getORCAPlane(
      agent_pos,
      agent_vel,
      robotRadius_ + safeDistance_,
      Vector3(p.x(), p.y(), p.z()),
      Vector3(0.0, 0.0, 0.0),
      std::max(0.08, map_->getRes() * 1.5),
      safeTimeHorizon_,
      safeTimeStep_));
  }

  // Height guard planes.
  {
    Plane floor_plane;
    floor_plane.normal = Vector3(0.0, 0.0, 1.0);
    floor_plane.point = Vector3(0.0, 0.0, 0.0);
    planes.push_back(floor_plane);

    Plane ceil_plane;
    ceil_plane.normal = Vector3(0.0, 0.0, -1.0);
    ceil_plane.point = Vector3(0.0, 0.0, 0.0);
    planes.push_back(ceil_plane);
  }

  Vector3 safe_vel = pref_vel;
  const size_t failed = navigationRunner::linearProgram3(planes, std::sqrt(2.0) * velLimit_, pref_vel, false, safe_vel);
  if (failed < planes.size()) {
    navigationRunner::linearProgram4(planes, failed, std::sqrt(2.0) * velLimit_, safe_vel);
  }

  Eigen::Vector3d result(safe_vel[0], safe_vel[1], safe_vel[2]);
  if (!enableHeightControl_) {
    result.z() = 0.0;
  }
  result.z() = std::clamp(result.z(), -maxVerticalVel_, maxVerticalVel_);
  return result;
}

void rlNavigation::controlCB()
{
  std::scoped_lock<std::mutex> lock(stateMutex_);

  if (!odomReceived_ || !firstGoal_) {
    return;
  }

  const Eigen::Vector3d goal(goal_.pose.position.x, goal_.pose.position.y, goal_.pose.position.z);
  Eigen::Vector3d local_goal = goal;
  if (!enableHeightControl_) {
    local_goal(2) = currPos_(2);
  }

  const double dist = (local_goal - currPos_).norm();
  if (dist <= goalTolerance_) {
    if (!missionCompleted_) {
      missionCompleted_ = true;
      RCLCPP_INFO(node_->get_logger(), "[AutoFlight][rl_navigation]: Goal reached. Holding position.");
    }

    autonomous_flight::msg::Target hold;
    hold.type_mask = autonomous_flight::msg::Target::IGNORE_ACC_VEL;
    hold.position.x = currPos_(0);
    hold.position.y = currPos_(1);
    hold.position.z = currPos_(2);
    hold.velocity.x = 0.0;
    hold.velocity.y = 0.0;
    hold.velocity.z = 0.0;
    hold.acceleration.x = 0.0;
    hold.acceleration.y = 0.0;
    hold.acceleration.z = 0.0;
    hold.yaw = AutoFlight::rpy_from_quaternion(odom_.pose.pose.orientation);
    updateTargetWithState(hold);
    return;
  }

  missionCompleted_ = false;

  Eigen::Vector3d cmd_vel = computeRawVelocityCommand();

  std::vector<Eigen::Vector3d> static_hits;
  collectStaticRayHits(static_hits);
  std::vector<Eigen::Vector3d> obs_pos;
  std::vector<Eigen::Vector3d> obs_vel;
  std::vector<Eigen::Vector3d> obs_size;
  map_->getDynamicObstacles(obs_pos, obs_vel, obs_size);

  if (useSafeAction_) {
    cmd_vel = applySafeAction(cmd_vel, obs_pos, obs_vel, obs_size, static_hits);
  }

  const double yaw = std::atan2(cmd_vel.y(), cmd_vel.x());

  autonomous_flight::msg::Target target;
  target.type_mask = autonomous_flight::msg::Target::IGNORE_POS_ACC;
  target.position.x = currPos_(0);
  target.position.y = currPos_(1);
  target.position.z = currPos_(2);
  target.velocity.x = cmd_vel.x();
  target.velocity.y = cmd_vel.y();
  target.velocity.z = cmd_vel.z();
  target.acceleration.x = 0.0;
  target.acceleration.y = 0.0;
  target.acceleration.z = 0.0;
  target.yaw = useYawControl_ ? yaw : AutoFlight::rpy_from_quaternion(odom_.pose.pose.orientation);

  updateTargetWithState(target);
}

void rlNavigation::visCB()
{
  if (!rlVisPub_ || !odomReceived_) {
    return;
  }

  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker goal_m;
  goal_m.header.frame_id = mapFrameId_;
  goal_m.header.stamp = node_->now();
  goal_m.ns = "rl_navigation";
  goal_m.id = 0;
  goal_m.type = visualization_msgs::msg::Marker::SPHERE;
  goal_m.action = visualization_msgs::msg::Marker::ADD;
  goal_m.pose.position = goal_.pose.position;
  goal_m.pose.orientation.w = 1.0;
  goal_m.scale.x = 0.35;
  goal_m.scale.y = 0.35;
  goal_m.scale.z = 0.35;
  goal_m.color.a = 1.0;
  goal_m.color.r = 1.0;
  goal_m.color.g = 0.3;
  goal_m.color.b = 0.2;
  msg.markers.push_back(goal_m);

  rlVisPub_->publish(msg);
}

void rlNavigation::run()
{
  registerCallback();
  RCLCPP_INFO(node_->get_logger(), "[AutoFlight][rl_navigation]: Mission runner started.");
}

}  // namespace AutoFlight
