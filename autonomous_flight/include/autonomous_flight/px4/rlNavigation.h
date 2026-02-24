#ifndef AUTOFLIGHT_RL_NAVIGATION_H
#define AUTOFLIGHT_RL_NAVIGATION_H

#include <autonomous_flight/px4/flightBase.h>
#include <autonomous_flight/rl_navigation/solver.h>

#include <map_manager/dynamicMap.h>

#include <visualization_msgs/msg/marker_array.hpp>

namespace AutoFlight {

class rlNavigation : public flightBase {
private:
  std::shared_ptr<mapManager::dynamicMap> map_;

  rclcpp::TimerBase::SharedPtr controlTimer_;
  rclcpp::TimerBase::SharedPtr visTimer_;

  rclcpp::CallbackGroup::SharedPtr controlCbGroup_;
  rclcpp::CallbackGroup::SharedPtr visCbGroup_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rlVisPub_;

  double controlDt_{0.05};
  double velLimit_{1.0};
  double maxVerticalVel_{0.5};
  double goalTolerance_{1.0};
  double slowdownRadius_{3.0};
  bool useYawControl_{true};
  bool enableHeightControl_{false};

  int raycastBeams_{36};
  double raycastRange_{4.0};
  double obstacleInfluenceRadius_{2.0};
  double staticRepulsionGain_{0.35};
  double dynamicRepulsionGain_{0.75};

  bool useSafeAction_{true};
  double safeTimeHorizon_{1.0};
  double safeTimeStep_{0.05};
  double safeDistance_{0.25};
  double robotRadius_{0.30};

  bool missionCompleted_{false};
  rclcpp::Time lastControlTime_{0, 0, RCL_ROS_TIME};
  std::mutex stateMutex_;

  void initParam();
  void initModules();
  void registerPub();
  void registerCallback();

  void controlCB();
  void visCB();

  Eigen::Vector3d computeRawVelocityCommand();
  Eigen::Vector3d applySafeAction(
    const Eigen::Vector3d & preferred_velocity,
    const std::vector<Eigen::Vector3d> & dynamic_obs_pos,
    const std::vector<Eigen::Vector3d> & dynamic_obs_vel,
    const std::vector<Eigen::Vector3d> & dynamic_obs_size,
    const std::vector<Eigen::Vector3d> & static_hits);

  bool collectStaticRayHits(std::vector<Eigen::Vector3d> & hits) const;

public:
  explicit rlNavigation(const rclcpp::Node::SharedPtr & node);
  void run();
};

}  // namespace AutoFlight

#endif  // AUTOFLIGHT_RL_NAVIGATION_H
