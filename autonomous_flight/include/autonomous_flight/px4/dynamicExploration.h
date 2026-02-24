/*
	FILE: dynamicExploration.h
	-----------------------------
	header of dynamic exploration
*/
#ifndef DYNAMIC_EXPLORATION
#define DYNAMIC_EXPLORATION
#include <autonomous_flight/px4/flightBase.h>
#include <global_planner/dep.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <map_manager/dynamicMap.h>
#include <std_srvs/srv/trigger.hpp>


namespace AutoFlight{
	class dynamicExploration : flightBase{
	private:
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<globalPlanner::DEP> expPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		rclcpp::TimerBase::SharedPtr explorationTimer_;
		rclcpp::TimerBase::SharedPtr startExplorationTimer_;
		rclcpp::TimerBase::SharedPtr plannerTimer_;
		rclcpp::TimerBase::SharedPtr replanCheckTimer_;
		rclcpp::TimerBase::SharedPtr trajExeTimer_;
		rclcpp::TimerBase::SharedPtr visTimer_;

		// callback groups
		rclcpp::CallbackGroup::SharedPtr plannerCbGroup_;
		rclcpp::CallbackGroup::SharedPtr replanCbGroup_;
		rclcpp::CallbackGroup::SharedPtr trajExeCbGroup_;
		rclcpp::CallbackGroup::SharedPtr visCbGroup_;
		rclcpp::CallbackGroup::SharedPtr exploreReplanCbGroup_;
		rclcpp::CallbackGroup::SharedPtr startExplorationCbGroup_;

		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr polyTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pwlTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bsplineTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr inputTrajPub_;
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startExplorationSrv_;
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr endMissionSrv_;
		
		// parameters
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		double wpStablizeTime_;
		bool initialScan_;
		double replanTimeForDynamicObstacle_;
		double collisionReplanCooldownSec_{0.30};
		Eigen::Vector3d freeRange_;
		double reachGoalDistance_;
		double minWaypointDistance_{0.2};
		bool operatorConfirm_ = false;
		bool replanOnFinishOrFail_ = true;
		bool stabilizeBeforeRotate_ = true;
		bool replanOnCollisionFail_ = true;
		bool startExplorationRequested_ = false;
		bool explorationStarted_ = false;
		bool endMissionRequested_ = false;
		bool endMissionActive_ = false;
		bool landingPhase_ = false;
		bool missionEnded_ = false;
		double endMissionLastRetrySec_{-1.0};
		double endMissionMaxSegmentDistance_{1.5};

		// exploration data
		bool explorationReplan_ = true;
		bool replan_ = false;
		bool newWaypoints_ = false;
		int waypointIdx_ = 1;
		nav_msgs::msg::Path waypoints_; // latest waypoints from exploration planner
		nav_msgs::msg::Path inputTrajMsg_;
		nav_msgs::msg::Path polyTrajMsg_;
		nav_msgs::msg::Path pwlTrajMsg_;
		nav_msgs::msg::Path bsplineTrajMsg_;
		bool trajectoryReady_ = false;
		rclcpp::Time trajStartTime_;
		double trajTime_; // current trajectory time
		trajPlanner::bspline trajectory_;
		rclcpp::Time lastDynamicObstacleTime_;
		bool waypointRotatePending_ = false;
		rclcpp::Time waypointRotateReadyTime_;
		double waypointRotateYaw_ = 0.0;
		double lastCollisionReplanSec_{-1.0};
		std::mutex navStateMutex_;

		void clearWaypointPlan();
		nav_msgs::msg::Path buildTwoPointPath(double x, double y, double z) const;
		void requestExplorationReplan(bool enabled);
		void endMission();
	
	public:
		explicit dynamicExploration(const rclcpp::Node::SharedPtr& node);

		void initParam();
		void initModules();
		void registerCallback();
		void registerPub();

		void explorationCB();
		void plannerCB();
		void replanCheckCB();
		void trajExeCB();
		void visCB();

		void run();
		void initExplore();
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions);
		bool hasCollision();
		bool hasDynamicCollision();
		void exploreReplan();
		double computeExecutionDistance();
		bool replanForDynamicObstacle();
		bool reachExplorationGoal();
		bool isGoalValid();
		nav_msgs::msg::Path getCurrentTraj(double dt);
		nav_msgs::msg::Path getRestGlobalPath();
		nav_msgs::msg::Path getRestGlobalPath(const Eigen::Vector3d& pos);
		nav_msgs::msg::Path getRestGlobalPath(const Eigen::Vector3d& pos, double yaw);
	};
}
#endif