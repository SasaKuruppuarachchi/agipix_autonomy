/*
	FILE: dynamicNavigation.h
	------------------------
	dynamic navigation header file in real world
*/

#ifndef AUTOFLIGHT_DYNAMIC_NAVIGATION_H
#define AUTOFLIGHT_DYNAMIC_NAVIGATION_H

#include <autonomous_flight/px4/flightBase.h>
#include <map_manager/dynamicMap.h>
#include <global_planner/rrtOccMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>

namespace AutoFlight{
	class dynamicNavigation : public flightBase{
	private:
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<globalPlanner::rrtOccMap<3>> rrtPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		rclcpp::TimerBase::SharedPtr plannerTimer_;
		rclcpp::TimerBase::SharedPtr replanCheckTimer_;
		rclcpp::TimerBase::SharedPtr trajExeTimer_;
		rclcpp::TimerBase::SharedPtr visTimer_;

		// callback groups
		rclcpp::CallbackGroup::SharedPtr plannerCbGroup_;
		rclcpp::CallbackGroup::SharedPtr replanCbGroup_;
		rclcpp::CallbackGroup::SharedPtr trajExeCbGroup_;
		rclcpp::CallbackGroup::SharedPtr visCbGroup_;

		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rrtPathPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr polyTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pwlTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bsplineTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr inputTrajPub_;

		// parameters
		bool useGlobalPlanner_;
		bool noYawTurning_;
		bool useYawControl_;
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		double replanTimeForDynamicObstacle_;
		double collisionReplanCooldownSec_{0.30};
		std::string trajSavePath_;

		// navigation data
		bool replan_ = false;
		bool needGlobalPlan_ = false;
		bool globalPlanReady_ = false;
		nav_msgs::msg::Path rrtPathMsg_;
		nav_msgs::msg::Path polyTrajMsg_;
		nav_msgs::msg::Path pwlTrajMsg_;
		nav_msgs::msg::Path bsplineTrajMsg_;
		nav_msgs::msg::Path inputTrajMsg_;
		bool trajectoryReady_ = false;
		rclcpp::Time trajStartTime_;
		double trajTime_; // current trajectory time
		double prevInputTrajTime_ = 0.0;
		trajPlanner::bspline trajectory_; // trajectory data for tracking
		double facingYaw_;
		bool firstTimeSave_ = false;
		bool lastDynamicObstacle_ = false;
		rclcpp::Time lastDynamicObstacleTime_;
		double lastCollisionReplanSec_{-1.0};
		std::mutex navStateMutex_;
		



	public:
		explicit dynamicNavigation(const rclcpp::Node::SharedPtr& node);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();

		void plannerCB();
		void replanCheckCB();
		void trajExeCB();
		void visCB();

		void run();	
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions);	
		bool hasCollision();
		bool hasDynamicCollision();
		double computeExecutionDistance();
		bool replanForDynamicObstacle();
		nav_msgs::msg::Path getCurrentTraj(double dt);
		nav_msgs::msg::Path getRestGlobalPath();
	};
}

#endif