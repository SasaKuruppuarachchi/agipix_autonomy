/*
	FILE: navigation.h
	------------------------
	navigation header file in px4
*/

#ifndef AUTOFLIGHT_NAVIGATION_H
#define AUTOFLIGHT_NAVIGATION_H

#include <autonomous_flight/px4/flightBase.h>
#include <map_manager/occupancyMap.h>
#include <global_planner/rrtOccMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <time_optimizer/trajectoryDivider.h>
#include <time_optimizer/bsplineTimeOptimizer.h>

namespace AutoFlight{
	class navigation : public flightBase{
	private:
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<globalPlanner::rrtOccMap<3>> rrtPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;
		std::shared_ptr<timeOptimizer::trajDivider> trajDivider_;
		std::shared_ptr<timeOptimizer::bsplineTimeOptimizer> timeOptimizer_;



		rclcpp::TimerBase::SharedPtr plannerTimer_;
		rclcpp::TimerBase::SharedPtr replanCheckTimer_;
		rclcpp::TimerBase::SharedPtr trajExeTimer_;
		rclcpp::TimerBase::SharedPtr visTimer_;

		// callback groups (mutually exclusive for critical callbacks)
		rclcpp::CallbackGroup::SharedPtr plannerCbGroup_;
		rclcpp::CallbackGroup::SharedPtr replanCbGroup_;
		rclcpp::CallbackGroup::SharedPtr trajExeCbGroup_;
		rclcpp::CallbackGroup::SharedPtr visCbGroup_;

		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rrtPathPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr polyTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pwlTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bsplineTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr inputTrajPub_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr inputTrajPointsPub_;

		// parameters
		bool useGlobalPlanner_;
		bool noYawTurning_;
		bool useYawControl_;
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		std::string trajSavePath_;
		bool useTimeOptimizer_;
		int maxConsecutivePlanFailuresBeforeStop_{3};
		double collisionReplanCooldownSec_{0.30};

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
		double facingYaw_;
		trajPlanner::bspline trajectory_; // trajectory data for tracking
		bool firstTimeSave_ = false;
		int consecutivePlanFailureCount_{0};
		double lastCollisionReplanSec_{-1.0};
		std::mutex navStateMutex_;
		


	public:
		explicit navigation(const rclcpp::Node::SharedPtr& node);
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
		double computeExecutionDistance();
		nav_msgs::msg::Path getCurrentTraj(double dt);
		nav_msgs::msg::Path getRestGlobalPath();
		void publishInputTraj();
	};
}

#endif