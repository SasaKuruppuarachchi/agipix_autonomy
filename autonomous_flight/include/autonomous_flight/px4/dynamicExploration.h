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


namespace AutoFlight{
	class dynamicExploration : flightBase{
	private:
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<globalPlanner::DEP> expPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		rclcpp::TimerBase::SharedPtr explorationTimer_;
		rclcpp::TimerBase::SharedPtr plannerTimer_;
		rclcpp::TimerBase::SharedPtr replanCheckTimer_;
		rclcpp::TimerBase::SharedPtr trajExeTimer_;
		rclcpp::TimerBase::SharedPtr visTimer_;

		// callback groups
		rclcpp::CallbackGroup::SharedPtr plannerCbGroup_;
		rclcpp::CallbackGroup::SharedPtr replanCbGroup_;
		rclcpp::CallbackGroup::SharedPtr trajExeCbGroup_;
		rclcpp::CallbackGroup::SharedPtr visCbGroup_;

		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr polyTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pwlTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bsplineTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr inputTrajPub_;
		
		// parameters
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		double wpStablizeTime_;
		bool initialScan_;
		double replanTimeForDynamicObstacle_;
		Eigen::Vector3d freeRange_;
		double reachGoalDistance_;

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
	
	public:
		std::thread exploreReplanWorker_;
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
		void waitTime(double time);
	};
}
#endif