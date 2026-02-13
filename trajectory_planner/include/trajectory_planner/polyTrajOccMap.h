/*
	FILE: polyTrajOccMap.h
	-------------------------
	minimun snap polynomial trajectory planner based on occupancy grid map
*/

#ifndef POLYTRAJPLANNEROCCMAP_H
#define POLYTRAJPLANNEROCCMAP_H
#include <rclcpp/rclcpp.hpp>
#include <trajectory_planner/utils.h>
#include <map_manager/occupancyMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/polyTrajSolver.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

namespace trajPlanner{
	class polyTrajOccMap{
	private:
		rclcpp::Node::SharedPtr node_;
		rclcpp::TimerBase::SharedPtr visTimer_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajVisPub_;
		rclcpp::CallbackGroup::SharedPtr visCallbackGroup_;
		rclcpp::Clock::SharedPtr clock_;
		rclcpp::Logger logger_{rclcpp::get_logger("polyTrajOccMap")};
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<trajPlanner::polyTrajSolver> trajSolver_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTrajSolver_;
		std::vector<pose> path_;
		nav_msgs::msg::Path trajVisMsg_;
		geometry_msgs::msg::Twist initVel_;
		geometry_msgs::msg::Twist endVel_;
		geometry_msgs::msg::Twist initAcc_;
		geometry_msgs::msg::Twist endAcc_;

		// parameters
		int polyDegree_;
		int diffDegree_;
		int continuityDegree_;
		double desiredVel_;
		double desiredAcc_;
		double initR_; // initial corridor constraint radius
		double timeout_;
		double corridorRes_;
		double fs_;
		bool softConstraint_;
		double softConstraintRadius_;
		double delT_;
		int maxIter_;
		bool usePWL_;

		// status
		bool findValidTraj_ = false;

	public:
		polyTrajOccMap(const rclcpp::Node::SharedPtr& node);
		void initParam();
		void registerPub();
		void registerCallback();
		void setMap(const std::shared_ptr<mapManager::occMap>& map);
		void initSolver();
		void initPWLSolver();

		// update desired vel
		void updateDesiredVel(double desiredVel);
		void updateDesiredAcc(double desiredAcc);

		// update waypoint path
		void updatePath(const nav_msgs::msg::Path& path);
		void updatePath(const nav_msgs::msg::Path& path, const std::vector<Eigen::Vector3d>& startEndCondition);
		void updatePath(const std::vector<pose>& path);

		// initial condition
		void updateInitVel(double vx, double vy, double vz);
		void updateInitVel(const geometry_msgs::msg::Twist& v);
		void updateEndVel(double vx, double vy, double vz);
		void updateEndVel(const geometry_msgs::msg::Twist& v);
		void updateInitAcc(double ax, double ay, double az);
		void updateInitAcc(const geometry_msgs::msg::Twist& a);
		void updateEndAcc(double ax, double ay, double az);
		void updateEndAcc(const geometry_msgs::msg::Twist& a);
		void setDefaultInit();

		bool makePlan(bool corridorConstraint);
		bool makePlan(std::vector<pose>& trajectory);
		bool makePlan(std::vector<pose>& trajectory, bool corridorConstraint);
		bool makePlan(nav_msgs::msg::Path& trajectory);
		bool makePlan(nav_msgs::msg::Path& trajectory, bool corridorConstraint);

		// visualization
		void visCB();
		void publishTrajVis();
		
		nav_msgs::msg::Path getTrajectory(double dt);
		geometry_msgs::msg::PoseStamped getPose(double t);
		Eigen::Vector3d getPos(double t);
		Eigen::Vector3d getVel(double t);
		Eigen::Vector3d getAcc(double t);
		double getDuration();
		bool checkCollisionTraj(const std::vector<pose>& trajectory, double delT, std::set<int>& collisionSeg);
		void adjustCorridorSize(const std::set<int>& collisionSeg, std::vector<double>& corridorSizeVec);
		void trajMsgConverter(const std::vector<pose>& trajectoryTemp, nav_msgs::msg::Path& trajectory);
	};
}

#endif