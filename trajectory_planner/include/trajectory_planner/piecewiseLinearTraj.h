/*
	File: piecewiseLinearTraj.h
	----------------------------
	generate piecewise-linear trajectory from waypoint path
*/

#ifndef PIECEWISELINEARTRAJ_H
#define PIECEWISELINEARTRAJ_H

#include <rclcpp/rclcpp.hpp>
#include <trajectory_planner/utils.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/path.hpp>

using std::cout; using std::endl;

namespace trajPlanner{
	class pwlTraj{
	private:
		rclcpp::Node::SharedPtr node_;
		rclcpp::Clock::SharedPtr clock_;
		rclcpp::Logger logger_{rclcpp::get_logger("pwlTraj")};
		double desiredVel_ = 1.0;
		double desiredAngularVel_ = 0.5;
		std::vector<trajPlanner::pose> path_;
		std::vector<double> desiredTime_;


	public:
		pwlTraj(const rclcpp::Node::SharedPtr& node);
		void updatePath(const nav_msgs::msg::Path& path, bool useYaw=false);
		void updatePath(const std::vector<trajPlanner::pose>& path, bool useYaw=false);
		void updatePath(const nav_msgs::msg::Path& path, double desiredVel, bool useYaw=false);
		void updatePath(const std::vector<trajPlanner::pose>& path, double desiredVel, bool useYaw=false);	
		void avgTimeAllocation(bool useYaw=false);
		void avgTimeAllocation(double desiredVel, bool useYaw=false);
		void adjustHeading(const geometry_msgs::msg::Quaternion& quat);
		void adjustHeading(double yaw);

		void makePlan(nav_msgs::msg::Path& trajectory, double delT);
		void makePlan(std::vector<trajPlanner::pose>& trajectory, double delT);

		geometry_msgs::msg::PoseStamped getPose(double t);
		std::vector<double> getTimeKnot();
		double getDuration();
		double getDesiredVel();
		double getDesiredAngularVel();
		geometry_msgs::msg::PoseStamped getFirstPose();
	};
}


#endif