/*
	FILE: fakeDetector.h
	---------------------
	fake dynamic obtacle detector for gazebo simulation
*/
#ifndef FAKEDETECTOR_H
#define FAKEDETECTOR_H
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <onboard_detector/utils.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <thread>
#include <mutex>

using std::cout; using std::endl;

namespace onboardDetector{
	class fakeDetector{
	private:
		rclcpp::Node::SharedPtr node_;
		rclcpp::TimerBase::SharedPtr obstaclePubTimer_;
		rclcpp::TimerBase::SharedPtr visTimer_;
		rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr gazeboSub_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visPub_; // publish bounding box
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

		// callback groups
		rclcpp::CallbackGroup::SharedPtr gazeboCbGroup_;
		rclcpp::CallbackGroup::SharedPtr odomCbGroup_;
		rclcpp::CallbackGroup::SharedPtr visCbGroup_;

		std::vector<std::string> targetObstacle_;
		std::vector<int> targetIndex_;
		bool firstTime_;
		std::vector<onboardDetector::box3D> obstacleMsg_;
		std::vector<onboardDetector::box3D> lastObVec_;
		std::vector<rclcpp::Time> lastTimeVec_;
		std::vector<std::vector<double>> lastTimeVel_;

		// visualization:
		nav_msgs::msg::Odometry odom_;
		double colorDistance_;
		visualization_msgs::msg::MarkerArray visMsg_;

	public:
		explicit fakeDetector(const rclcpp::Node::SharedPtr& node);

		void visCB();
		void stateCB(const gazebo_msgs::msg::ModelStates::SharedPtr allStates);
		void odomCB(const nav_msgs::msg::Odometry::SharedPtr odom);
		std::vector<int>& findTargetIndex(const std::vector<std::string>& modelNames);
		void updateVisMsg();
		void publishObstacles();
		void publishVisualization();
		bool isObstacleInSensorRange(const onboardDetector::box3D& ob, double fov);
		void getObstacles(std::vector<onboardDetector::box3D>& obstacles);
		void getObstaclesInSensorRange(double fov, std::vector<onboardDetector::box3D>& obstacles);
	};
}

#endif