/*
	FILE: flightBase.h
	-------------------
	base implementation for autonomous flight
*/

#ifndef FLIGHTBASE_H
#define FLIGHTBASE_H
#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/utils.h>
#include <autonomous_flight/msg/target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <string>
#include <atomic>

using std::cout; using std::endl;
namespace AutoFlight{
	class flightBase{
	protected:
		rclcpp::Node::SharedPtr node_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clickSub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clickSubLegacy_;
		rclcpp::Publisher<autonomous_flight::msg::Target>::SharedPtr statePub_;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr targetMarkerPub_;
		rclcpp::TimerBase::SharedPtr stateUpdateTimer_;
		rclcpp::TimerBase::SharedPtr targetPubTimer_;
		std::mutex targetStateMutex_;

		// callback groups (critical callbacks are mutually exclusive)
		rclcpp::CallbackGroup::SharedPtr stateCbGroup_;
		rclcpp::CallbackGroup::SharedPtr odomCbGroup_;
		rclcpp::CallbackGroup::SharedPtr clickCbGroup_;
		rclcpp::CallbackGroup::SharedPtr targetPubCbGroup_;
		rclcpp::CallbackGroup::SharedPtr stateUpdateCbGroup_;
		std::atomic_bool hasStateTarget_{false};
		
		nav_msgs::msg::Odometry odom_;
		geometry_msgs::msg::PoseStamped poseTgt_;
		autonomous_flight::msg::Target stateTgt_;
		geometry_msgs::msg::PoseStamped goal_;
		Eigen::Vector3d currPos_;
		double currYaw_;
		Eigen::Vector3d currVel_, currAcc_, prevVel_; 
		rclcpp::Time prevStateTime_;
		bool stateUpdateFirstTime_ = true;
		
		// parameters
		double takeoffHgt_;
		bool waitForTopicsReady_ = true;
		double takeoffWaitTimeoutSec_ = 8.0;
		bool requireTakeoffFeedback_ = false;
		std::string mapFrameId_ = "map";
		std::string baseLinkFrameId_ = "base_link";
		std::string odomTopic_ = "sensor_measurements/odom";
		std::string goalTopic_ = "goal_pose";
		std::string legacyGoalTopic_ = "move_base_simple/goal";
		bool subscribeLegacyGoalTopic_ = true;
		bool yawControl_;
		int timeStep_;
		double radius_;
		double velocity_;
		bool skipTakeoffIfFlying_ = true;
		double flyingHeightThreshold_ = 0.35;
		bool publishTargetMarker_ = false;
		std::string targetMarkerTopic_ = "autonomous_flight/target_state_marker";
		double targetMarkerScale_ = 0.20;
		int targetQosDepth_ = 1;
		std::string targetQosReliability_ = "best_effort";
		std::string targetQosDurability_ = "volatile";

		// status
		bool odomReceived_ = false;
		bool firstGoal_ = false;
		bool goalReceived_ = false;


	public:

		explicit flightBase(const rclcpp::Node::SharedPtr& node);
		
		void publishTarget();
		void publishTargetMarker(const autonomous_flight::msg::Target& target);

		// callback functions
		void odomCB(const nav_msgs::msg::Odometry::SharedPtr odom);
		void clickCB(const geometry_msgs::msg::PoseStamped::SharedPtr cp);
		void stateUpdateCB();

		void takeoff();
		void circle();
		void run(); // in flight base, this is a trajectory test function
		void stop(); // stop at the current position
		void moveToOrientation(double yaw, double desiredAngularVel);

		void updateTarget(const geometry_msgs::msg::PoseStamped& ps);
		void updateTargetWithState(const autonomous_flight::msg::Target& target);
		bool isReach(const geometry_msgs::msg::PoseStamped& poseTgt, bool useYaw=true);
		bool isReach(const geometry_msgs::msg::PoseStamped& poseTgt, double dist, bool useYaw=true);
	};

	struct trajData{
		nav_msgs::msg::Path trajectory;
		nav_msgs::msg::Path currTrajectory;
		rclcpp::Time startTime;
		double tCurr;
		double duration;
		double timestep;
		bool init = false;
		int forwardIdx = 3;
		int minIdx = 5;

		void updateTrajectory(const nav_msgs::msg::Path& _trajectory, double _duration){
			this->trajectory = _trajectory;
			this->currTrajectory = _trajectory;
			this->duration = _duration;
			this->tCurr = 0.0;
			this->startTime = rclcpp::Clock(RCL_ROS_TIME).now();
			this->timestep = this->duration/(this->trajectory.poses.size()-1);
			if (not this->init){
				this->init = true;
			}
		}

		int getCurrIdx(){
			rclcpp::Time currTime = rclcpp::Clock(RCL_ROS_TIME).now();
			this->tCurr = (currTime - this->startTime).seconds() + this->timestep;
			if (this->tCurr > this->duration){
				this->tCurr = this->duration;
			}

			int idx = floor(this->tCurr/this->timestep);
			return idx;
		}

		autonomous_flight::msg::Target getState(){
			autonomous_flight::msg::Target target;
			geometry_msgs::msg::PoseStamped ps = this->getPose();
			target.position.x = ps.pose.position.x;
			target.position.y = ps.pose.position.y;
			target.position.z = ps.pose.position.z;
			target.velocity.x = 0.0;
			target.velocity.y = 0.0;
			target.velocity.z = 0.0;
			target.acceleration.x = 0.0;
			target.acceleration.y = 0.0;
			target.acceleration.z = 0.0;
			target.yaw = AutoFlight::rpy_from_quaternion(ps.pose.orientation);
			return target;
		}

		autonomous_flight::msg::Target getState(const geometry_msgs::msg::Pose& psCurr){
			autonomous_flight::msg::Target target;
			geometry_msgs::msg::PoseStamped ps = this->getPose(psCurr);
			target.position.x = ps.pose.position.x;
			target.position.y = ps.pose.position.y;
			target.position.z = ps.pose.position.z;
			target.velocity.x = 0.0;
			target.velocity.y = 0.0;
			target.velocity.z = 0.0;
			target.acceleration.x = 0.0;
			target.acceleration.y = 0.0;
			target.acceleration.z = 0.0;
			target.yaw = AutoFlight::rpy_from_quaternion(ps.pose.orientation);
			return target;
		}

		autonomous_flight::msg::Target getStateWithoutYaw(const geometry_msgs::msg::Pose& psCurr){
			autonomous_flight::msg::Target target;
			geometry_msgs::msg::PoseStamped ps = this->getPoseWithoutYaw(psCurr);
			target.position.x = ps.pose.position.x;
			target.position.y = ps.pose.position.y;
			target.position.z = ps.pose.position.z;
			target.velocity.x = 0.0;
			target.velocity.y = 0.0;
			target.velocity.z = 0.0;
			target.acceleration.x = 0.0;
			target.acceleration.y = 0.0;
			target.acceleration.z = 0.0;
			target.yaw = AutoFlight::rpy_from_quaternion(ps.pose.orientation);
			return target;
		}

		geometry_msgs::msg::PoseStamped getPose(){
			int idx = this->getCurrIdx();
			idx = std::max(idx+forwardIdx, minIdx);
			int newIdx = std::min(idx, int(this->trajectory.poses.size()-1));

			std::vector<geometry_msgs::msg::PoseStamped> pathVec;
			for (size_t i=idx; i<this->trajectory.poses.size(); ++i){
				pathVec.push_back(this->trajectory.poses[i]);
			}
			this->currTrajectory.poses = pathVec;
			return this->trajectory.poses[newIdx];
		}

		geometry_msgs::msg::PoseStamped getPose(const geometry_msgs::msg::Pose& psCurr){
			int idx = this->getCurrIdx();
			idx = std::max(idx+forwardIdx, minIdx);
			int newIdx = std::min(idx, int(this->trajectory.poses.size()-1));
			if (newIdx > int(this->trajectory.poses.size()) - 1){
				geometry_msgs::msg::PoseStamped ps;
				ps.pose = psCurr;
				return ps;
			}
			std::vector<geometry_msgs::msg::PoseStamped> pathVec;
			geometry_msgs::msg::PoseStamped psFirst;
			psFirst.pose = psCurr;
			pathVec.push_back(psFirst);
			for (size_t i=idx; i<this->trajectory.poses.size(); ++i){
				pathVec.push_back(this->trajectory.poses[i]);
			}
			this->currTrajectory.poses = pathVec;
			return this->trajectory.poses[newIdx];
		}

		geometry_msgs::msg::PoseStamped getPoseWithoutYaw(const geometry_msgs::msg::Pose& psCurr){
			int idx = this->getCurrIdx();
			idx = std::max(idx+forwardIdx, minIdx);
			int newIdx = std::min(idx, int(this->trajectory.poses.size()-1));
			if (newIdx > int(this->trajectory.poses.size()) - 1){
				geometry_msgs::msg::PoseStamped ps;
				ps.pose = psCurr;
				return ps;
			}

			std::vector<geometry_msgs::msg::PoseStamped> pathVec;
			geometry_msgs::msg::PoseStamped psFirst;
			psFirst.pose = psCurr;
			pathVec.push_back(psFirst);
			for (size_t i=idx; i<this->trajectory.poses.size(); ++i){
				pathVec.push_back(this->trajectory.poses[i]);
			}
			this->currTrajectory.poses = pathVec;
			geometry_msgs::msg::PoseStamped psTarget = this->trajectory.poses[newIdx];
			psTarget.pose.orientation = psCurr.orientation;
			return psTarget;	
		}

		void stop(const geometry_msgs::msg::Pose& psCurr){
			std::vector<geometry_msgs::msg::PoseStamped> pathVec;
			geometry_msgs::msg::PoseStamped ps;
			ps.pose = psCurr;
			pathVec.push_back(ps);
			this->trajectory.poses = pathVec;
			this->currTrajectory = this->trajectory;
			this->tCurr = 0.0;
			this->startTime = rclcpp::Clock(RCL_ROS_TIME).now();
			this->duration = this->timestep; 
		}

			double getRemainTime(){
			rclcpp::Time currTime = rclcpp::Clock(RCL_ROS_TIME).now();
			double tCurr = (currTime - this->startTime).seconds() + this->timestep;
			return this->duration - tCurr;
		}

		bool needReplan(double factor){
			if (this->getRemainTime() <= this->duration * (1 - factor)){
				return true;
			}
			else{
				return false;
			}
		}
	};
}

#endif
