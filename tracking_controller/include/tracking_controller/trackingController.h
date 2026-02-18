/*
	FILE: trackingController.h
	-------------------------------
	function definition of px4 tracking controller
*/
#ifndef TRACKING_CONTROLLER_H
#define TRACKING_CONTROLLER_H
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <deque>
#include <queue>
#include <vector>
#include <memory>
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tracking_controller/msg/target.hpp>
#include <tracking_controller/utils.h>
#include <tracking_controller/control_interfaces.h>

using std::cout; using std::endl;
namespace controller{
	class trackingController{
		private:
			rclcpp::Node::SharedPtr node_;
			rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_; // Subscribe to odometry
			rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_; // IMU data subscriber
			rclcpp::Subscription<tracking_controller::msg::Target>::SharedPtr targetSub_; // subscriber for the tracking target states
			rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr poseVisPub_; // current pose publisher
			rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr targetVisPub_; // target pose publisher
			rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr histTrajVisPub_; // history trajectory publisher
			rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr targetHistTrajVisPub_; // target trajectory publisher
			rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velAndAccVisPub_; // velocity and acceleration visualization publisher
			rclcpp::TimerBase::SharedPtr cmdTimer_; // command timer
			rclcpp::TimerBase::SharedPtr thrustEstimatorTimer_; // thrust estimator timer
			rclcpp::TimerBase::SharedPtr visTimer_; // visualization timer

			// callback groups
			rclcpp::CallbackGroup::SharedPtr odomCbGroup_;
			rclcpp::CallbackGroup::SharedPtr imuCbGroup_;
			rclcpp::CallbackGroup::SharedPtr targetCbGroup_;
			rclcpp::CallbackGroup::SharedPtr cmdCbGroup_;
			rclcpp::CallbackGroup::SharedPtr thrustCbGroup_;
			rclcpp::CallbackGroup::SharedPtr visCbGroup_;

			// parameters
			bool bodyRateControl_ = false;
			bool attitudeControl_ = false;
			bool accControl_ = true;
			std::string backendType_ = "dds";
			std::string ddsTargetTopic_ = "/px4_control_interface/controller_target_state";
			std::string odomTopic_ = "/drone0/sensor_measurements/odom";
			std::string imuTopic_ = "/drone0/sensor_measurements/imu";
			Eigen::Vector3d pPos_, iPos_, dPos_;
			Eigen::Vector3d pVel_, iVel_, dVel_;
			double attitudeControlTau_;
			double hoverThrust_;
			bool verbose_;

			// controller data
			bool odomReceived_ = false;
			bool imuReceived_ = false;
			bool thrustReady_ = false;
			bool firstTargetReceived_ = false;
			bool targetReceived_ = false;
			bool firstTime_ = true;
			nav_msgs::msg::Odometry odom_;
			sensor_msgs::msg::Imu imuData_;
			tracking_controller::msg::Target target_;
			rclcpp::Time prevTime_;
			double deltaTime_;
			Eigen::Vector3d posErrorInt_; // integral of position error
			Eigen::Vector3d velErrorInt_; // integral of velocity error
			Eigen::Vector3d deltaPosError_, prevPosError_; // delta of position error
			Eigen::Vector3d deltaVelError_, prevVelError_; // delta of velocity error
			double cmdThrust_;
			rclcpp::Time cmdThrustTime_;

			// kalman filter
			bool kfFirstTime_ = true;
			rclcpp::Time kfStartTime_;
			double stateVar_ = 0.01;
			double processNoiseVar_ = 0.01;
			double measureNoiseVar_ = 0.02;
			std::deque<double> prevEstimateThrusts_;

			// visualization
			geometry_msgs::msg::PoseStamped poseVis_;
			std::deque<geometry_msgs::msg::PoseStamped> histTraj_;
			geometry_msgs::msg::PoseStamped targetPoseVis_;
			std::deque<geometry_msgs::msg::PoseStamped> targetHistTraj_;
			bool velFirstTime_ = true;
			Eigen::Vector3d prevVel_;
			rclcpp::Time velPrevTime_;
			std::unique_ptr<SetpointSink> setpointSink_;


		public:
			explicit trackingController(const rclcpp::Node::SharedPtr& node);
			void initParam();
			void registerPub();
			void registerCallback();

			// callback functions
			void odomCB(const nav_msgs::msg::Odometry::SharedPtr odom);
			void imuCB(const sensor_msgs::msg::Imu::SharedPtr imu);
			void targetCB(const tracking_controller::msg::Target::SharedPtr target);
			void cmdCB();
			void thrustEstimateCB();
			void visCB();

			void publishCommand(const Eigen::Vector4d& cmd);
			void publishCommand(const Eigen::Vector4d& cmd, const Eigen::Vector3d& accRef);
			void publishCommand(const Eigen::Vector3d& accRef);
			void computeAttitudeAndAccRef(Eigen::Vector4d& attitudeRefQuat, Eigen::Vector3d& accRef);
			void computeBodyRate(const Eigen::Vector4d& attitudeRefQuat, const Eigen::Vector3d& accRef, Eigen::Vector4d& cmd);

			// visualization
			void publishPoseVis();
			void publishHistTraj();
			void publishTargetVis();
			void publishTargetHistTraj();
			void publishVelAndAccVis();
	};
}

#endif