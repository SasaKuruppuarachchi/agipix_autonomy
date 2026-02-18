/*
	FILE: default_backends.h
	---------------------------------
	default backend implementations for controller interfaces
*/
#ifndef TRACKING_CONTROLLER_DEFAULT_BACKENDS_H
#define TRACKING_CONTROLLER_DEFAULT_BACKENDS_H

#include <rclcpp/rclcpp.hpp>
#include <tracking_controller/msg/target.hpp>
#include <tracking_controller/control_interfaces.h>

namespace controller {

class DdsSetpointSink final : public SetpointSink {
private:
	rclcpp::Node::SharedPtr node_;
	rclcpp::Publisher<tracking_controller::msg::Target>::SharedPtr targetPub_;

public:
	DdsSetpointSink(const rclcpp::Node::SharedPtr& node, const std::string& topic);

	void publishBodyRateThrust(const Eigen::Vector4d& cmd) override;
	void publishAttitudeThrust(const Eigen::Vector4d& attitudeQuat, double thrust) override;
	void publishAccelerationYaw(
		const Eigen::Vector3d& accRef,
		double yaw,
		const Eigen::Vector3d& posRef,
		const Eigen::Vector3d& velRef,
		std::uint8_t typeMask) override;
};

}  // namespace controller

#endif
