/*
	FILE: default_backends.cpp
	---------------------------------
	default backend implementations for controller interfaces
*/

#include <tracking_controller/default_backends.h>

namespace controller {

DdsSetpointSink::DdsSetpointSink(const rclcpp::Node::SharedPtr& node, const std::string& topic)
	: node_(node) {
	this->targetPub_ = this->node_->create_publisher<tracking_controller::msg::Target>(topic, rclcpp::QoS(10));
}

void DdsSetpointSink::publishBodyRateThrust(const Eigen::Vector4d& /*cmd*/) {
	RCLCPP_WARN_THROTTLE(
		this->node_->get_logger(),
		*this->node_->get_clock(),
		2000,
		"[trackingController]: DDS sink ignores body-rate/thrust output; use acceleration mode for DDS path.");
}

void DdsSetpointSink::publishAttitudeThrust(const Eigen::Vector4d& /*attitudeQuat*/, double /*thrust*/) {
	RCLCPP_WARN_THROTTLE(
		this->node_->get_logger(),
		*this->node_->get_clock(),
		2000,
		"[trackingController]: DDS sink ignores attitude/thrust output; use acceleration mode for DDS path.");
}

void DdsSetpointSink::publishAccelerationYaw(
	const Eigen::Vector3d& accRef,
	double yaw,
	const Eigen::Vector3d& posRef,
	const Eigen::Vector3d& velRef,
	std::uint8_t typeMask) {
	tracking_controller::msg::Target msg;
	msg.header.stamp = this->node_->now();
	msg.header.frame_id = "drone0/map";
	msg.type_mask = typeMask;

	msg.position.x = posRef(0);
	msg.position.y = posRef(1);
	msg.position.z = posRef(2);
	msg.velocity.x = velRef(0);
	msg.velocity.y = velRef(1);
	msg.velocity.z = velRef(2);
	msg.acceleration.x = accRef(0);
	msg.acceleration.y = accRef(1);
	msg.acceleration.z = accRef(2);
	msg.yaw = static_cast<float>(yaw);

	this->targetPub_->publish(msg);
}

}  // namespace controller
