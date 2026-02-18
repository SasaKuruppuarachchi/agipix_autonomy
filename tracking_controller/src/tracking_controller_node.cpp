/*
	FILE: tracking_controller_node.cpp
	---------------------------------
	tracking controller for px4-based quadcopter
*/
#include <rclcpp/rclcpp.hpp>
#include <tracking_controller/trackingController.h>

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("tracking_controller_node");

	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
	executor.add_node(node);

	controller::trackingController tc(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}