/*
	FILE: dynamic_inspection_node.cpp
	-----------------------------
	dynamic inspection ROS node
*/

#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/dynamicInspection.h>

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("dynamic_inspection_node");

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);

	AutoFlight::dynamicInspection inspector (node);
	inspector.run();

	executor.spin();
	rclcpp::shutdown();
	return 0;
}