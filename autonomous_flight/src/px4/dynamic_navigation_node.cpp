/*
	FILE: dynamic navigation_node.cpp
	-----------------------------
	dynamic navigation ROS node
*/

#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/dynamicNavigation.h>

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("dynamic_navigation_node");

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);

	AutoFlight::dynamicNavigation navigator (node);
	navigator.run();

	executor.spin();
	rclcpp::shutdown();
	return 0;
}