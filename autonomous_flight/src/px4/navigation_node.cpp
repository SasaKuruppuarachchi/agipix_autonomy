/*
	FILE: navigation_node.cpp
	-----------------------------
	navigation ROS node
*/

#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/navigation.h>

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("navigation_node");

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);

	AutoFlight::navigation navigator (node);
	navigator.run();

	executor.spin();
	rclcpp::shutdown();
	return 0;
}