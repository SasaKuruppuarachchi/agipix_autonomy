/*
	FILE: dynamic_exploration_node.cpp
	-----------------------------
	dynamic exploration ROS node
*/

#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/dynamicExploration.h>

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("dynamic_exploration_node");

	AutoFlight::dynamicExploration explorer (node);
	explorer.run();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}