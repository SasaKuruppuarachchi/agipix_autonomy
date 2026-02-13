/*
	FILE: takeoff_and_hover_node.cpp
	---------------------------
	Simple flight test for autonomous flight
*/

#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/flightBase.h>

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("takeoff_and_hover_node");

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);

	AutoFlight::flightBase fb (node);
	std::thread spinner([&executor](){ executor.spin(); });
	
	fb.takeoff();

	executor.cancel();
	spinner.join();
	rclcpp::shutdown();
	return 0;
}