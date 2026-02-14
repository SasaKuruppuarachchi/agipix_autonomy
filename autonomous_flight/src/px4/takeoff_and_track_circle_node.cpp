/*
	FILE: takeoff_and_track_circle_node.cpp
	---------------------------
	Simple flight test for autonomous flight
*/

#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/flightBase.h>

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("takeoff_and_track_circle_node");

	AutoFlight::flightBase fb (node);

	fb.takeoff();
	fb.circle();
	fb.stop();

	rclcpp::shutdown();
	return 0;
}