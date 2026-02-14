#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/inspection.h>


int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("auto_inspection_node");

	AutoFlight::inspector ip (node);
	ip.run();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
