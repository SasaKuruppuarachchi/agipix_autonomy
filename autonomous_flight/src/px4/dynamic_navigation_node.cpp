/*
	FILE: dynamic navigation_node.cpp
	-----------------------------
	dynamic navigation ROS node
*/

#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/dynamicNavigation.h>

namespace {
class DynamicNavigationRunner {
public:
	DynamicNavigationRunner(const rclcpp::Node::SharedPtr& node, AutoFlight::dynamicNavigation* navigator)
	: node_(node), navigator_(navigator) {
		this->startCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->startTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(50),
			[this]() {
				this->startTimer_->cancel();
				RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Start dynamic navigation mission.");
				this->navigator_->run();
			},
			this->startCbGroup_);
	}

private:
	rclcpp::Node::SharedPtr node_;
	AutoFlight::dynamicNavigation* navigator_;
	rclcpp::CallbackGroup::SharedPtr startCbGroup_;
	rclcpp::TimerBase::SharedPtr startTimer_;
};
} // namespace

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("dynamic_navigation_node");

	AutoFlight::dynamicNavigation navigator (node);
	DynamicNavigationRunner runner(node, &navigator);

	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}