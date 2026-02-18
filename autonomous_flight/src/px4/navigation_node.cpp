/*
	FILE: navigation_node.cpp
	-----------------------------
	navigation ROS node
*/

#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/navigation.h>

namespace {
class NavigationRunner {
public:
	NavigationRunner(const rclcpp::Node::SharedPtr& node, AutoFlight::navigation* navigation)
	: node_(node), navigation_(navigation) {
		auto cbg = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->startTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(50),
			[this]() {
				this->startTimer_->cancel();
				RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Start navigation mission.");
				this->navigation_->run();
			},
			cbg);
	}

private:
	rclcpp::Node::SharedPtr node_;
	AutoFlight::navigation* navigation_;
	rclcpp::TimerBase::SharedPtr startTimer_;
};
} // namespace

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("navigation_node");

	AutoFlight::navigation navigation (node);
	NavigationRunner runner(node, &navigation);

	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}