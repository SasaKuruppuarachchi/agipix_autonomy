/*
	FILE: dynamic_exploration_node.cpp
	-----------------------------
	dynamic exploration ROS node
*/

#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/dynamicExploration.h>

namespace {
class DynamicExplorationRunner {
public:
	DynamicExplorationRunner(const rclcpp::Node::SharedPtr& node, AutoFlight::dynamicExploration* explorer)
	: node_(node), explorer_(explorer) {
		this->startCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->startTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(50),
			[this]() {
				this->startTimer_->cancel();
				RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Start dynamic exploration mission.");
				this->explorer_->run();
			},
			this->startCbGroup_);
	}

private:
	rclcpp::Node::SharedPtr node_;
	AutoFlight::dynamicExploration* explorer_;
	rclcpp::CallbackGroup::SharedPtr startCbGroup_;
	rclcpp::TimerBase::SharedPtr startTimer_;
};
} // namespace

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("dynamic_exploration_node");

	AutoFlight::dynamicExploration explorer (node);
	DynamicExplorationRunner runner(node, &explorer);

	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}