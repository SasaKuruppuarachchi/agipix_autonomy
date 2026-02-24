/*
	FILE: dynamic_inspection_node.cpp
	-----------------------------
	dynamic inspection ROS node
*/

#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/dynamicInspection.h>

namespace {
class DynamicInspectionRunner {
public:
	DynamicInspectionRunner(const rclcpp::Node::SharedPtr& node, AutoFlight::dynamicInspection* inspector)
	: node_(node), inspector_(inspector) {
		this->startCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->startTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(50),
			[this]() {
				this->startTimer_->cancel();
				RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Start dynamic inspection mission.");
				this->inspector_->run();
			},
			this->startCbGroup_);
	}

private:
	rclcpp::Node::SharedPtr node_;
	AutoFlight::dynamicInspection* inspector_;
	rclcpp::CallbackGroup::SharedPtr startCbGroup_;
	rclcpp::TimerBase::SharedPtr startTimer_;
};
} // namespace

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("dynamic_inspection_node");

	AutoFlight::dynamicInspection inspector (node);
	DynamicInspectionRunner runner(node, &inspector);

	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}