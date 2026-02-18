/*
	FILE: takeoff_and_hover_node.cpp
	---------------------------
	Simple flight test for autonomous flight
*/

#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/flightBase.h>

namespace {
class TakeoffAndHoverRunner {
public:
	TakeoffAndHoverRunner(const rclcpp::Node::SharedPtr& node, AutoFlight::flightBase* fb)
	: node_(node), fb_(fb) {
		auto cbg = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->startTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(50),
			[this]() {
				this->startTimer_->cancel();
				RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Start takeoff-and-hover mission.");
				this->fb_->takeoff();
				RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Hover reached. Shutting down node.");
				rclcpp::shutdown();
			},
			cbg);
	}

private:
	rclcpp::Node::SharedPtr node_;
	AutoFlight::flightBase* fb_;
	rclcpp::TimerBase::SharedPtr startTimer_;
};
} // namespace

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("takeoff_and_hover_node");

	AutoFlight::flightBase fb (node);
	TakeoffAndHoverRunner runner(node, &fb);

	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
	executor.add_node(node);
	executor.spin();

	return 0;
}