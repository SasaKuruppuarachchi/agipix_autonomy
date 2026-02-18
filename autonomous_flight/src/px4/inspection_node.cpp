#include <rclcpp/rclcpp.hpp>
#include <autonomous_flight/px4/inspection.h>

namespace {
class InspectionRunner {
public:
	InspectionRunner(const rclcpp::Node::SharedPtr& node, AutoFlight::inspector* inspector)
	: node_(node), inspector_(inspector) {
		auto cbg = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->startTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(100),
			[this]() {
				this->startTimer_->cancel();
				RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Starting inspection mission.");
				this->inspector_->run();
				this->missionStarted_ = true;
			},
			cbg);

		this->finishTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(200),
			[this]() {
				if (!this->missionStarted_) {
					return;
				}
				if (this->inspector_->isMissionFinished()) {
					RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Inspection mission finished. Shutting down node.");
					this->finishTimer_->cancel();
					rclcpp::shutdown();
				}
			},
			cbg);
	}

private:
	rclcpp::Node::SharedPtr node_;
	AutoFlight::inspector* inspector_;
	rclcpp::TimerBase::SharedPtr startTimer_;
	rclcpp::TimerBase::SharedPtr finishTimer_;
	bool missionStarted_ = false;
};
}


int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("auto_inspection_node");

	AutoFlight::inspector ip (node);
	InspectionRunner runner(node, &ip);

	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
