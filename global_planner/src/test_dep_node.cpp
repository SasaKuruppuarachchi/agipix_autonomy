/*
*	File: test_dep_node.cpp
*	---------------
*   dynamic exploration planner test
*/

#include <rclcpp/rclcpp.hpp>
#include <global_planner/dep.h>
#include <iostream>

class DepTestNode {
public:
	DepTestNode()
	: node_(rclcpp::Node::make_shared("dep_test_node"))
	, map_(std::make_shared<mapManager::dynamicMap>(node_, true)) {
		planCbGroup_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		map_->initMap(true);
		startTime_ = node_->now();
		startupTimer_ = node_->create_wall_timer(
			std::chrono::milliseconds(100), std::bind(&DepTestNode::startupCB, this), planCbGroup_);
	}

	rclcpp::Node::SharedPtr getNode() const {
		return node_;
	}

private:
	void startupCB(){
		if ((node_->now() - startTime_).seconds() < 2.0){
			return;
		}
		startupTimer_->cancel();
		planner_ = std::make_shared<globalPlanner::DEP>(node_);
		planner_->setMap(map_);
		planTimer_ = node_->create_wall_timer(
			std::chrono::milliseconds(500), std::bind(&DepTestNode::planCB, this), planCbGroup_);
	}

	void planCB(){
		if (planningInProgress_){
			return;
		}
		planningInProgress_ = true;
		planner_->setMap(map_);
		std::cout << "start planning." << std::endl;
		rclcpp::Time startTime = node_->now();
		planner_->makePlan();
		rclcpp::Time endTime = node_->now();
		std::cout << "planning time: " << (endTime - startTime).seconds() << "s." << std::endl;
		std::cout << "end planning." << std::endl;
		planningInProgress_ = false;
	}

	rclcpp::Node::SharedPtr node_;
	std::shared_ptr<mapManager::dynamicMap> map_;
	std::shared_ptr<globalPlanner::DEP> planner_;
	bool planningInProgress_ = false;
	rclcpp::Time startTime_;

	rclcpp::CallbackGroup::SharedPtr planCbGroup_;
	rclcpp::TimerBase::SharedPtr startupTimer_;
	rclcpp::TimerBase::SharedPtr planTimer_;
};

int main(int argc, char**argv){
	rclcpp::init(argc, argv);
	DepTestNode node;
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node.getNode());
	executor.spin();
	rclcpp::shutdown();

	return 0;
}