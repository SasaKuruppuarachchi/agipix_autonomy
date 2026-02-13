/*
*\tFile: rrt_interactive_node.cpp
*\t---------------
*   rrt interactive
*/
#include <rclcpp/rclcpp.hpp>
#include <global_planner/rrtOctomap.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mutex>

using std::cout;
using std::endl;

class RrtInteractiveNode {
public:
	RrtInteractiveNode()
	: node_(rclcpp::Node::make_shared("rrt_test_node"))
	, rrtplanner_(node_) {
		clickedCbGroup_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		planCbGroup_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		startVisCbGroup_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		goalVisCbGroup_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		rclcpp::SubscriptionOptions options;
		options.callback_group = clickedCbGroup_;
		goalPointSub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/move_base_simple/goal", 1000,
			std::bind(&RrtInteractiveNode::goalPointCB, this, std::placeholders::_1), options);
		startPointSub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"/initialpose", 1000,
			std::bind(&RrtInteractiveNode::startPointCB, this, std::placeholders::_1), options);

		startVisPub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/start_position", 1000);
		goalVisPub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/goal_position", 1000);

		startVisTimer_ = node_->create_wall_timer(
			std::chrono::milliseconds(100), std::bind(&RrtInteractiveNode::publishStartVis, this), startVisCbGroup_);
		goalVisTimer_ = node_->create_wall_timer(
			std::chrono::milliseconds(100), std::bind(&RrtInteractiveNode::publishGoalVis, this), goalVisCbGroup_);
		planTimer_ = node_->create_wall_timer(
			std::chrono::milliseconds(100), std::bind(&RrtInteractiveNode::planTimerCB, this), planCbGroup_);

		cout << rrtplanner_ << endl;
	}

	rclcpp::Node::SharedPtr getNode() const {
		return node_;
	}

private:
	void goalPointCB(const geometry_msgs::msg::PoseStamped::SharedPtr cp){
		std::lock_guard<std::mutex> lock(pointMutex_);
		if (!newStartMsg_){
			startPoint_[0] = cp->pose.position.x;
			startPoint_[1] = cp->pose.position.y;
			startPoint_[2] = 1.0;
			newStartMsg_ = true;
			newPlanRequested_ = true;
			awaitingStart_ = false;
			initStart_ = true;
			initGoal_ = false;
			startMarker_.header.frame_id = "map";
			startMarker_.header.stamp = node_->now();
			startMarker_.ns = "start_vis";
			startMarker_.id = 0;
			startMarker_.type = visualization_msgs::msg::Marker::SPHERE;
			startMarker_.action = visualization_msgs::msg::Marker::ADD;
			startMarker_.pose.position.x = startPoint_[0];
			startMarker_.pose.position.y = startPoint_[1];
			startMarker_.pose.position.z = startPoint_[2];
			startMarker_.lifetime = rclcpp::Duration::from_seconds(0.5);
			startMarker_.scale.x = 0.4;
			startMarker_.scale.y = 0.4;
			startMarker_.scale.z = 0.4;
			startMarker_.color.a = 0.7;
			startMarker_.color.r = 1.0;
			startMarker_.color.g = 0.5;
			startMarker_.color.b = 1.0;
			startWarned_ = false;
			return;
		}
			if (awaitingStart_){
				startPoint_[0] = cp->pose.position.x;
				startPoint_[1] = cp->pose.position.y;
				startPoint_[2] = 1.0;
				newStartMsg_ = true;
				newPlanRequested_ = true;
				awaitingStart_ = false;

				initStart_ = true;
				initGoal_ = false;
				startMarker_.header.frame_id = "map";
				startMarker_.header.stamp = node_->now();
				startMarker_.ns = "start_vis";
				startMarker_.id = 0;
				startMarker_.type = visualization_msgs::msg::Marker::SPHERE;
				startMarker_.action = visualization_msgs::msg::Marker::ADD;
				startMarker_.pose.position.x = startPoint_[0];
				startMarker_.pose.position.y = startPoint_[1];
				startMarker_.pose.position.z = startPoint_[2];
				startMarker_.lifetime = rclcpp::Duration::from_seconds(0.5);
				startMarker_.scale.x = 0.4;
				startMarker_.scale.y = 0.4;
				startMarker_.scale.z = 0.4;
				startMarker_.color.a = 0.7;
				startMarker_.color.r = 1.0;
				startMarker_.color.g = 0.5;
				startMarker_.color.b = 1.0;
				startWarned_ = false;
				return;
			}
		goalPoint_[0] = cp->pose.position.x;
		goalPoint_[1] = cp->pose.position.y;
		goalPoint_[2] = 1.0;
		newGoalMsg_ = true;
		newPlanRequested_ = true;

		initGoal_ = true;
		goalMarker_.header.frame_id = "map";
		goalMarker_.header.stamp = node_->now();
		goalMarker_.ns = "goal_vis";
		goalMarker_.id = 0;
		goalMarker_.type = visualization_msgs::msg::Marker::SPHERE;
		goalMarker_.action = visualization_msgs::msg::Marker::ADD;
		goalMarker_.pose.position.x = goalPoint_[0];
		goalMarker_.pose.position.y = goalPoint_[1];
		goalMarker_.pose.position.z = goalPoint_[2];
		goalMarker_.lifetime = rclcpp::Duration::from_seconds(0.5);
		goalMarker_.scale.x = 0.4;
		goalMarker_.scale.y = 0.4;
		goalMarker_.scale.z = 0.4;
		goalMarker_.color.a = 0.7;
		goalMarker_.color.r = 0.2;
		goalMarker_.color.g = 1.0;
		goalMarker_.color.b = 0.2;
		goalWarned_ = false;
	}

	void startPointCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr cp){
		std::lock_guard<std::mutex> lock(pointMutex_);
		startPoint_[0] = cp->pose.pose.position.x;
		startPoint_[1] = cp->pose.pose.position.y;
		startPoint_[2] = 1.0;
		newStartMsg_ = true;
		newPlanRequested_ = true;
		awaitingStart_ = false;

		initStart_ = true;
		initGoal_ = false;
		startMarker_.header.frame_id = "map";
		startMarker_.header.stamp = node_->now();
		startMarker_.ns = "start_vis";
		startMarker_.id = 0;
		startMarker_.type = visualization_msgs::msg::Marker::SPHERE;
		startMarker_.action = visualization_msgs::msg::Marker::ADD;
		startMarker_.pose.position.x = startPoint_[0];
		startMarker_.pose.position.y = startPoint_[1];
		startMarker_.pose.position.z = startPoint_[2];
		startMarker_.lifetime = rclcpp::Duration::from_seconds(0.5);
		startMarker_.scale.x = 0.4;
		startMarker_.scale.y = 0.4;
		startMarker_.scale.z = 0.4;
		startMarker_.color.a = 0.7;
		startMarker_.color.r = 1.0;
		startMarker_.color.g = 0.5;
		startMarker_.color.b = 1.0;
	}

	void publishStartVis(){
		if (initStart_){
			startVisPub_->publish(startMarker_);
		}
	}

	void publishGoalVis(){
		if (initGoal_){
			goalVisPub_->publish(goalMarker_);
		}
	}

	void planTimerCB(){
		std::vector<double> startPoint;
		std::vector<double> goalPoint;
		bool shouldPlan = false;
		{
			std::lock_guard<std::mutex> lock(pointMutex_);
			if (!newStartMsg_){
				if (!startWarned_){
					cout << "[Planner Node]: Wait for start point..." << endl;
					startWarned_ = true;
				}
				return;
			}
			if (!newGoalMsg_){
				if (!goalWarned_){
					cout << "[Planner Node]: Wait for goal point..." << endl;
					goalWarned_ = true;
				}
				return;
			}
			if (!newPlanRequested_ || planningInProgress_){
				return;
			}
			planningInProgress_ = true;
			newPlanRequested_ = false;
			startPoint = startPoint_;
			goalPoint = goalPoint_;
			shouldPlan = true;
			startWarned_ = false;
			goalWarned_ = false;
		}

		if (!shouldPlan){
			return;
		}

		cout << "----------------------------------------------------" << endl;
		cout << "[Planner Node]: Request No. " << planCount_ + 1 << endl;
		rrtplanner_.updateStart(startPoint);
		rrtplanner_.updateGoal(goalPoint);
		nav_msgs::msg::Path path;
		rrtplanner_.makePlan(path);
		++planCount_;
		cout << "----------------------------------------------------" << endl;

		{
			std::lock_guard<std::mutex> lock(pointMutex_);
			planningInProgress_ = false;
			newStartMsg_ = false;
			newGoalMsg_ = false;
			awaitingStart_ = true;
			initGoal_ = false;
		}
	}

	static constexpr int kDim = 3;

	rclcpp::Node::SharedPtr node_;
	globalPlanner::rrtOctomap<kDim> rrtplanner_;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalPointSub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr startPointSub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr startVisPub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goalVisPub_;
	rclcpp::TimerBase::SharedPtr startVisTimer_;
	rclcpp::TimerBase::SharedPtr goalVisTimer_;
	rclcpp::TimerBase::SharedPtr planTimer_;

	rclcpp::CallbackGroup::SharedPtr clickedCbGroup_;
	rclcpp::CallbackGroup::SharedPtr planCbGroup_;
	rclcpp::CallbackGroup::SharedPtr startVisCbGroup_;
	rclcpp::CallbackGroup::SharedPtr goalVisCbGroup_;

	std::mutex pointMutex_;
	bool newStartMsg_ = false;
	bool newGoalMsg_ = false;
	bool newPlanRequested_ = false;
	bool planningInProgress_ = false;
	bool startWarned_ = false;
	bool goalWarned_ = false;
	bool awaitingStart_ = true;
	std::vector<double> startPoint_ {0, 0, 1.0};
	std::vector<double> goalPoint_ {0, 0, 1.0};
	int planCount_ = 0;

	bool initStart_ = false;
	bool initGoal_ = false;
	visualization_msgs::msg::Marker startMarker_;
	visualization_msgs::msg::Marker goalMarker_;
};

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	RrtInteractiveNode node;
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node.getNode());
	executor.spin();
	rclcpp::shutdown();
	return 0;
}