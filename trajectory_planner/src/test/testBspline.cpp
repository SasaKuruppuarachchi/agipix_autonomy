#include <rclcpp/rclcpp.hpp>
#include <trajectory_planner/bsplineTraj.h>
#include <trajectory_planner/bspline.h>
#include <global_planner/rrtOctomap.h>
#include <trajectory_planner/polyTrajOctomap.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mutex>
#include <thread>

using std::cout;
using std::endl;

class TestBsplineNode : public rclcpp::Node {
public:
	TestBsplineNode() : rclcpp::Node("test_bspline_node") {
		subCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		startVisCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		goalVisCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		mainCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		rclcpp::SubscriptionOptions subOptions;
		subOptions.callback_group = subCbGroup_;
		clickedPointSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/move_base_simple/goal",
			1000,
			std::bind(&TestBsplineNode::clickedPointCB, this, std::placeholders::_1),
			subOptions);

		posePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/trajectory_pose", 1000);
		startVisPub_ = this->create_publisher<visualization_msgs::msg::Marker>("/start_position", 1000);
		goalVisPub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_position", 1000);

		startVisTimer_ = this->create_wall_timer(
			std::chrono::duration<double>(0.1),
			std::bind(&TestBsplineNode::publishStartVis, this),
			startVisCbGroup_);
		goalVisTimer_ = this->create_wall_timer(
			std::chrono::duration<double>(0.1),
			std::bind(&TestBsplineNode::publishGoalVis, this),
			goalVisCbGroup_);

		mainTimer_ = this->create_wall_timer(
			std::chrono::duration<double>(0.1),
			std::bind(&TestBsplineNode::mainTimerCB, this),
			mainCbGroup_);

		initialized_ = false;
	}

	void run(){
		initPlanners();

		cout << "test bspline" << endl;
		double timestep = 0.1;
		int degree = 3;
		Eigen::MatrixXd controlPoints;
		int size = 10;
		controlPoints.resize(3, size);
		for (int i=0; i<size; ++i){
			controlPoints(0, i) = i;
			controlPoints(1, i) = i;
			controlPoints(2, i) = i;
		}

		cout << "control points: " << endl;
		cout << controlPoints << endl;
		trajPlanner::bspline bs (degree, controlPoints, timestep);

		for (int i=0; i<4; ++i){
			Eigen::Vector3d con (0, 0, 0);
			startEndCondition_.push_back(con);
		}
	}

private:
	void initPlanners(){
		if (initialized_) return;
		auto self = shared_from_this();
		rrtPlanner_ = std::make_shared<globalPlanner::rrtOctomap<3>>(self);
		cout << *rrtPlanner_ << endl;
		polyPlanner_ = std::make_shared<trajPlanner::polyTrajOctomap>(self);
		cout << *polyPlanner_ << endl;
		bsplineTraj_ = std::make_shared<trajPlanner::bsplineTraj>(self);
		initialized_ = true;
	}

	void clickedPointCB(const geometry_msgs::msg::PoseStamped::SharedPtr cp){
		std::lock_guard<std::mutex> lock(msgMutex_);
		newPoint_[0] = cp->pose.position.x;
		newPoint_[1] = cp->pose.position.y;
		newPoint_[2] = 1.0;
		newMsg_ = true;
	}

	bool consumeNewMsg(){
		std::lock_guard<std::mutex> lock(msgMutex_);
		if (!newMsg_) return false;
		newMsg_ = false;
		return true;
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

	void mainTimerCB(){
		if (!initialized_){
			initPlanners();
			return;
		}

		switch (state_){
			case PlanState::WAIT_START:
				if (consumeNewMsg()){
					start_ = newPoint_;
					rrtPlanner_->updateStart(start_);
					cout << "----------------------------------------------------" << endl;
					cout << "[Planner Node]: Request No. " << countLoop_ + 1 << endl;
					cout << "[Planner Node]: start point OK. (" << start_[0] << " " << start_[1] << " " << start_[2] << ")" << endl;

					initStart_ = true;
					initGoal_ = false;
					startMarker_.header.frame_id = "map";
					startMarker_.header.stamp = this->now();
					startMarker_.ns = "start_vis";
					startMarker_.id = 0;
					startMarker_.type = visualization_msgs::msg::Marker::SPHERE;
					startMarker_.action = visualization_msgs::msg::Marker::ADD;
					startMarker_.pose.position.x = start_[0];
					startMarker_.pose.position.y = start_[1];
					startMarker_.pose.position.z = start_[2];
					startMarker_.lifetime = rclcpp::Duration::from_seconds(0.5);
					startMarker_.scale.x = 0.4;
					startMarker_.scale.y = 0.4;
					startMarker_.scale.z = 0.4;
					startMarker_.color.a = 0.7;
					startMarker_.color.r = 1.0;
					startMarker_.color.g = 0.5;
					startMarker_.color.b = 1.0;

					state_ = PlanState::WAIT_GOAL;
				}
				break;
			case PlanState::WAIT_GOAL:
				if (consumeNewMsg()){
					goal_ = newPoint_;
					rrtPlanner_->updateGoal(goal_);
					cout << "[Planner Node]: goal point OK. (" << goal_[0] << " " << goal_[1] << " " << goal_[2] << ")" << endl;

					initGoal_ = true;
					goalMarker_.header.frame_id = "map";
					goalMarker_.header.stamp = this->now();
					goalMarker_.ns = "goal_vis";
					goalMarker_.id = 0;
					goalMarker_.type = visualization_msgs::msg::Marker::SPHERE;
					goalMarker_.action = visualization_msgs::msg::Marker::ADD;
					goalMarker_.pose.position.x = goal_[0];
					goalMarker_.pose.position.y = goal_[1];
					goalMarker_.pose.position.z = goal_[2];
					goalMarker_.lifetime = rclcpp::Duration::from_seconds(0.5);
					goalMarker_.scale.x = 0.4;
					goalMarker_.scale.y = 0.4;
					goalMarker_.scale.z = 0.4;
					goalMarker_.color.a = 0.7;
					goalMarker_.color.r = 0.2;
					goalMarker_.color.g = 1.0;
					goalMarker_.color.b = 0.2;

					state_ = PlanState::PLAN;
				}
				break;
			case PlanState::PLAN:
				planOnce();
				++countLoop_;
				cout << "----------------------------------------------------" << endl;
				state_ = PlanState::WAIT_START;
				break;
		}
	}

	void planOnce(){
		nav_msgs::msg::Path path;
		rrtPlanner_->makePlan(path);
		polyPlanner_->updatePath(path);
		nav_msgs::msg::Path trajectory;
		polyPlanner_->makePlan(trajectory);
		double duration = polyPlanner_->getDuration();
		cout << "[Planner Node]: Duration: " << duration << "s." << endl;
		// bsplineTraj_->updatePath(trajectory, startEndCondition_);
		// bsplineFitPath_ = bsplineTraj_->evalTrajToMsg();
	}

	std::mutex msgMutex_;
	bool newMsg_{false};
	std::vector<double> newPoint_{0, 0, 1.0};
	std::vector<double> start_{0, 0, 1.0};
	std::vector<double> goal_{0, 0, 1.0};
	int countLoop_{0};
	std::vector<Eigen::Vector3d> startEndCondition_;

	enum class PlanState { WAIT_START, WAIT_GOAL, PLAN };
	PlanState state_{PlanState::WAIT_START};
	bool initStart_{false};
	bool initGoal_{false};
	visualization_msgs::msg::Marker startMarker_;
	visualization_msgs::msg::Marker goalMarker_;
	nav_msgs::msg::Path bsplineFitPath_;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clickedPointSub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr startVisPub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goalVisPub_;

	rclcpp::TimerBase::SharedPtr startVisTimer_;
	rclcpp::TimerBase::SharedPtr goalVisTimer_;
	rclcpp::TimerBase::SharedPtr mainTimer_;

	rclcpp::CallbackGroup::SharedPtr subCbGroup_;
	rclcpp::CallbackGroup::SharedPtr startVisCbGroup_;
	rclcpp::CallbackGroup::SharedPtr goalVisCbGroup_;
	rclcpp::CallbackGroup::SharedPtr mainCbGroup_;

	bool initialized_{false};
	std::shared_ptr<globalPlanner::rrtOctomap<3>> rrtPlanner_;
	std::shared_ptr<trajPlanner::polyTrajOctomap> polyPlanner_;
	std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;
};

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TestBsplineNode>();
	rclcpp::executors::MultiThreadedExecutor exec;
	exec.add_node(node);
	node->run();
	exec.spin();
	rclcpp::shutdown();
	return 0;
}