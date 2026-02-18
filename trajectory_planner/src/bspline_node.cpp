#include <rclcpp/rclcpp.hpp>
#include <map_manager/occupancyMap.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/bsplineTraj.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mutex>
#include <thread>

using std::cout;
using std::endl;

class BsplineNavigationNode : public rclcpp::Node {
public:
	BsplineNavigationNode()
	: rclcpp::Node("bspline_node") {
		if (!this->has_parameter("desired_velocity")){
			this->declare_parameter<double>("desired_velocity", 1.0);
		} else {
			RCLCPP_WARN(this->get_logger(), "Parameter '%s' already declared. Skipping declaration.", "desired_velocity");
		}
		if (!this->has_parameter("desired_acceleration")){
			this->declare_parameter<double>("desired_acceleration", 1.0);
		} else {
			RCLCPP_WARN(this->get_logger(), "Parameter '%s' already declared. Skipping declaration.", "desired_acceleration");
		}
		this->get_parameter("desired_velocity", desiredVel_);
		this->get_parameter("desired_acceleration", desiredAcc_);

		subCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		startVisCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		goalVisCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		trajVisCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		mainCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		rclcpp::SubscriptionOptions subOptions;
		subOptions.callback_group = subCbGroup_;
		clickedPointSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/move_base_simple/goal",
			1000,
			std::bind(&BsplineNavigationNode::clickedPointCB, this, std::placeholders::_1),
			subOptions);

		posePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/trajectory_pose", 1000);
		startVisPub_ = this->create_publisher<visualization_msgs::msg::Marker>("/start_position", 1000);
		goalVisPub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_position", 1000);
		originalBsplinePub_ = this->create_publisher<nav_msgs::msg::Path>("/original_trajectory", 1000);
		originalCptsPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/original_control_points", 1000);
		inputTrajPub_ = this->create_publisher<nav_msgs::msg::Path>("/input_traj", 1000);
		inputTrajPointPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/input_traj_point", 1000);
		originalCptsBeforePub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/control_points_before", 1000);

		startVisTimer_ = this->create_wall_timer(
			std::chrono::duration<double>(0.1),
			std::bind(&BsplineNavigationNode::publishStartVis, this),
			startVisCbGroup_);
		goalVisTimer_ = this->create_wall_timer(
			std::chrono::duration<double>(0.1),
			std::bind(&BsplineNavigationNode::publishGoalVis, this),
			goalVisCbGroup_);
		trajVisTimer_ = this->create_wall_timer(
			std::chrono::duration<double>(1.0 / 30.0),
			std::bind(&BsplineNavigationNode::publishTraj, this),
			trajVisCbGroup_);

		mainTimer_ = this->create_wall_timer(
			std::chrono::duration<double>(0.1),
			std::bind(&BsplineNavigationNode::mainTimerCB, this),
			mainCbGroup_);

		initialized_ = false;
	}

private:
	void initPlanners(){
		if (initialized_) return;
		auto self = shared_from_this();
		map_.reset(new mapManager::occMap(self));
		map_->initMap();
		polyTraj_.reset(new trajPlanner::polyTrajOccMap(self));
		polyTraj_->setMap(map_);
		polyTraj_->updateDesiredVel(desiredVel_);
		polyTraj_->updateDesiredAcc(desiredAcc_);

		bsplineTraj_.reset(new trajPlanner::bsplineTraj(self));
		bsplineTraj_->setMap(map_);
		bsplineTraj_->updateMaxVel(desiredVel_);
		bsplineTraj_->updateMaxAcc(desiredAcc_);
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

	void publishTrajectory(const Eigen::MatrixXd& controlPoints,
			const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &cptPublisher,
			const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &trajPublisher,
			const std::string &ns, double r, double g, double b, double ts){
		if (controlPoints.cols() == 0) return;
		visualization_msgs::msg::MarkerArray msg;
		std::vector<visualization_msgs::msg::Marker> pointVec;
		visualization_msgs::msg::Marker point;
		int pointCount = 0;
		for (int i=0; i<controlPoints.cols(); ++i){
			point.header.frame_id = "drone0/map";
			point.header.stamp = this->now();
			point.ns = ns;
			point.id = pointCount;
			point.type = visualization_msgs::msg::Marker::SPHERE;
			point.action = visualization_msgs::msg::Marker::ADD;
			point.pose.position.x = controlPoints(0, i);
			point.pose.position.y = controlPoints(1, i);
			point.pose.position.z = controlPoints(2, i);
			point.lifetime = rclcpp::Duration::from_seconds(0.05);
			point.scale.x = 0.2;
			point.scale.y = 0.2;
			point.scale.z = 0.2;
			point.color.a = 1.0;
			point.color.r = r;
			point.color.g = g;
			point.color.b = b;
			pointVec.push_back(point);
			++pointCount;
		}
		msg.markers = pointVec;
		cptPublisher->publish(msg);

		nav_msgs::msg::Path traj;
		Eigen::Vector3d p;
		geometry_msgs::msg::PoseStamped ps;
		trajPlanner::bspline bsplineTraj = trajPlanner::bspline (3, controlPoints, ts);
		for (double t=0; t<=bsplineTraj.getDuration(); t+=0.1){
			p = bsplineTraj.at(t);
			ps.pose.position.x = p(0);
			ps.pose.position.y = p(1);
			ps.pose.position.z = p(2);
			traj.poses.push_back(ps);
		}
		traj.header.frame_id = "drone0/map";
		trajPublisher->publish(traj);
	}

	void publishPathMsg(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &trajPub, nav_msgs::msg::Path& traj){
		traj.header.frame_id = "drone0/map";
		trajPub->publish(traj);
	}

	void publishInputPath(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &inputPathPub,
			const nav_msgs::msg::Path& input, const std::string &ns, double r, double g, double b){
		visualization_msgs::msg::MarkerArray msg;
		std::vector<visualization_msgs::msg::Marker> pointVec;
		visualization_msgs::msg::Marker point;
		int pointCount = 0;
		for (int i=0; i<int(input.poses.size()); ++i){
			point.header.frame_id = "drone0/map";
			point.header.stamp = this->now();
			point.ns = ns;
			point.id = pointCount;
			point.type = visualization_msgs::msg::Marker::SPHERE;
			point.action = visualization_msgs::msg::Marker::ADD;
			point.pose.position.x = input.poses[i].pose.position.x;
			point.pose.position.y = input.poses[i].pose.position.y;
			point.pose.position.z = input.poses[i].pose.position.z;
			point.lifetime = rclcpp::Duration::from_seconds(0.1);
			point.scale.x = 0.15;
			point.scale.y = 0.15;
			point.scale.z = 0.15;
			point.color.a = 0.7;
			point.color.r = r;
			point.color.g = g;
			point.color.b = b;
			pointVec.push_back(point);
			++pointCount;
		}
		msg.markers = pointVec;
		inputPathPub->publish(msg);
	}

	void publishControlPoints(const Eigen::MatrixXd& controlPoints,
			const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &cptPublisher,
			const std::string &ns, double r, double g, double b){
		visualization_msgs::msg::MarkerArray msg;
		std::vector<visualization_msgs::msg::Marker> pointVec;
		visualization_msgs::msg::Marker point;
		int pointCount = 0;
		for (int i=0; i<controlPoints.cols(); ++i){
			point.header.frame_id = "drone0/map";
			point.header.stamp = this->now();
			point.ns = ns;
			point.id = pointCount;
			point.type = visualization_msgs::msg::Marker::SPHERE;
			point.action = visualization_msgs::msg::Marker::ADD;
			point.pose.position.x = controlPoints(0, i);
			point.pose.position.y = controlPoints(1, i);
			point.pose.position.z = controlPoints(2, i);
			point.lifetime = rclcpp::Duration::from_seconds(0.05);
			point.scale.x = 0.2;
			point.scale.y = 0.2;
			point.scale.z = 0.2;
			point.color.a = 1.0;
			point.color.r = r;
			point.color.g = g;
			point.color.b = b;
			pointVec.push_back(point);
			++pointCount;
		}
		msg.markers = pointVec;
		cptPublisher->publish(msg);
	}

	void publishTraj(){
		publishPathMsg(inputTrajPub_, inputTraj_);
		publishTrajectory(originCpts_, originalCptsPub_, originalBsplinePub_, "origin", 0, 0, 1, controlPointTs_);
		publishInputPath(inputTrajPointPub_, inputTraj_, "origin", 0, 0, 1);
		publishControlPoints(controlPointsBefore_, originalCptsBeforePub_, "origin_cpts_before", 0, 0, 1);
	}

	void mainTimerCB(){
		if (!initialized_){
			initPlanners();
			return;
		}

		switch (state_){
			case PlanState::WAIT_START:
				if (consumeNewMsg()){
					if (firstTime_){
						start_ = newPoint_;
						firstTime_ = false;
					} else {
						start_ = goal_;
					}

					cout << "----------------------------------------------------" << endl;
					cout << "[Planner Node]: Request No. " << countLoop_ + 1 << endl;
					cout << "[Planner Node]: start point OK. (" << start_[0] << " " << start_[1] << " " << start_[2] << ")" << endl;

					initStart_ = true;
					initGoal_ = false;
					startMarker_.header.frame_id = "drone0/map";
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
					cout << "[Planner Node]: goal point OK. (" << goal_[0] << " " << goal_[1] << " " << goal_[2] << ")" << endl;

					initGoal_ = true;
					goalMarker_.header.frame_id = "drone0/map";
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
				firstTime_ = true;
				++countLoop_;
				cout << "----------------------------------------------------" << endl;
				state_ = PlanState::WAIT_START;
				break;
		}
	}

	void planOnce(){
		std::vector<Eigen::Vector3d> startEndConditions;
		Eigen::Vector3d currVel (0.0, 0.0, 0.0);
		Eigen::Vector3d currAcc (0.0, 0.0, 0.0);
		Eigen::Vector3d endVel (0.0, 0.0, 0.0);
		Eigen::Vector3d endAcc (0.0, 0.0, 0.0);

		startEndConditions.push_back(currVel);
		startEndConditions.push_back(endVel);
		startEndConditions.push_back(currAcc);
		startEndConditions.push_back(endAcc);

		nav_msgs::msg::Path waypointsMsg;
		geometry_msgs::msg::PoseStamped pStart, pGoal;
		pStart.pose.position.x = start_[0];
		pStart.pose.position.y = start_[1];
		pStart.pose.position.z = start_[2];
		pGoal.pose.position.x = goal_[0];
		pGoal.pose.position.y = goal_[1];
		pGoal.pose.position.z = goal_[2];
		waypointsMsg.poses = {pStart, pGoal};

		polyTraj_->updatePath(waypointsMsg, startEndConditions);
		polyTraj_->makePlan(false);

		nav_msgs::msg::Path adjustedInputPolyTraj;
		bool satisfyDistanceCheck = false;
		double initTs = bsplineTraj_->getInitTs();
		double dtTemp = initTs;
		double finalTimeTemp;
		rclcpp::Time startTime = this->now();
		while (true){
			rclcpp::Time currTime = this->now();
			if ((currTime - startTime).seconds() >= 0.05){
				cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
				break;
			}
			nav_msgs::msg::Path inputPolyTraj = polyTraj_->getTrajectory(dtTemp);
			satisfyDistanceCheck = bsplineTraj_->inputPathCheck(inputPolyTraj, adjustedInputPolyTraj, dtTemp, finalTimeTemp);
			if (satisfyDistanceCheck) break;
			dtTemp *= 0.8;
		}
		if (!satisfyDistanceCheck || adjustedInputPolyTraj.poses.empty()){
			cout << "[BsplineTraj]: Input path check failed. Skip planning." << endl;
			return;
		}
		inputTraj_ = adjustedInputPolyTraj;

		bool updateSuccess = bsplineTraj_->updatePath(inputTraj_, startEndConditions);
		if (!updateSuccess){
			cout << "[BsplineTraj]: Update path failed. Skip planning." << endl;
			return;
		}
		controlPointsBefore_ = bsplineTraj_->getControlPoints();
		if (updateSuccess){
			bool planSuccess = bsplineTraj_->makePlan();
			originCpts_ = bsplineTraj_->getControlPoints();
			cout << "Original Plan success: " << planSuccess << endl;
		}
		controlPointTs_ = bsplineTraj_->getControlPointTs();
	}

	std::mutex msgMutex_;
	bool firstTime_{true};
	bool newMsg_{false};
	std::vector<double> newPoint_{0, 0, 1.0};
	std::vector<double> start_{0, 0, 1.0};
	std::vector<double> goal_{0, 0, 1.0};
	int countLoop_{0};

	enum class PlanState { WAIT_START, WAIT_GOAL, PLAN };
	PlanState state_{PlanState::WAIT_START};

	bool initStart_{false};
	visualization_msgs::msg::Marker startMarker_;
	bool initGoal_{false};
	visualization_msgs::msg::Marker goalMarker_;
	Eigen::MatrixXd originCpts_;
	double controlPointTs_{0.0};
	nav_msgs::msg::Path inputTraj_;
	Eigen::MatrixXd controlPointsBefore_;

	double desiredVel_{1.0};
	double desiredAcc_{1.0};

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clickedPointSub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr startVisPub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goalVisPub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr originalBsplinePub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr originalCptsPub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr inputTrajPub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr inputTrajPointPub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr originalCptsBeforePub_;

	rclcpp::TimerBase::SharedPtr startVisTimer_;
	rclcpp::TimerBase::SharedPtr goalVisTimer_;
	rclcpp::TimerBase::SharedPtr trajVisTimer_;
	rclcpp::TimerBase::SharedPtr mainTimer_;

	rclcpp::CallbackGroup::SharedPtr subCbGroup_;
	rclcpp::CallbackGroup::SharedPtr startVisCbGroup_;
	rclcpp::CallbackGroup::SharedPtr goalVisCbGroup_;
	rclcpp::CallbackGroup::SharedPtr trajVisCbGroup_;
	rclcpp::CallbackGroup::SharedPtr mainCbGroup_;

	bool initialized_{false};
	std::shared_ptr<mapManager::occMap> map_;
	std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
	std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;
};

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<BsplineNavigationNode>();
	rclcpp::executors::MultiThreadedExecutor exec;
	exec.add_node(node);
	exec.spin();
	rclcpp::shutdown();
	return 0;
}