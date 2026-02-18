/*
	FILE: dynamicExploration.cpp
	-----------------------------
	Implementation of dynamic exploration
*/

#include <autonomous_flight/px4/dynamicExploration.h>
#include <limits>

namespace AutoFlight{
	dynamicExploration::dynamicExploration(const rclcpp::Node::SharedPtr& node) : flightBase(node){
		this->initParam();
		this->initModules();
		this->registerPub();
	}

	void dynamicExploration::initParam(){
		// desired velocity
		this->node_->declare_parameter<double>("desired_velocity", 0.5);
		this->node_->get_parameter("desired_velocity", this->desiredVel_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Desired velocity is set to: %.2fm/s.", this->desiredVel_);

		// desired acceleration
		this->node_->declare_parameter<double>("desired_acceleration", 2.0);
		this->node_->get_parameter("desired_acceleration", this->desiredAcc_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Desired acceleration is set to: %.2fm/s^2.", this->desiredAcc_);

		//  desired angular velocity
		this->node_->declare_parameter<double>("desired_angular_velocity", 0.5);
		this->node_->get_parameter("desired_angular_velocity", this->desiredAngularVel_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Angular velocity is set to: %.2frad/s.", this->desiredAngularVel_);

		//  desired angular velocity
		this->node_->declare_parameter<double>("waypoint_stablize_time", 1.0);
		this->node_->get_parameter("waypoint_stablize_time", this->wpStablizeTime_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Waypoint stablize time is set to: %.2fs.", this->wpStablizeTime_);

		//  initial scan
		this->node_->declare_parameter<bool>("initial_scan", false);
		this->node_->get_parameter("initial_scan", this->initialScan_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Initial scan is set to: %s", this->initialScan_ ? "true" : "false");

    	// replan time for dynamic obstacle
		this->node_->declare_parameter<double>("replan_time_for_dynamic_obstacles", 0.3);
		this->node_->get_parameter("replan_time_for_dynamic_obstacles", this->replanTimeForDynamicObstacle_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Dynamic obstacle replan time is set to: %.2fs.", this->replanTimeForDynamicObstacle_);

    	// free range 
    	std::vector<double> freeRangeTemp;
		this->node_->declare_parameter<std::vector<double>>("free_range", std::vector<double>{2.0, 2.0, 1.0});
		this->node_->get_parameter("free_range", freeRangeTemp);
		this->freeRange_(0) = freeRangeTemp[0];
		this->freeRange_(1) = freeRangeTemp[1];
		this->freeRange_(2) = freeRangeTemp[2];
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Free range is set to: %.2f %.2f %.2fm.", this->freeRange_(0), this->freeRange_(1), this->freeRange_(2));

    	// reach goal distance
		this->node_->declare_parameter<double>("reach_goal_distance", 0.1);
		this->node_->get_parameter("reach_goal_distance", this->reachGoalDistance_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Reach goal distance is set to: %.2fm.", this->reachGoalDistance_);

		this->node_->declare_parameter<bool>("require_operator_confirmation", false);
		this->node_->get_parameter("require_operator_confirmation", this->operatorConfirm_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Operator confirmation is set to: %s.", this->operatorConfirm_ ? "true" : "false");
	}

	void dynamicExploration::initModules(){
		// initialize map
		this->map_.reset(new mapManager::dynamicMap (this->node_));
		map_->initMap();
		// initialize exploration planner
		this->expPlanner_.reset(new globalPlanner::DEP (this->node_));
		this->expPlanner_->setMap(this->map_);
		this->expPlanner_->loadVelocity(this->desiredVel_, this->desiredAngularVel_);

		// initialize polynomial trajectory planner
		this->polyTraj_.reset(new trajPlanner::polyTrajOccMap (this->node_));
		this->polyTraj_->setMap(this->map_);
		this->polyTraj_->updateDesiredVel(this->desiredVel_);
		this->polyTraj_->updateDesiredAcc(this->desiredAcc_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->node_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->node_));
		this->bsplineTraj_->setMap(this->map_);
		this->bsplineTraj_->updateMaxVel(this->desiredVel_);
		this->bsplineTraj_->updateMaxAcc(this->desiredAcc_);		
	}

	void dynamicExploration::registerCallback(){
		// exploration replan callback
		this->exploreReplanCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->explorationTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(300), std::bind(&dynamicExploration::exploreReplan, this), this->exploreReplanCbGroup_);

		// planner callback
		this->plannerCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->replanCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->trajExeCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->visCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		this->plannerTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(20), std::bind(&dynamicExploration::plannerCB, this), this->plannerCbGroup_);

		// replan check timer
		this->replanCheckTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(10), std::bind(&dynamicExploration::replanCheckCB, this), this->replanCbGroup_);
		
		// trajectory execution callback
		this->trajExeTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(10), std::bind(&dynamicExploration::trajExeCB, this), this->trajExeCbGroup_);
	
		// visualization execution callabck
		this->visTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(33), std::bind(&dynamicExploration::visCB, this), this->visCbGroup_);
	}

	void dynamicExploration::registerPub(){
		this->polyTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("dynamicExploration/poly_traj", 1000);
		this->pwlTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("dynamicExploration/pwl_trajectory", 1000);
		this->bsplineTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("dynamicExploration/bspline_trajectory", 1000);
		this->inputTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("dynamicExploration/input_trajectory", 1000);
	}

	void dynamicExploration::explorationCB(){
		if (this->explorationReplan_){
			this->expPlanner_->setMap(this->map_);
			rclcpp::Time startTime = this->node_->now();
			bool replanSuccess = this->expPlanner_->makePlan();
			if (replanSuccess){
				this->waypoints_ = this->expPlanner_->getBestPath();
				this->newWaypoints_ = true;
				this->waypointIdx_ = 1;
				this->explorationReplan_ = false;
			}
			rclcpp::Time endTime = this->node_->now();
			cout << "[AutoFlight]: DEP planning time: " << (endTime - startTime).seconds() << "s." << endl;			
		}
	}

	void dynamicExploration::plannerCB(){
		// cout << "in planner callback" << endl;

		if (this->replan_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			nav_msgs::msg::Path inputTraj;
			std::vector<Eigen::Vector3d> startEndConditions;
			this->getStartEndConditions(startEndConditions); 
			// double initTs = this->bsplineTraj_->getInitTs();

			// generate new trajectory
			nav_msgs::msg::Path simplePath;
			geometry_msgs::msg::PoseStamped pStart, pGoal;
			pStart.pose = this->odom_.pose.pose;
			pGoal = this->goal_;
			simplePath.poses = {pStart, pGoal};
			this->pwlTraj_->updatePath(simplePath, false);
			this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
			// if (not this->trajectoryReady_){
			// 	// generate new trajectory
			// 	nav_msgs::Path simplePath;
			// 	geometry_msgs::PoseStamped pStart, pGoal;
			// 	pStart.pose = this->odom_.pose.pose;
			// 	pGoal = this->goal_;
			// 	simplePath.poses = {pStart, pGoal};
			// 	this->pwlTraj_->updatePath(simplePath, false);
			// 	this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
			// }
			// else{
			// 	Eigen::Vector3d bsplineLastPos = this->trajectory_.at(this->trajectory_.getDuration());
			// 	geometry_msgs::PoseStamped lastPs; lastPs.pose.position.x = bsplineLastPos(0); lastPs.pose.position.y = bsplineLastPos(1); lastPs.pose.position.z = bsplineLastPos(2);
			// 	Eigen::Vector3d goalPos (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
			// 	// if ((bsplineLastPos - goalPos).norm() >= 0.1){
			// 	nav_msgs::Path inputPWLTraj;
			// 	nav_msgs::Path simplePath;
			// 	geometry_msgs::PoseStamped pStart, pGoal;
			// 	pStart = lastPs;
			// 	pGoal = this->goal_;
			// 	simplePath.poses = {pStart, pGoal};
			// 	this->pwlTraj_->updatePath(simplePath, false);
			// 	this->pwlTraj_->makePlan(inputPWLTraj, this->bsplineTraj_->getControlPointDist());


			// 	nav_msgs::Path adjustedInputCombinedTraj;
			// 	bool satisfyDistanceCheck = false;
			// 	double dtTemp = initTs;
			// 	double finalTimeTemp;
			// 	ros::Time startTime = ros::Time::now();
			// 	ros::Time currTime;
			// 	while (ros::ok()){
			// 		currTime = ros::Time::now();
			// 		if ((currTime - startTime).toSec() >= 0.05){
			// 			cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
			// 			break;
			// 		}							
			// 		nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
			// 		nav_msgs::Path inputCombinedTraj;
			// 		inputCombinedTraj.poses = inputRestTraj.poses;
			// 		for (size_t i=1; i<inputPWLTraj.poses.size(); ++i){
			// 			inputCombinedTraj.poses.push_back(inputPWLTraj.poses[i]);
			// 		}
					
			// 		satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputCombinedTraj, adjustedInputCombinedTraj, dtTemp, finalTimeTemp);
			// 		if (satisfyDistanceCheck) break;
					
			// 		dtTemp *= 0.8; // magic number 0.8
			// 	}
			// 	inputTraj = adjustedInputCombinedTraj;
			// 	// }
			// 	// else{
			// 	// 	nav_msgs::Path adjustedInputRestTraj;
			// 	// 	bool satisfyDistanceCheck = false;
			// 	// 	double dtTemp = initTs;
			// 	// 	double finalTimeTemp;
			// 	// 	ros::Time startTime = ros::Time::now();
			// 	// 	ros::Time currTime;
			// 	// 	while (ros::ok()){
			// 	// 		currTime = ros::Time::now();
			// 	// 		if ((currTime - startTime).toSec() >= 0.05){
			// 	// 			cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
			// 	// 			break;
			// 	// 		}
			// 	// 		nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
			// 	// 		satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputRestTraj, adjustedInputRestTraj, dtTemp, finalTimeTemp);
			// 	// 		if (satisfyDistanceCheck) break;
						
			// 	// 		dtTemp *= 0.8;
			// 	// 	}
			// 	// 	inputTraj = adjustedInputRestTraj;					
			// 	// }

			// }

			

			this->inputTrajMsg_ = inputTraj;
			bool updateSuccess = this->bsplineTraj_->updatePath(inputTraj, startEndConditions);
			if (obstaclesPos.size() != 0 and updateSuccess){
				this->bsplineTraj_->updateDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			if (updateSuccess){
				nav_msgs::msg::Path bsplineTrajMsgTemp;
				bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
				if (planSuccess){
					this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
					this->trajStartTime_ = this->node_->now();
					this->trajTime_ = 0.0; // reset trajectory time
					this->trajectory_ = this->bsplineTraj_->getTrajectory();

					// optimize time
					// ros::Time timeOptStartTime = ros::Time::now();
					// this->timeOptimizer_->optimize(this->trajectory_, this->desiredVel_, this->desiredAcc_, 0.1);
					// ros::Time timeOptEndTime = ros::Time::now();
					// cout << "[AutoFlight]: Time optimizatoin spends: " << (timeOptEndTime - timeOptStartTime).toSec() << "s." << endl;

					this->trajectoryReady_ = true;
					this->replan_ = false;
					cout << "\033[1;32m[AutoFlight]: Trajectory generated successfully.\033[0m " << endl;
				}
				else{
					// if the current trajectory is still valid, then just ignore this iteration
					// if the current trajectory/or new goal point is assigned is not valid, then just stop
					if (this->hasCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails." << endl;
						this->replan_ = false;
					}
					else if (this->hasDynamicCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails. Replan for dynamic obstacles." << endl;
						this->replan_ = true;
					}
					else{
						if (this->trajectoryReady_){
							cout << "[AutoFlight]: Trajectory fail. Use trajectory from previous iteration." << endl;
							this->replan_ = false;
						}
						else{
							cout << "[AutoFlight]: Unable to generate a feasible trajectory." << endl;
							cout << "\033[1;32m[AutoFlight]: Wait for new path to replan.\033[0m" << endl;
							this->replan_ = false;
						}
					}
				}
			}
			else{
				this->trajectoryReady_ = false;
				this->stop();
				this->replan_ = false;
				cout << "[AutoFlight]: Goal is not valid. Stop." << endl;
			}


		}
	}

	void dynamicExploration::replanCheckCB(){
		/*
			Replan if
			1. collision detected
			2. new goal point assigned
			3. fixed distance
		*/

		if (this->waypointRotatePending_){
			if (this->newWaypoints_){
				// New global path supersedes pending rotate-to-waypoint transition.
				this->waypointRotatePending_ = false;
			}
			else if (this->node_->now() < this->waypointRotateReadyTime_){
				return;
			}
			else {
				cout << "[AutoFlight]: Rotate and replan..." << endl;
				this->moveToOrientation(this->waypointRotateYaw_, this->desiredAngularVel_);
				cout << "[AutoFlight]: Finish rotation." << endl;

				// change current goal
				if (this->waypointIdx_ < int(this->waypoints_.poses.size())){
					this->goal_ = this->waypoints_.poses[this->waypointIdx_];
				}
				if (this->waypointIdx_ + 1 > int(this->waypoints_.poses.size())){
					cout << "\033[1;32m[AutoFlight]: Finishing entire path. Wait for new path to replan.\033[0m" << endl;
					this->replan_ = false;
				}
				else{
					cout << "[AutoFlight]: Start planning for next waypoint." << endl;
					this->replan_ = true;
				}
				++this->waypointIdx_;
				this->trajectoryReady_ = false;
				this->waypointRotatePending_ = false;
				return;
			}
		}

		if (this->newWaypoints_){
			this->replan_ = false;
			this->trajectoryReady_ = false;
			double yaw = atan2(this->waypoints_.poses[1].pose.position.y - this->odom_.pose.pose.position.y, this->waypoints_.poses[1].pose.position.x - this->odom_.pose.pose.position.x);
			// cout << "[AutoFlight]: Go to next waypoint. Press ENTER to continue rotation." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();
			this->moveToOrientation(yaw, this->desiredAngularVel_);
			// cout << "[AutoFlight]: Press ENTER to move forward." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();		
			this->replan_ = true;
			this->newWaypoints_ = false;
			if (this->waypointIdx_ < int(this->waypoints_.poses.size())){
				this->goal_ = this->waypoints_.poses[this->waypointIdx_];
			}
			++this->waypointIdx_;
			cout << "[AutoFlight]: Replan for new waypoints." << endl; 

			return;
		}

		// if (this->isReach(this->goal_, 0.1, false) and this->waypointIdx_ <= int(this->waypoints_.poses.size())){
		// cout << "outside the if" << endl;
		// cout << "waypoints size: " << this->waypoints_.poses.size() << endl;
		// cout << "current waypoint idx: " << this->waypointIdx_ << endl;
		if (this->waypoints_.poses.size() != 0 and this->isReach(this->goal_, this->reachGoalDistance_, false) and this->waypointIdx_ <= int(this->waypoints_.poses.size())){
			// cout << "1" << endl;
			// when reach current goal point, reset replan and trajectory ready
			this->replan_ = false;
			this->trajectoryReady_ = false;
			// cout << "[AutoFlight]: Go to next waypoint. Press ENTER to continue rotation." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();
			cout << "[AutoFlight]: Stabilizing before rotate and replan..." << endl;
			geometry_msgs::msg::Quaternion quat = this->goal_.pose.orientation;
			double yaw = AutoFlight::rpy_from_quaternion(quat);
			this->waypointRotateYaw_ = yaw;
			this->waypointRotateReadyTime_ = this->node_->now() + rclcpp::Duration::from_seconds(std::max(0.0, this->wpStablizeTime_));
			this->waypointRotatePending_ = true;
			return;		
		}
		else if (this->waypoints_.poses.size() != 0 and this->isReach(this->goal_, this->reachGoalDistance_, true) and (this->replan_ or this->trajectoryReady_)){
			cout << "\033[[AutoFlight]: Finishing entire path. Wait for new path to replan.\033[0m" << endl;
			this->replan_ = false;
			this->trajectoryReady_ = false;
			return;		
		}

		if (this->waypoints_.poses.size() != 0){
			if (not this->isGoalValid() and (this->replan_ or this->trajectoryReady_)){
				this->replan_ = false;
				this->trajectoryReady_ = false;
				cout << "\033[1;32m[AutoFlight]: Current goal is invalid. Need new path to replan.\033[0m" << endl;
				// this->explorationReplan_ = true;
				return;
			}
		}

		// if (this->reachExplorationGoal()){
		// 	this->replan_ = false;
		// 	this->trajectoryReady_ = false;
		// 	geometry_msgs::Quaternion quat = this->waypoints_.poses.back().pose.orientation;
		// 	double yaw = AutoFlight::rpy_from_quaternion(quat);
		// 	cout << "[AutoFlight]: Reach exploration goal. Rotate and replan..." << endl;
		// 	this->moveToOrientation(yaw, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Finish rotation. Start to replan." << endl;
		// 	this->replan_ = true;
		// 	return;
		// }

		if (this->trajectoryReady_){
			if (not this->expPlanner_->isPosValid(this->trajectory_.at(this->trajectory_.getDuration()))){
				this->trajectoryReady_ = false;
				this->replan_ = false;
				this->stop();
				cout << "\033[1;32m[AutoFlight]: the goal of current local trajectory is not safe. Need replan.\033[0m" << endl;
				return;
			}

			if (this->hasCollision()){ // if trajectory not ready, do not replan
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for collision." << endl;
				return;
			}

			// replan for dynamic obstacles
			if (this->computeExecutionDistance() >= 0.3 and this->hasDynamicCollision()){
			// if (this->hasDynamicObstacle()){
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for dynamic obstacles." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 1.5 and AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) >= 3){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 0.3 and this->replanForDynamicObstacle()){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan for dynamic obstacles." << endl;
				return;
			}
		}	
	}

	void dynamicExploration::trajExeCB(){
		if (this->trajectoryReady_){
			rclcpp::Time currTime = this->node_->now();
			double realTime = (currTime - this->trajStartTime_).seconds();
			this->trajTime_ = this->bsplineTraj_->getLinearReparamTime(realTime);
			double linearReparamFactor = this->bsplineTraj_->getLinearFactor();
			Eigen::Vector3d pos = this->trajectory_.at(this->trajTime_);
			Eigen::Vector3d vel = this->trajectory_.getDerivative().at(this->trajTime_) * linearReparamFactor;
			Eigen::Vector3d acc = this->trajectory_.getDerivative().getDerivative().at(this->trajTime_) * pow(linearReparamFactor, 2);
			double endTime = this->trajectory_.getDuration()/linearReparamFactor;

			double leftTime = endTime - realTime; 
			tracking_controller::msg::Target target;
			if (leftTime <= 0.0){ // zero vel and zero acc if close to
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = 0.0;
				target.velocity.y = 0.0;
				target.velocity.z = 0.0;
				target.acceleration.x = 0.0;
				target.acceleration.y = 0.0;
				target.acceleration.z = 0.0;
				target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				this->updateTargetWithState(target);						
			}
			else{
				target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = vel(0);
				target.velocity.y = vel(1);
				target.velocity.z = vel(2);
				target.acceleration.x = acc(0);
				target.acceleration.y = acc(1);
				target.acceleration.z = acc(2);
				this->updateTargetWithState(target);						
			}
		}
	}

	void dynamicExploration::visCB(){
		if (this->polyTrajMsg_.poses.size() != 0){
			this->polyTrajPub_->publish(this->polyTrajMsg_);
		}
		if (this->pwlTrajMsg_.poses.size() != 0){
			this->pwlTrajPub_->publish(this->pwlTrajMsg_);
		}
		if (this->bsplineTrajMsg_.poses.size() != 0){
			this->bsplineTrajPub_->publish(this->bsplineTrajMsg_);
		}
		if (this->inputTrajMsg_.poses.size() != 0){
			this->inputTrajPub_->publish(this->inputTrajMsg_);
		}
	}

	void dynamicExploration::run(){
		if (this->operatorConfirm_){
			cout << "\033[1;32m[AutoFlight]: Please double check all parameters. Continuing in non-blocking mode (CTRL+C to abort).\033[0m" << endl;
			RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: require_operator_confirmation=true, but dynamic exploration startup proceeds non-blocking.");
		}
		else{
			cout << "\033[1;32m[AutoFlight]: Please double check all parameters. Continuing automatically (set require_operator_confirmation=true to pause).\033[0m" << endl;
		}
		this->takeoff();

		if (this->operatorConfirm_){
			cout << "\033[1;32m[AutoFlight]: Takeoff succeed. Continuing in non-blocking mode (CTRL+C to abort).\033[0m" << endl;
			RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: require_operator_confirmation=true, but post-takeoff flow proceeds non-blocking.");
		}
		else{
			cout << "\033[1;32m[AutoFlight]: Takeoff succeed. Continuing automatically.\033[0m" << endl;
		}

		// int temp1 = system("mkdir -p ~/rosbag_exploration_info &");
		// int temp2 = system("mv ~/rosbag_exploration_info/exploration_info ~/rosbag_exploration_info/previous &");
		// int temp3 = system("ros2 bag record -o ~/rosbag_exploration_info/exploration_info /camera/aligned_depth_to_color/image_raw_t /camera/color/image_raw_t /dynamic_map/inflated_voxel_map_t /onboard_detector/dynamic_bboxes /drone0/sensor_measurements/odom /dynamicExploration/bspline_trajectory /autonomous_flight/target_state /tracking_controller/target_pose /dep/best_paths /dep/roadmap /dep/candidate_paths /dep/best_paths /dep/frontier_regions /dynamic_map/occupancy_map_2D &");
		// if (temp1==-1 or temp2==-1 or temp3==-1){
		// 	cout << "[AutoFlight]: Recording fails." << endl;
		// }

		this->initExplore();

		if (this->operatorConfirm_){
			cout << "\033[1;32m[AutoFlight]: Start planning in non-blocking mode (CTRL+C to abort).\033[0m" << endl;
			RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: require_operator_confirmation=true, but planning start proceeds non-blocking.");
		}
		else{
			cout << "\033[1;32m[AutoFlight]: Start planning.\033[0m" << endl;
		}

		this->registerCallback();
	}

	void dynamicExploration::initExplore(){
		// set start region to be free
		// Eigen::Vector3d range (2.0, 2.0, 1.0);
		Eigen::Vector3d startPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d c1 = startPos - this->freeRange_;
		Eigen::Vector3d c2 = startPos + this->freeRange_;
		this->map_->freeRegion(c1, c2);
		cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << this->freeRange_.transpose() << endl;

		if (this->initialScan_){
			cout << "[AutoFlight]: Start initial scan..." << endl;
			this->moveToOrientation(-PI_const/2, this->desiredAngularVel_);
			cout << "\033[1;32m[AutoFlight]: Continue initial scan to next 90 degree (non-blocking).\033[0m" << endl;
			if (this->operatorConfirm_){
				RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: require_operator_confirmation=true, but initial scan proceeds non-blocking between steps.");
			}
						
			this->moveToOrientation(-PI_const, this->desiredAngularVel_);
			cout << "\033[1;32m[AutoFlight]: Continue initial scan to next 90 degree (non-blocking).\033[0m" << endl;
			if (this->operatorConfirm_){
				RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: require_operator_confirmation=true, but initial scan proceeds non-blocking between steps.");
			}

			this->moveToOrientation(PI_const/2, this->desiredAngularVel_);
			cout << "\033[1;32m[AutoFlight]: Continue initial scan to next 90 degree (non-blocking).\033[0m" << endl;
			if (this->operatorConfirm_){
				RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: require_operator_confirmation=true, but initial scan proceeds non-blocking between steps.");
			}
			
			this->moveToOrientation(0, this->desiredAngularVel_);
			cout << "[AutoFlight]: End initial scan." << endl; 
		}		
	}

	void dynamicExploration::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions){
		/*	
			1. start velocity
			2. start acceleration (set to zero)
			3. end velocity
			4. end acceleration (set to zero) 
		*/

		Eigen::Vector3d currVel = this->currVel_;
		Eigen::Vector3d currAcc = this->currAcc_;
		Eigen::Vector3d endVel (0.0, 0.0, 0.0);
		Eigen::Vector3d endAcc (0.0, 0.0, 0.0);

		// if (not this->trajectoryReady_){
		// 	double yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		// 	Eigen::Vector3d direction (cos(yaw), sin(yaw), 0.0);
		// 	currVel = this->desiredVel_ * direction;
		// 	currAcc = this->desiredAcc_ * direction;
		// }
		startEndConditions.push_back(currVel);
		startEndConditions.push_back(endVel);
		startEndConditions.push_back(currAcc);
		startEndConditions.push_back(endAcc);
	}

	bool dynamicExploration::hasCollision(){
		if (this->trajectoryReady_){
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=0.1){
				Eigen::Vector3d p = this->trajectory_.at(t);
				bool hasCollision = this->map_->isInflatedOccupied(p);
				if (hasCollision){
					return true;
				}
			}
		}
		return false;
	}

	bool dynamicExploration::hasDynamicCollision(){
		if (this->trajectoryReady_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);

			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=0.1){
				Eigen::Vector3d p = this->trajectory_.at(t);
				
				for (size_t i=0; i<obstaclesPos.size(); ++i){
					Eigen::Vector3d ob = obstaclesPos[i];
					Eigen::Vector3d size = obstaclesSize[i];
					Eigen::Vector3d lowerBound = ob - size/2;
					Eigen::Vector3d upperBound = ob + size/2;
					if (p(0) >= lowerBound(0) and p(0) <= upperBound(0) and
						p(1) >= lowerBound(1) and p(1) <= upperBound(1) and
						p(2) >= lowerBound(2) and p(2) <= upperBound(2)){
						return true;
					}					
				}
			}
		}
		return false;
	}

	void dynamicExploration::exploreReplan(){
		// set start region to be free
		// Eigen::Vector3d range (2.0, 2.0, 1.0);
		// Eigen::Vector3d startPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		// Eigen::Vector3d c1 = startPos - range;
		// Eigen::Vector3d c2 = startPos + range;
		// this->map_->freeRegion(c1, c2);
		// cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << range.transpose() << endl;

		// if (this->initialScan_){
		// 	cout << "[AutoFlight]: Start initial scan..." << endl;
		// 	this->moveToOrientation(-PI_const/2, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();
						
		// 	this->moveToOrientation(-PI_const, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();

		// 	this->moveToOrientation(PI_const/2, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();
			
		// 	this->moveToOrientation(0, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: End initial scan." << endl; 
		// }
		this->expPlanner_->setMap(this->map_);
		rclcpp::Time startTime = this->node_->now();
		bool replanSuccess = this->expPlanner_->makePlan();
		if (replanSuccess){
			this->waypoints_ = this->expPlanner_->getBestPath();
			this->newWaypoints_ = true;
			this->waypointIdx_ = 1;
		}
		rclcpp::Time endTime = this->node_->now();
		if (this->operatorConfirm_){
			RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: operator confirmation is enabled, but planner callback remains non-blocking.");
		}
		cout << "[AutoFlight]: DEP planning time: " << (endTime - startTime).seconds() << "s." << endl;
	}

	double dynamicExploration::computeExecutionDistance(){
		if (this->trajectoryReady_ and not this->replan_){
			Eigen::Vector3d prevP, currP;
			bool firstTime = true;
			double totalDistance = 0.0;
			double remainDistance = 0.0;
			for (double t=0.0; t<=this->trajectory_.getDuration(); t+=0.1){
				currP = this->trajectory_.at(t);
				if (firstTime){
					firstTime = false;
				}
				else{
					if (t <= this->trajTime_){
						totalDistance += (currP - prevP).norm();
					}
					else{
						remainDistance += (currP - prevP).norm();
					}
				}
				prevP = currP;
			}
			if (remainDistance <= 1.0){ // no replan when less than 1m
				return -1.0;
			}
			return totalDistance;
		}
		return -1.0;
	}

	bool dynamicExploration::replanForDynamicObstacle(){
		rclcpp::Time currTime = this->node_->now();
		std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
		this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);

		bool replan = false;
		bool hasDynamicObstacle = (obstaclesPos.size() != 0);
		if (hasDynamicObstacle){
			double timePassed = (currTime - this->lastDynamicObstacleTime_).seconds();
			if (timePassed >= this->replanTimeForDynamicObstacle_){
				replan = true;
				this->lastDynamicObstacleTime_ = currTime;
			}
		}
		return replan;
	}

	bool dynamicExploration::reachExplorationGoal(){
		if (this->waypoints_.poses.size() == 0) return false;
		double distThresh = 0.1;
		double yawDiffThresh = 0.1;
		Eigen::Vector3d currPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d targetPos (this->waypoints_.poses.back().pose.position.x, this->waypoints_.poses.back().pose.position.y, this->waypoints_.poses.back().pose.position.z);
		double yawTarget = AutoFlight::rpy_from_quaternion(this->waypoints_.poses.back().pose.orientation); 
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		if ((currPos - targetPos).norm() <= distThresh and std::abs(currYaw - yawTarget) >= yawDiffThresh){
			return true;
		}
		return false;
	}

	bool dynamicExploration::isGoalValid(){
		Eigen::Vector3d pGoal (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
		if (this->map_->isInflatedOccupied(pGoal)){
			return false;
		}
		else{
			return true;
		}
	}

	nav_msgs::msg::Path dynamicExploration::getCurrentTraj(double dt){
		nav_msgs::msg::Path currentTraj;
		currentTraj.header.frame_id = this->mapFrameId_;
		currentTraj.header.stamp = this->node_->now();
	
		if (this->trajectoryReady_){
			// include the current pose
			// geometry_msgs::PoseStamped psCurr;
			// psCurr.pose = this->odom_.pose.pose;
			// currentTraj.poses.push_back(psCurr);
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=dt){
				Eigen::Vector3d pos = this->trajectory_.at(t);
				geometry_msgs::msg::PoseStamped ps;
				ps.pose.position.x = pos(0);
				ps.pose.position.y = pos(1);
				ps.pose.position.z = pos(2);
				currentTraj.poses.push_back(ps);
			}		
		}
		return currentTraj;
	}

	nav_msgs::msg::Path dynamicExploration::getRestGlobalPath(){
		nav_msgs::msg::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::msg::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::msg::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, pDiff) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::msg::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	nav_msgs::msg::Path dynamicExploration::getRestGlobalPath(const Eigen::Vector3d& pos){
		nav_msgs::msg::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr = pos;
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::msg::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::msg::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, pDiff) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::msg::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	nav_msgs::msg::Path dynamicExploration::getRestGlobalPath(const Eigen::Vector3d& pos, double yaw){
		nav_msgs::msg::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr = pos;
		Eigen::Vector3d direction (cos(yaw), sin(yaw), 0);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::msg::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);

			geometry_msgs::msg::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, direction) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::msg::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

}