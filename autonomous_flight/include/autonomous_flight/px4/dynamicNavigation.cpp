/*
	FILE: dynamicNavigation.cpp
	------------------------
	dynamic navigation implementation file in real world
*/
#include <autonomous_flight/px4/dynamicNavigation.h>

namespace AutoFlight{
	dynamicNavigation::dynamicNavigation(const rclcpp::Node::SharedPtr& node) : flightBase(node){
		this->initParam();
		this->initModules();
		this->registerPub();
	}

	void dynamicNavigation::initParam(){
    	// parameters    
    	// use global planner or not	
		this->node_->declare_parameter<bool>("use_global_planner", false);
		this->node_->get_parameter("use_global_planner", this->useGlobalPlanner_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Global planner use is set to: %s.", this->useGlobalPlanner_ ? "true" : "false");

		// No turning of yaw
		this->node_->declare_parameter<bool>("no_yaw_turning", false);
		this->node_->get_parameter("no_yaw_turning", this->noYawTurning_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Yaw turning use is set to: %s.", this->noYawTurning_ ? "true" : "false");

		// full state control (yaw)
		this->node_->declare_parameter<bool>("use_yaw_control", false);
		this->node_->get_parameter("use_yaw_control", this->useYawControl_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Yaw control use is set to: %s.", this->useYawControl_ ? "true" : "false");

    	// desired linear velocity    	
		this->node_->declare_parameter<double>("desired_velocity", 1.0);
		this->node_->get_parameter("desired_velocity", this->desiredVel_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Desired velocity is set to: %.2fm/s.", this->desiredVel_);

		// desired acceleration
		this->node_->declare_parameter<double>("desired_acceleration", 1.0);
		this->node_->get_parameter("desired_acceleration", this->desiredAcc_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Desired acceleration is set to: %.2fm/s^2.", this->desiredAcc_);


    	// desired angular velocity    	
		this->node_->declare_parameter<double>("desired_angular_velocity", 1.0);
		this->node_->get_parameter("desired_angular_velocity", this->desiredAngularVel_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Desired angular velocity is set to: %.2frad/s.", this->desiredAngularVel_);

    	// replan time for dynamic obstacle
		this->node_->declare_parameter<double>("replan_time_for_dynamic_obstacles", 0.3);
		this->node_->get_parameter("replan_time_for_dynamic_obstacles", this->replanTimeForDynamicObstacle_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Dynamic obstacle replan time is set to: %.2fs.", this->replanTimeForDynamicObstacle_);

		this->node_->declare_parameter<double>("collision_replan_cooldown_sec", 0.30);
		this->node_->get_parameter("collision_replan_cooldown_sec", this->collisionReplanCooldownSec_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Collision replan cooldown is set to: %.2fs.", this->collisionReplanCooldownSec_);

    	// trajectory data save path   	
		this->node_->declare_parameter<std::string>("trajectory_info_save_path", "No");
		this->node_->get_parameter("trajectory_info_save_path", this->trajSavePath_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Trajectory info save path is set to: %s.", this->trajSavePath_.c_str());
	}

	void dynamicNavigation::initModules(){
		// initialize map
		this->map_.reset(new mapManager::dynamicMap (this->node_));
		map_->initMap();
		// initialize rrt planner
		this->rrtPlanner_.reset(new globalPlanner::rrtOccMap<3> (this->node_));
		this->rrtPlanner_->setMap(this->map_);

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

	void dynamicNavigation::registerPub(){
		this->rrtPathPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("dynamicNavigation/rrt_path", 10);
		this->polyTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("dynamicNavigation/poly_traj", 10);
		this->pwlTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("dynamicNavigation/pwl_trajectory", 10);
		this->bsplineTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("dynamicNavigation/bspline_trajectory", 10);
		this->inputTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("dynamicNavigation/input_trajectory", 10);
	}

	void dynamicNavigation::registerCallback(){
		// planner callback
		this->plannerCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->replanCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->trajExeCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->visCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Callback groups created. creating timers");

		this->plannerTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(20), std::bind(&dynamicNavigation::plannerCB, this), this->plannerCbGroup_);

		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Planner timer created.");

		// collision check callback
		this->replanCheckTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(10), std::bind(&dynamicNavigation::replanCheckCB, this), this->replanCbGroup_);

		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Replan check timer created.");

		// trajectory execution callback
		this->trajExeTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(10), std::bind(&dynamicNavigation::trajExeCB, this), this->trajExeCbGroup_);

		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Trajectory execution timer created.");

		// visualization callback
		this->visTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(33), std::bind(&dynamicNavigation::visCB, this), this->visCbGroup_);
		
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Visualization timer created.");
	}

	void dynamicNavigation::plannerCB(){
		std::scoped_lock<std::mutex> lock(this->navStateMutex_);
		RCLCPP_INFO_ONCE(this->node_->get_logger(), "[AutoFlight]: plannerCB is running.");
		if (not this->firstGoal_) return;

		if (this->replan_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			// get start and end condition for trajectory generation (the end condition is the final zero condition)
			std::vector<Eigen::Vector3d> startEndConditions;
			this->getStartEndConditions(startEndConditions); 
			nav_msgs::msg::Path inputTraj;
			// bspline trajectory generation
			double finalTime; // final time for bspline trajectory
			double initTs = this->bsplineTraj_->getInitTs();
			const int maxInputCheckIters = 12;
			if (this->useGlobalPlanner_){
				if (this->needGlobalPlan_){
					this->rrtPlanner_->updateStart(this->odom_.pose.pose);
					this->rrtPlanner_->updateGoal(this->goal_.pose);
					nav_msgs::msg::Path rrtPathMsgTemp;
					this->rrtPlanner_->makePlan(rrtPathMsgTemp);
					if (rrtPathMsgTemp.poses.size() >= 2){
						this->rrtPathMsg_ = rrtPathMsgTemp;
						this->globalPlanReady_ = true;
					}
					this->needGlobalPlan_ = false;
					return;
				}
				else{
					if (this->globalPlanReady_){
						// get rest of global plan
						nav_msgs::msg::Path restPath = this->getRestGlobalPath();
						this->polyTraj_->updatePath(restPath, startEndConditions);
						this->polyTraj_->makePlan(this->polyTrajMsg_); // no corridor constraint		
						nav_msgs::msg::Path adjustedInputPolyTraj;
						bool satisfyDistanceCheck = false;
						double dtTemp = initTs;
						double finalTimeTemp;
						rclcpp::Time startTime = this->node_->now();
						rclcpp::Time currTime;
						for (int inputCheckIter = 0; inputCheckIter < maxInputCheckIters; ++inputCheckIter){
							currTime = this->node_->now();
							if ((currTime - startTime).seconds() >= 0.05){
								cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
								break;
							}
							nav_msgs::msg::Path inputPolyTraj = this->polyTraj_->getTrajectory(dtTemp);
							satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputPolyTraj, adjustedInputPolyTraj, dtTemp, finalTimeTemp);
							if (satisfyDistanceCheck) break;
							dtTemp *= 0.8;
						}

						inputTraj = adjustedInputPolyTraj;
						finalTime = finalTimeTemp;
						startEndConditions[1] = this->polyTraj_->getVel(finalTime);
						startEndConditions[3] = this->polyTraj_->getAcc(finalTime);	

					}
					else{
						cout << "[AutoFlight]: Global planner fails. Check goal and map." << endl;
					}		
				}				
			}
			else{
				if (obstaclesPos.size() == 0){ // use prev planned trajectory if there is no dynamic obstacle
					if (not this->trajectoryReady_){ // use polynomial trajectory as input
						nav_msgs::msg::Path waypoints, polyTrajTemp;
						geometry_msgs::msg::PoseStamped start, goal;
						start.pose = this->odom_.pose.pose; goal = this->goal_;
						waypoints.poses = std::vector<geometry_msgs::msg::PoseStamped> {start, goal};					
						
						this->polyTraj_->updatePath(waypoints, startEndConditions);
						this->polyTraj_->makePlan(false); // no corridor constraint
						
						nav_msgs::msg::Path adjustedInputPolyTraj;
						bool satisfyDistanceCheck = false;
						double dtTemp = initTs;
						double finalTimeTemp;
						rclcpp::Time startTime = this->node_->now();
						rclcpp::Time currTime;
						for (int inputCheckIter = 0; inputCheckIter < maxInputCheckIters; ++inputCheckIter){
							currTime = this->node_->now();
							if ((currTime - startTime).seconds() >= 0.05){
								cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
								break;
							}
							nav_msgs::msg::Path inputPolyTraj = this->polyTraj_->getTrajectory(dtTemp);
							satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputPolyTraj, adjustedInputPolyTraj, dtTemp, finalTimeTemp);
							if (satisfyDistanceCheck) break;
							
							dtTemp *= 0.8;
						}

						inputTraj = adjustedInputPolyTraj;
						finalTime = finalTimeTemp;
						startEndConditions[1] = this->polyTraj_->getVel(finalTime);
						startEndConditions[3] = this->polyTraj_->getAcc(finalTime);
					}
					else{
						Eigen::Vector3d bsplineLastPos = this->trajectory_.at(this->trajectory_.getDuration());
						geometry_msgs::msg::PoseStamped lastPs; lastPs.pose.position.x = bsplineLastPos(0); lastPs.pose.position.y = bsplineLastPos(1); lastPs.pose.position.z = bsplineLastPos(2);
						Eigen::Vector3d goalPos (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
						// check the distance between last point and the goal position
						if ((bsplineLastPos - goalPos).norm() >= 0.2){ // use polynomial trajectory to make the rest of the trajectory
							nav_msgs::msg::Path waypoints, polyTrajTemp;
							waypoints.poses = std::vector<geometry_msgs::msg::PoseStamped>{lastPs, this->goal_};
							std::vector<Eigen::Vector3d> polyStartEndConditions;
							Eigen::Vector3d polyStartVel = this->trajectory_.getDerivative().at(this->trajectory_.getDuration());
							Eigen::Vector3d polyEndVel (0.0, 0.0, 0.0);
							Eigen::Vector3d polyStartAcc = this->trajectory_.getDerivative().getDerivative().at(this->trajectory_.getDuration());
							Eigen::Vector3d polyEndAcc (0.0, 0.0, 0.0);
							polyStartEndConditions.push_back(polyStartVel);
							polyStartEndConditions.push_back(polyEndVel);
							polyStartEndConditions.push_back(polyStartAcc);
							polyStartEndConditions.push_back(polyEndAcc);
							this->polyTraj_->updatePath(waypoints, polyStartEndConditions);
							this->polyTraj_->makePlan(false); // no corridor constraint
							
							nav_msgs::msg::Path adjustedInputCombinedTraj;
							bool satisfyDistanceCheck = false;
							double dtTemp = initTs;
							double finalTimeTemp;
							rclcpp::Time startTime = this->node_->now();
							rclcpp::Time currTime;
							for (int inputCheckIter = 0; inputCheckIter < maxInputCheckIters; ++inputCheckIter){
								currTime = this->node_->now();
								if ((currTime - startTime).seconds() >= 0.05){
									cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
									break;
								}							
								nav_msgs::msg::Path inputRestTraj = this->getCurrentTraj(dtTemp);
								nav_msgs::msg::Path inputPolyTraj = this->polyTraj_->getTrajectory(dtTemp);
								nav_msgs::msg::Path inputCombinedTraj;
								inputCombinedTraj.poses = inputRestTraj.poses;
								for (size_t i=1; i<inputPolyTraj.poses.size(); ++i){
									inputCombinedTraj.poses.push_back(inputPolyTraj.poses[i]);
								}
								
								satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputCombinedTraj, adjustedInputCombinedTraj, dtTemp, finalTimeTemp);
								if (satisfyDistanceCheck) break;
								
								dtTemp *= 0.8; // magic number 0.8
							}
							inputTraj = adjustedInputCombinedTraj;
							finalTime = finalTimeTemp - this->trajectory_.getDuration(); // need to subtract prev time since it is combined trajectory
							startEndConditions[1] = this->polyTraj_->getVel(finalTime);
							startEndConditions[3] = this->polyTraj_->getAcc(finalTime);
						}
						else{
							nav_msgs::msg::Path adjustedInputRestTraj;
							bool satisfyDistanceCheck = false;
							double dtTemp = initTs;
							double finalTimeTemp;
							rclcpp::Time startTime = this->node_->now();
							rclcpp::Time currTime;
							for (int inputCheckIter = 0; inputCheckIter < maxInputCheckIters; ++inputCheckIter){
								currTime = this->node_->now();
								if ((currTime - startTime).seconds() >= 0.05){
									cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
									break;
								}
								nav_msgs::msg::Path inputRestTraj = this->getCurrentTraj(dtTemp);
								satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputRestTraj, adjustedInputRestTraj, dtTemp, finalTimeTemp);
								if (satisfyDistanceCheck) break;
								
								dtTemp *= 0.8;
							}
							inputTraj = adjustedInputRestTraj;
						}
					}
				}
				else{
					nav_msgs::msg::Path simplePath;
					geometry_msgs::msg::PoseStamped pStart, pGoal;
					pStart.pose = this->odom_.pose.pose;
					pGoal = this->goal_;
					std::vector<geometry_msgs::msg::PoseStamped> pathVec {pStart, pGoal};
					simplePath.poses = pathVec;				
					this->pwlTraj_->updatePath(simplePath, 1.0, false);
					this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
				}
			}
			

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
							cout << "[AutoFlight]: Unable to generate a feasible trajectory. Please provide a new goal." << endl;
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

	void dynamicNavigation::replanCheckCB(){
		std::scoped_lock<std::mutex> lock(this->navStateMutex_);
		RCLCPP_INFO_ONCE(this->node_->get_logger(), "[AutoFlight]: replanCheckCB is running.");
		/*
			Replan if
			1. collision detected
			2. new goal point assigned
			3. fixed distance
		*/
		if (this->goalReceived_){
			this->replan_ = false;
			this->trajectoryReady_ = false;
			if (not this->noYawTurning_ and not this->useYawControl_){
				double yaw = atan2(this->goal_.pose.position.y - this->odom_.pose.pose.position.y, this->goal_.pose.position.x - this->odom_.pose.pose.position.x);
				this->facingYaw_ = yaw;
				// Avoid blocking inside timer callback. Yaw is tracked during trajectory execution.
				RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Skip blocking pre-rotation in replan callback (non-blocking mode).");
			}
			this->firstTimeSave_ = true;
			this->replan_ = true;
			this->goalReceived_ = false;
			if (this->useGlobalPlanner_){
				cout << "[AutoFlight]: Start global planning." << endl;
				this->needGlobalPlan_ = true;
				this->globalPlanReady_ = false;
			}

			cout << "[AutoFlight]: Replan for new goal position." << endl; 
			return;
		}

		if (this->replan_){
			return;
		}

		if (this->trajectoryReady_){
			if (this->hasCollision()){ // if trajectory not ready, do not replan
				const double nowSec = this->node_->now().seconds();
				const bool cooldownElapsed =
					(this->lastCollisionReplanSec_ < 0.0) ||
					((nowSec - this->lastCollisionReplanSec_) >= this->collisionReplanCooldownSec_);
				if (cooldownElapsed){
					this->replan_ = true;
					this->lastCollisionReplanSec_ = nowSec;
					RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: Replan for collision.");
				}
				else{
					RCLCPP_WARN_THROTTLE(
						this->node_->get_logger(),
						*this->node_->get_clock(),
						1000,
						"[AutoFlight]: Collision detected but replan is rate-limited by cooldown.");
				}
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

	void dynamicNavigation::trajExeCB(){
		std::scoped_lock<std::mutex> lock(this->navStateMutex_);
		RCLCPP_INFO_ONCE(this->node_->get_logger(), "[AutoFlight]: trajExeCB is running.");
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
			autonomous_flight::msg::Target target;
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
				if (not this->useYawControl_){
					target.yaw = this->facingYaw_;
				}
				else if (this->noYawTurning_){
					target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				}
				else{
					target.yaw = atan2(vel(1), vel(0));
				}				
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

	void dynamicNavigation::visCB(){
		std::scoped_lock<std::mutex> lock(this->navStateMutex_);
		RCLCPP_INFO_ONCE(this->node_->get_logger(), "[AutoFlight]: visCB is running.");
		if (this->rrtPathMsg_.poses.size() != 0){
			this->rrtPathPub_->publish(this->rrtPathMsg_);
		}
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

	void dynamicNavigation::run(){
		// Executor-native path: takeoff/state transitions handled by mode executor.
		// This node only publishes mission targets for /autonomous_flight/target_state.
		this->registerCallback();
	}

	void dynamicNavigation::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions){
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

	bool dynamicNavigation::hasCollision(){
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

	bool dynamicNavigation::hasDynamicCollision(){
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

	double dynamicNavigation::computeExecutionDistance(){
		if (this->trajectoryReady_ and not this->replan_){
			Eigen::Vector3d prevP, currP;
			bool firstTime = true;
			double totalDistance = 0.0;
			for (double t=0.0; t<=this->trajTime_; t+=0.1){
				currP = this->trajectory_.at(t);
				if (firstTime){
					firstTime = false;
				}
				else{
					totalDistance += (currP - prevP).norm();
				}
				prevP = currP;
			}
			return totalDistance;
		}
		return -1.0;
	}

	bool dynamicNavigation::replanForDynamicObstacle(){
		rclcpp::Time currTime = this->node_->now();
		std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
		this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);

		bool replan = false;
		bool hasDynamicObstacle = (obstaclesPos.size() != 0);
		if (hasDynamicObstacle){
			double timePassed = (currTime - this->lastDynamicObstacleTime_).seconds();
			if (this->lastDynamicObstacle_ == false or timePassed >= this->replanTimeForDynamicObstacle_){
				replan = true;
				this->lastDynamicObstacleTime_ = currTime;
			}
			this->lastDynamicObstacle_ = true;
		}
		else{
			this->lastDynamicObstacle_ = false;
		}

		return replan;
	}

	nav_msgs::msg::Path dynamicNavigation::getCurrentTraj(double dt){
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


	nav_msgs::msg::Path dynamicNavigation::getRestGlobalPath(){
		nav_msgs::msg::Path currPath;

		int nextIdx = this->rrtPathMsg_.poses.size()-1;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->rrtPathMsg_.poses.size()-1; ++i){
			geometry_msgs::msg::PoseStamped ps = this->rrtPathMsg_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::msg::PoseStamped psNext = this->rrtPathMsg_.poses[i+1];
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
		for (size_t i=nextIdx; i<this->rrtPathMsg_.poses.size(); ++i){
			currPath.poses.push_back(this->rrtPathMsg_.poses[i]);
		}
		return currPath;		
	}

}