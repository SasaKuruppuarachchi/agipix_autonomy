/*
	FILE: navigation.cpp
	------------------------
	navigation implementation file in real flight
*/
#include <autonomous_flight/px4/navigation.h>

namespace AutoFlight{
	navigation::navigation(const rclcpp::Node::SharedPtr& node) : flightBase(node){
		this->initParam();
		this->initModules();
		this->registerPub();
	}

	void navigation::initParam(){
    	// parameters    
		// Use global Planner
		this->node_->declare_parameter<bool>("use_global_planner", false);
		this->node_->get_parameter("use_global_planner", this->useGlobalPlanner_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Use global planner is set to: %s.", this->useGlobalPlanner_ ? "true" : "false");

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

    	// trajectory data save path   	
		this->node_->declare_parameter<std::string>("trajectory_info_save_path", "No");
		this->node_->get_parameter("trajectory_info_save_path", this->trajSavePath_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Trajectory info save path is set to: %s.", this->trajSavePath_.c_str());

		// whether or not to use time optimizer
		this->node_->declare_parameter<bool>("use_time_optimizer", false);
		this->node_->get_parameter("use_time_optimizer", this->useTimeOptimizer_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Use time optimizer is set to: %s.", this->useTimeOptimizer_ ? "true" : "false");
	}

	void navigation::initModules(){
		// initialize map
		this->map_.reset(new mapManager::occMap (this->node_));
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

		// initialize the trajectory divider
		this->trajDivider_.reset(new timeOptimizer::trajDivider (this->node_));
		this->trajDivider_->setMap(this->map_);

		// initialize the time optimizer
		this->timeOptimizer_.reset(new timeOptimizer::bsplineTimeOptimizer (this->node_));
		this->timeOptimizer_->setMap(this->map_);
	}

	void navigation::registerPub(){
		this->rrtPathPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("navigation/rrt_path", 10);
		this->polyTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("navigation/poly_traj", 10);
		this->pwlTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("navigation/pwl_trajectory", 10);
		this->bsplineTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("navigation/bspline_trajectory", 10);
		this->inputTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("navigation/input_trajectory", 10);
		this->inputTrajPointsPub_ = this->node_->create_publisher<visualization_msgs::msg::MarkerArray>("navigation/input_trajetory_points", 10);
	}

	void navigation::registerCallback(){
		// planner callback
		this->plannerCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->replanCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->trajExeCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->visCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		this->plannerTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(100), std::bind(&navigation::plannerCB, this), this->plannerCbGroup_);
		
		// collision check callback
		this->replanCheckTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(10), std::bind(&navigation::replanCheckCB, this), this->replanCbGroup_);

		// trajectory execution callback
		this->trajExeTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(10), std::bind(&navigation::trajExeCB, this), this->trajExeCbGroup_);

		// visualization callback
		this->visTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(33), std::bind(&navigation::visCB, this), this->visCbGroup_);
	}

	void navigation::plannerCB(){
		if (not this->firstGoal_) return;

		if (this->replan_){
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
						// startEndConditions[3] = Eigen::Vector3d (0.0, 0.0, 0.0);	
						// startEndConditions[3] = this->polyTraj_->getAcc(finalTime);	

					}
					else{
						cout << "[AutoFlight]: Global planner fails. Check goal and map." << endl;
					}		
				}				
			}
			else{
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
					// this->bsplineTraj_->updateControlPointsTs(dtTemp);
					// cout << "time step to sample is: " << dtTemp << endl;
					// startEndConditions[3] = Eigen::Vector3d (0.0, 0.0, 0.0);
					// startEndConditions[3] = this->polyTraj_->getAcc(finalTime);
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
						// startEndConditions[3] = Eigen::Vector3d (0.0, 0.0, 0.0);
						// startEndConditions[3] = this->polyTraj_->getAcc(finalTime);
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
			

			this->inputTrajMsg_ = inputTraj;

			bool updateSuccess = this->bsplineTraj_->updatePath(inputTraj, startEndConditions);
			if (updateSuccess){
				nav_msgs::msg::Path bsplineTrajMsgTemp;
				bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
				if (planSuccess){
					this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
					this->trajStartTime_ = this->node_->now();
					this->trajTime_ = 0.0; // reset trajectory time
					this->trajectory_ = this->bsplineTraj_->getTrajectory();

					// optimize time
					if (this->useTimeOptimizer_){
						rclcpp::Time timeOptStartTime = this->node_->now();
						this->timeOptimizer_->optimize(this->trajectory_, this->desiredVel_, this->desiredAcc_, 0.1);
						rclcpp::Time timeOptEndTime = this->node_->now();
						cout << "[AutoFlight]: Time optimizatoin spends: " << (timeOptEndTime - timeOptStartTime).seconds() << "s." << endl;
					}
					this->trajectoryReady_ = true;
					this->replan_ = false;
					cout << "\033[1;32m[AutoFlight]: Trajectory generated successfully.\033[0m " << endl;

					if (this->trajSavePath_ != "No" and this->firstTimeSave_){
						this->bsplineTraj_->writeCurrentTrajInfo(this->trajSavePath_, 0.05);
						this->firstTimeSave_ = false;
					}
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

	void navigation::replanCheckCB(){
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

		// return;
		if (this->trajectoryReady_){
			if (this->hasCollision()){ // if trajectory not ready, do not replan
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for collision." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 1.5 and AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) >= 3){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan." << endl;
				return;
			}
		}
	}

	void navigation::trajExeCB(){
		if (this->trajectoryReady_){
			rclcpp::Time currTime = this->node_->now();
			double realTime = (currTime - this->trajStartTime_).seconds();
			Eigen::Vector3d pos, vel, acc;
			double endTime;
			if (this->useTimeOptimizer_){
				this->trajTime_ = this->timeOptimizer_->getStates(realTime, pos, vel, acc);
				endTime = this->timeOptimizer_->getDuration();
			}
			else{
				this->trajTime_ = this->bsplineTraj_->getLinearReparamTime(realTime);
				double linearReparamFactor = this->bsplineTraj_->getLinearFactor();
				pos = this->trajectory_.at(this->trajTime_);
				vel = this->trajectory_.getDerivative().at(this->trajTime_) * linearReparamFactor;
				acc = this->trajectory_.getDerivative().getDerivative().at(this->trajTime_) * pow(linearReparamFactor, 2);
				endTime = this->trajectory_.getDuration()/linearReparamFactor;
			}


			double leftTime = endTime - realTime; 
			// cout << "left time: " << leftTime << endl;
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


	void navigation::visCB(){
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

		this->publishInputTraj();
	}

	void navigation::run(){
		// take off the drone
		this->takeoff();

		// int temp1 = system("mkdir -p ~/rosbag_navigation_info &");
		// int temp2 = system("mv ~/rosbag_navigation_info/navigation_info ~/rosbag_navigation_info/previous &");
		// int temp3 = system("ros2 bag record -o ~/rosbag_navigation_info/navigation_info /camera/color/image_raw /occupancy_map/inflated_voxel_map /navigation/bspline_trajectory /drone0/sensor_measurements/odom /autonomous_flight/target_state /tracking_controller/vel_and_acc_info /tracking_controller/target_pose /tracking_controller/trajectory_history /trajDivider/braking_zone /trajDivider/kdtree_range &");
		// if (temp1==-1 or temp2==-1 or temp3==-1){
		// 	cout << "[AutoFlight]: Recording fails." << endl;
		// }

		// register timer callback
		this->registerCallback();
	}


	void navigation::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions){
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

	bool navigation::hasCollision(){
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

	double navigation::computeExecutionDistance(){
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

	nav_msgs::msg::Path navigation::getCurrentTraj(double dt){
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

	nav_msgs::msg::Path navigation::getRestGlobalPath(){
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


	void navigation::publishInputTraj(){
		// this function publishes the input path as trajectory and also sample points
		if (this->inputTrajMsg_.poses.size() != 0){
			// publish the input trajectory as a smooth path
			this->inputTrajPub_->publish(this->inputTrajMsg_);

			visualization_msgs::msg::MarkerArray msg;
			std::vector<visualization_msgs::msg::Marker> pointVec;
			visualization_msgs::msg::Marker point;
			int pointCount = 0;
			for (int i=0; i<int(this->inputTrajMsg_.poses.size()); ++i){
				point.header.frame_id = this->mapFrameId_;
				point.header.stamp = this->node_->now();
				point.ns = "input_traj_points";
				point.id = pointCount;
				point.type = visualization_msgs::msg::Marker::SPHERE;
				point.action = visualization_msgs::msg::Marker::ADD;
				point.pose.position.x = this->inputTrajMsg_.poses[i].pose.position.x;
				point.pose.position.y = this->inputTrajMsg_.poses[i].pose.position.y;
				point.pose.position.z = this->inputTrajMsg_.poses[i].pose.position.z;
				point.lifetime = rclcpp::Duration::from_seconds(0.05);
				point.scale.x = 0.2;
				point.scale.y = 0.2;
				point.scale.z = 0.2;
				point.color.a = 1.0;
				point.color.r = 0;
				point.color.g = 1;
				point.color.b = 0;
				pointVec.push_back(point);
				++pointCount;			
			}
			msg.markers = pointVec;	
			this->inputTrajPointsPub_->publish(msg);
		}
	}
}