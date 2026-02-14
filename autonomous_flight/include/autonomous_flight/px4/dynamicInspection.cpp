/*
	FILE: dynamicInspection.cpp
	-----------------------------
	Implementation of dynamic inspection
*/

#include <autonomous_flight/px4/dynamicInspection.h>
#include <tracking_controller/msg/target.hpp>

namespace nav_msgs { using Path = nav_msgs::msg::Path; }
namespace geometry_msgs {
	using PoseStamped = geometry_msgs::msg::PoseStamped;
	using Quaternion = geometry_msgs::msg::Quaternion;
	using Point = geometry_msgs::msg::Point;
}
namespace visualization_msgs {
	using Marker = visualization_msgs::msg::Marker;
	using MarkerArray = visualization_msgs::msg::MarkerArray;
}

namespace AutoFlight{
	dynamicInspection::dynamicInspection(const rclcpp::Node::SharedPtr& node) : flightBase(node), node_(node){
		this->initParam();
		this->initModules();
		this->registerPub();
	}

	void dynamicInspection::initParam(){
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

		// inspection moving velocity
		this->node_->declare_parameter<double>("inspection_velocity", 0.5);
		this->node_->get_parameter("inspection_velocity", this->inspectionVel_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Inspection velocity is set to: %.2fm/s.", this->inspectionVel_);

		// minimum wall area
		this->node_->declare_parameter<double>("min_wall_area", 20.0);
		this->node_->get_parameter("min_wall_area", this->minWallArea_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Minimum wall area is set to: %.2fm^2.", this->minWallArea_);

		// safe distance to all walls
		this->node_->declare_parameter<double>("safe_distance_to_wall", 2.5);
		this->node_->get_parameter("safe_distance_to_wall", this->safeDistance_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Safe distance to wall is set to: %.2fm.", this->safeDistance_);

		// side safe distance to all walls
		this->node_->declare_parameter<double>("side_safe_distance", 1.5);
		this->node_->get_parameter("side_safe_distance", this->sideSafeDistance_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Side safe distance is set to: %.2fm.", this->sideSafeDistance_);

		// inspection height
		std::vector<double> inspectionHeightsTemp;
		this->node_->declare_parameter<std::vector<double>>("inspection_height", std::vector<double>{2.5});
		this->node_->get_parameter("inspection_height", inspectionHeightsTemp);
		this->inspectionHeight_ = inspectionHeightsTemp[0];
		for (double h : inspectionHeightsTemp){
			this->inspectionHeights_.push_back(h);
			RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Inspection height is set to: %.2fm.", h);
		}

		// ascend step
		this->node_->declare_parameter<double>("ascend_step", 3.0);
		this->node_->get_parameter("ascend_step", this->ascendStep_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Ascend step is set to: %.2fm.", this->ascendStep_);

		// descend step
		this->node_->declare_parameter<double>("descend_step", 2.0);
		this->node_->get_parameter("descend_step", this->descendStep_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Descend step is set to: %.2fm.", this->descendStep_);

		// sensor range
		this->node_->declare_parameter<double>("sensor_range", 2.0);
		this->node_->get_parameter("sensor_range", this->sensorRange_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Sensor range is set to: %.2fm.", this->sensorRange_);

		// horizontal sensor angle 
		this->node_->declare_parameter<double>("sensor_angle_horizontal", PI_const/2);
		this->node_->get_parameter("sensor_angle_horizontal", this->sensorAngleH_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Horizontal sensor angle is set to: %.2f degree.", this->sensorAngleH_);
		this->sensorAngleH_ *= PI_const/180.0;

		// vertical sensor angle 
		this->node_->declare_parameter<double>("sensor_angle_vertical", PI_const/3);
		this->node_->get_parameter("sensor_angle_vertical", this->sensorAngleV_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Vertical sensor angle is set to: %.2f degree.", this->sensorAngleV_);
		this->sensorAngleV_ *= PI_const/180.0;

		// sample number
		this->node_->declare_parameter<int>("explore_sample_number", 10);
		this->node_->get_parameter("explore_sample_number", this->exploreSampleNum_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Exploration sample number is set to: %d", this->exploreSampleNum_);

		// inspection goal given or not
		this->node_->declare_parameter<bool>("inspection_goal_given", false);
		this->node_->get_parameter("inspection_goal_given", this->inspectionGoalGiven_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: use inspection goal is set to: %s", this->inspectionGoalGiven_ ? "true" : "false");

		// inspection width given or not
		this->node_->declare_parameter<bool>("inspection_width_given", false);
		this->node_->get_parameter("inspection_width_given", this->inspectionWidthGiven_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: use inspection width is set to: %s", this->inspectionWidthGiven_ ? "true" : "false");

		if (this->inspectionGoalGiven_){
			// inspection location (last component is orientation in degree)
			std::vector<double> inspectionGoalsTemp;
			this->node_->declare_parameter<std::vector<double>>("inspection_goal", std::vector<double>{});
			if (not this->node_->get_parameter("inspection_goal", inspectionGoalsTemp)){
				this->inspectionGoalGiven_ = false;
				cout << "[AutoFlight]: Use default inspection mode." << endl;
			}
			else{
				this->inspectionGoal_(0) = inspectionGoalsTemp[0];
				this->inspectionGoal_(1) = inspectionGoalsTemp[1];
				this->inspectionGoal_(2) = inspectionGoalsTemp[2];
				this->inspectionOrientation_ = inspectionGoalsTemp[3];
				this->inspectionOrientation_ *= PI_const/180;
				for (size_t i=0; i<inspectionGoalsTemp.size(); i+=4){
					Eigen::Vector3d p (inspectionGoalsTemp[i+0], inspectionGoalsTemp[i+1], inspectionGoalsTemp[i+2]);
					this->inspectionGoals_.push_back(p);
					this->inspectionOrientations_.push_back(inspectionGoalsTemp[i+3] * PI_const/180);
					cout << "[AutoFlight]: Inspection goal is set to: [" << inspectionGoalsTemp[i+0]  << ", " << inspectionGoalsTemp[i+1] << ", " <<  inspectionGoalsTemp[i+2] << "]." << endl; 
					cout << "[AutoFlight]: Inspection angle is set to: " << inspectionGoalsTemp[i+3] << " degree." << endl;
				}
				
			}
		}

		if (this->inspectionWidthGiven_){
			// inspection width
			std::vector<double> inspectionWidthsTemp;
			this->node_->declare_parameter<std::vector<double>>("inspection_width", std::vector<double>{});
			if (not this->node_->get_parameter("inspection_width", inspectionWidthsTemp)){
				this->inspectionWidthGiven_ = false;
				cout << "[AutoFlight]: No inspection width parameter." << endl;
			}
			else{
				this->inspectionWidthGiven_ = true;
				this->inspectionWidth_ = inspectionWidthsTemp[0];
				for (double w : inspectionWidthsTemp){
					this->inspectionWidths_.push_back(w);
					cout << "[AutoFlight]: Inspection width is set to: " << w << "m." << endl;
				}
			}
		}

		// zigzag inspection path
		this->node_->declare_parameter<bool>("zigzag_inspection", true);
		this->node_->get_parameter("zigzag_inspection", this->zigzagInspection_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Zig zag inspection is set to: %s", this->zigzagInspection_ ? "true" : "false");

		// fringe inspection path
		this->node_->declare_parameter<bool>("fringe_inspection", true);
		this->node_->get_parameter("fringe_inspection", this->fringeInspection_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Fringe inspection is set to: %s", this->fringeInspection_ ? "true" : "false");

		// inspection order
		this->node_->declare_parameter<bool>("inspect_left_first", true);
		this->node_->get_parameter("inspect_left_first", this->leftFirst_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Inspection order is set is set to: %s", this->leftFirst_ ? "true" : "false");

		// confirm max angle
		this->node_->declare_parameter<double>("confirm_max_angle", PI_const*3.0/4.0);
		this->node_->get_parameter("confirm_max_angle", this->confirmMaxAngle_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Confirm max angle is set is set to: %.2f", this->confirmMaxAngle_);
		this->confirmMaxAngle_ *= PI_const/180.0;

		// inspection confirm
		this->node_->declare_parameter<bool>("inspection_confirm", true);
		this->node_->get_parameter("inspection_confirm", this->inspectionConfirm_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Inspection confirm is set is set to: %s", this->inspectionConfirm_ ? "true" : "false");

		// backward no turn option
		this->node_->declare_parameter<bool>("backward_no_turn", false);
		this->node_->get_parameter("backward_no_turn", this->backwardNoTurn_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Backward turn is set to: %s", this->backwardNoTurn_ ? "true" : "false");

    	// replan time for dynamic obstacle
		this->node_->declare_parameter<double>("replan_time_for_dynamic_obstacles", 0.3);
		this->node_->get_parameter("replan_time_for_dynamic_obstacles", this->replanTimeForDynamicObstacle_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Dynamic obstacle replan time is set to: %.2fs.", this->replanTimeForDynamicObstacle_);

	}

	void dynamicInspection::initModules(){
		// initialize map
		this->map_.reset(new mapManager::dynamicMap (this->node_));
		this->map_->initMap();
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

	void dynamicInspection::registerPub(){
		// goal publisher
		this->goalPub_ = this->node_->create_publisher<geometry_msgs::msg::PoseStamped>("inspection/current_goal", 10);

		// rrt path publisher
		this->rrtPathPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("inspection/rrt_path", 10);

		// polynomial trajectory publisher
		this->polyTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("inspection/poly_trajectory", 10);

		// piecewise linear trajectory publisher
		this->pwlTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("inspection/pwl_trajectory", 10);

		// bspline trajectory publisher
		this->bsplineTrajPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("inspection/bspline_trajectory", 10);

		// wall visualization publisher
		this->wallVisPub_ = this->node_->create_publisher<visualization_msgs::msg::MarkerArray>("inspection/wall", 10);
	}

	void dynamicInspection::registerCallback(){
		// planner callback
		this->plannerCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->trajExeCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->checkWallCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->collisionCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->replanCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->visCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		this->plannerTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(100), std::bind(&dynamicInspection::plannerCB, this), this->plannerCbGroup_);

		this->inspectTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(50), std::bind(&dynamicInspection::inspectTimerCB, this), this->plannerCbGroup_);

		// trajectory execution callback
		this->trajExeTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(10), std::bind(&dynamicInspection::trajExeCB, this), this->trajExeCbGroup_);

		// check wall callback
		if (not this->inspectionGoalGiven_){
			this->checkWallTimer_ = this->node_->create_wall_timer(
				std::chrono::milliseconds(100), std::bind(&dynamicInspection::checkWallCB, this), this->checkWallCbGroup_);
		}	

		// collision check callback
		this->collisionCheckTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(20), std::bind(&dynamicInspection::collisionCheckCB, this), this->collisionCbGroup_);

		// replan check callback
		this->replanTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(20), std::bind(&dynamicInspection::replanCB, this), this->replanCbGroup_);

		// visualization callback
		this->visTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(30), std::bind(&dynamicInspection::visCB, this), this->visCbGroup_);
	}

	void dynamicInspection::run(){
		cout << "\033[1;32m[AutoFlight]: Please double check all parameters. Then PRESS ENTER to continue or PRESS CTRL+C to stop.\033[0m" << endl;
		std::cin.clear();
		fflush(stdin);
		std::cin.get();
		this->takeoff();

		int temp1 = system("mkdir -p ~/rosbag_inspection_info &");
		int temp2 = system("mv ~/rosbag_inspection_info/inspection_info ~/rosbag_inspection_info/previous &");
		int temp3 = system("ros2 bag record -o ~/rosbag_inspection_info/inspection_info /inspection/rrt_path /inspection/poly_trajectory /inspection/pwl_trajectory /inspection/bspline_trajectory /dynamic_map/inflated_voxel_map_t /onboard_detector/dynamic_bboxes /mavros/local_position/pose --ros-args -r __node:=inspection_bag_info &");
		if (temp1==-1 or temp2==-1 or temp3==-1){
			cout << "[AutoFlight]: Recording fails." << endl;
		}


		cout << "\033[1;32m[AutoFlight]: Takeoff succeed. Then PRESS ENTER to continue or PRESS CTRL+C to land.\033[0m" << endl;
		std::cin.clear();
		fflush(stdin);
		std::cin.get();
		this->registerCallback();
	}

	void dynamicInspection::plannerCB(){
		if (this->flightState_ == FLIGHT_STATE::FORWARD){
			// navigate to the goal position
			if (this->replan_){
				std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
				std::vector<Eigen::Vector3d> startEndConditions;
				this->getStartEndConditions(startEndConditions); 

				nav_msgs::msg::Path inputTraj;
				// bspline trajectory generation
				double finalTime; // final time for bspline trajectory
				double initTs = this->bsplineTraj_->getInitTs();
				geometry_msgs::msg::PoseStamped pGoal = this->getForwardGoal();
				// cout << "current goal is: " << pGoal.pose.position.x << " " << pGoal.pose.position.y << " " << pGoal.pose.position.z << endl;
				if (obstaclesPos.size() == 0){
					if (not this->trajectoryReady_){ // use polynomial trajectory as input
						nav_msgs::msg::Path waypoints, polyTrajTemp;
						geometry_msgs::msg::PoseStamped start, goal;
						start.pose = this->odom_.pose.pose; goal = pGoal;
						waypoints.poses = std::vector<geometry_msgs::msg::PoseStamped> {start, goal};					

						this->polyTraj_->updatePath(waypoints, startEndConditions);
						this->polyTraj_->makePlan(false); // no corridor constraint
						
						nav_msgs::msg::Path adjustedInputPolyTraj;
						bool satisfyDistanceCheck = false;
						double dtTemp = initTs;
						double finalTimeTemp;
						rclcpp::Time startTime = this->node_->now();
						rclcpp::Time currTime;
						for (int iter = 0; iter < 50; ++iter){
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
						Eigen::Vector3d goalPos (pGoal.pose.position.x, pGoal.pose.position.y, pGoal.pose.position.z);
						// check the distance between last point and the goal position
						if ((bsplineLastPos - goalPos).norm() >= 0.2){ // use polynomial trajectory to make the rest of the trajectory
							nav_msgs::msg::Path waypoints, polyTrajTemp;
							waypoints.poses = std::vector<geometry_msgs::msg::PoseStamped>{lastPs, pGoal};
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
							for (int iter = 0; iter < 50; ++iter){
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
							for (int iter = 0; iter < 50; ++iter){
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
					geometry_msgs::msg::PoseStamped pStart;
					pStart.pose = this->odom_.pose.pose;
					std::vector<geometry_msgs::msg::PoseStamped> pathVec {pStart, pGoal};
					simplePath.poses = pathVec;				
					this->pwlTraj_->updatePath(simplePath, 1.0, false);
					this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());		
				}


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
			this->prevState_ = this->flightState_;


			// if reach the wall, change the state to INSPECT
			if (this->isReach(this->getForwardGoal(), 0.5, false)){
				if (this->isWallDetected() or this->inspectionGoalGiven_){
					this->stop();
					this->changeState(FLIGHT_STATE::INSPECT);
					cout << "[AutoFlight]: Switch from forward to inspection." << endl;
					cout << "[AutoFlight]: Start Inspection..." << endl;
					return;
				}
			}

			// if multiple times cannot navigate to the goal, change the state to EXPLORE
			if (this->countBsplineFailure_ > 3){
				if (!this->bsplineFailureWaitActive_){
					cout << "[AutoFlight]: Wait for some time..." << endl;
					this->bsplineFailureWaitUntil_ = this->node_->now() + rclcpp::Duration::from_seconds(0.5);
					this->bsplineFailureWaitActive_ = true;
				}
				if (this->node_->now() < this->bsplineFailureWaitUntil_){
					return;
				}
				this->bsplineFailureWaitActive_ = false;
			}
			else{
				this->bsplineFailureWaitActive_ = false;
			}

			if (this->countBsplineFailure_ > 3 and not this->isWallDetected()){
				this->changeState(FLIGHT_STATE::EXPLORE);
				cout << "[AutoFlight]: Switch from forward to explore." << endl;
				return;
			}
		}

		if (this->flightState_ == FLIGHT_STATE::EXPLORE){
			if (this->prevState_ == FLIGHT_STATE::FORWARD){
				// generate new local exploration goal
				Eigen::Vector3d pGoalExplore;
				bool bestViewPointSuccess = this->getBestViewPoint(pGoalExplore);
				if (bestViewPointSuccess){
					geometry_msgs::PoseStamped psGoalExplore = this->eigen2ps(pGoalExplore);
					this->rrtPlanner_->updateStart(this->odom_.pose.pose);
					this->rrtPlanner_->updateGoal(psGoalExplore.pose);
					this->rrtPlanner_->makePlan(this->rrtPathMsg_);	
					this->prevState_ = this->flightState_;
				}
				else{
					if (!this->lookAroundActive_){
						cout << "[AutoFlight]: Looking around to increase map range..." << endl;
						this->lookAroundActive_ = true;
						this->lookAroundStep_ = 0;
					}
					if (this->actionActive_){
						if (!this->isActionDone()){
							return;
						}
						this->actionActive_ = false;
						this->useYaw_ = false;
					}
					if (!this->actionActive_){
						if (this->lookAroundStep_ == 0){
							this->startYawAction(PI_const/2.0);
							this->lookAroundStep_++;
							return;
						}
						if (this->lookAroundStep_ == 1){
							this->startYawAction(-PI_const/2.0);
							this->lookAroundStep_++;
							return;
						}
						if (this->lookAroundStep_ == 2){
							this->startYawAction(0.0);
							this->lookAroundStep_++;
							return;
						}
						this->lookAroundActive_ = false;
					}
				}
				return;
			}

			if (this->prevState_ == FLIGHT_STATE::EXPLORE){
				if (this->replan_){
					nav_msgs::msg::Path inputTraj;
					// bspline trajectory generation
					double finalTime; // final time for bspline trajectory
					double initTs = this->bsplineTraj_->getInitTs();
					std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
					this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
					std::vector<Eigen::Vector3d> startEndConditions;
					this->getStartEndConditions(startEndConditions); 
					// get the latest global waypoint path
					nav_msgs::msg::Path latestGLobalPath = this->getRestGlobalPath();

					this->polyTraj_->updatePath(latestGLobalPath, startEndConditions);
					geometry_msgs::msg::Twist vel;
					double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
					Eigen::Vector3d startCondition (cos(currYaw), sin(currYaw), 0);
					this->polyTraj_->makePlan(this->polyTrajMsg_);

					nav_msgs::msg::Path adjustedInputPolyTraj;
					bool satisfyDistanceCheck = false;
					double dtTemp = initTs;
					double finalTimeTemp;
					rclcpp::Time startTime = this->node_->now();
					rclcpp::Time currTime;
					for (int iter = 0; iter < 50; ++iter){
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

			// change to forward if finish current exploration
			if (this->trajectoryReady_){
				Eigen::Vector3d goalEig = this->trajectory_.at(this->trajectory_.getDuration());
				geometry_msgs::PoseStamped goalPs = eigen2ps(goalEig);
				if (this->isReach(goalPs, 0.3, false)){
					this->prevState_ = this->flightState_;
					this->changeState(FLIGHT_STATE::FORWARD);
					cout << "[AutoFlight]: Switch from explore to forward." << endl;
					return;
				}
			}
			this->prevState_ = this->flightState_;
		}

		if (this->flightState_ == FLIGHT_STATE::INSPECT){
			if (!this->inspectionActive_){
				this->resetInspectionSequence();
				this->inspectionActive_ = true;
				this->inspectPhase_ = InspectPhase::CHECK_SURROUNDINGS;
				this->inspectionGoalIdx_ = 0;
				cout << "[AutoFlight]: Start Inspection..." << endl;
			}
			return;
		}

		if (this->flightState_ == FLIGHT_STATE::BACKWARD){
			// generate global waypoint path. set new goal to the origin position
			geometry_msgs::PoseStamped psBack = this->eigen2ps(Eigen::Vector3d (0, 0, this->takeoffHgt_));
			if (this->prevState_ == FLIGHT_STATE::INSPECT){
				// turn back
				if (not this->backwardNoTurn_){
					if (!this->backwardTurnPending_){
						this->backwardTurnPending_ = true;
						this->startYawAction(-PI_const);
						return;
					}
					if (this->actionActive_){
						if (!this->isActionDone()){
							return;
						}
						this->actionActive_ = false;
						this->useYaw_ = false;
					}
					this->backwardTurnPending_ = false;
				}
				cout << "[AutoFlight]: Start generating global plan..." << endl;
				this->rrtPlanner_->updateStart(this->odom_.pose.pose);
				this->rrtPlanner_->updateGoal(psBack.pose);
				this->rrtPlanner_->makePlan(this->rrtPathMsg_);
				cout << "[AutoFlight]: Global planning finished." << endl;
			}

			if (this->prevState_ == FLIGHT_STATE::BACKWARD){
				if (this->replan_){
					nav_msgs::msg::Path inputTraj;
					// bspline trajectory generation
					double finalTime; // final time for bspline trajectory
					double initTs = this->bsplineTraj_->getInitTs();
					std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
					this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
					std::vector<Eigen::Vector3d> startEndConditions;
					this->getStartEndConditions(startEndConditions); 
					// get the latest global waypoint path
					nav_msgs::msg::Path latestGLobalPath = this->getRestGlobalPath();

					this->polyTraj_->updatePath(latestGLobalPath, startEndConditions);
					geometry_msgs::msg::Twist vel;
					double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
					Eigen::Vector3d startCondition (cos(currYaw), sin(currYaw), 0);
					this->polyTraj_->makePlan(this->polyTrajMsg_);

					nav_msgs::msg::Path adjustedInputPolyTraj;
					bool satisfyDistanceCheck = false;
					double dtTemp = initTs;
					double finalTimeTemp;
					rclcpp::Time startTime = this->node_->now();
					rclcpp::Time currTime;
					for (int iter = 0; iter < 50; ++iter){
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
			this->prevState_ = this->flightState_;

			return;
		}
	}

	void dynamicInspection::trajExeCB(){
		if (this->flightState_ == FLIGHT_STATE::FORWARD or (this->flightState_ == FLIGHT_STATE::BACKWARD and this->prevState_ != FLIGHT_STATE::INSPECT) or (this->flightState_ == FLIGHT_STATE::EXPLORE and this->prevState_ == FLIGHT_STATE::EXPLORE)){
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
		else{
			if (not this->td_.init){
				return;
			}
			if (not this->useYaw_){
				this->updateTarget(this->td_.getPoseWithoutYaw(this->odom_.pose.pose));
			}
			else{
				this->updateTarget(this->td_.getPose(this->odom_.pose.pose));	
			}
		}
	}

	void dynamicInspection::resetInspectionSequence(){
		this->inspectPhase_ = InspectPhase::IDLE;
		this->checkState_ = CheckSurroundState::INIT;
		this->checkHeights_.clear();
		this->checkHeightIdx_ = 0;
		this->checkLeftSuccess_ = false;
		this->checkRightSuccess_ = false;
		this->zigzagPlanned_ = false;
		this->fringeStep_ = 0;
		this->fringePlanned_ = false;
		this->actionActive_ = false;
		this->actionTimeoutSec_ = 0.0;
	}

	void dynamicInspection::startAction(const nav_msgs::msg::Path& path, double duration, const geometry_msgs::msg::PoseStamped& goal, bool useYaw){
		this->td_.updateTrajectory(path, duration);
		this->actionGoal_ = goal;
		this->actionUseYaw_ = useYaw;
		this->useYaw_ = useYaw;
		this->actionStartTime_ = this->node_->now();
		this->actionActive_ = true;
		this->actionTimeoutSec_ = std::max(duration + 2.0, 2.0);
	}

	void dynamicInspection::startYawAction(double yaw){
		double yawTgt = yaw;
		double yawCurr = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		double yawDiff = yawTgt - yawCurr;
		double direction = 0.0;
		double yawDiffAbs = std::abs(yawDiff);
		if ((yawDiffAbs <= PI_const) && (yawDiff > 0)){
			direction = 1.0; // counter clockwise
		}
		else if ((yawDiffAbs <= PI_const) && (yawDiff < 0)){
			direction = -1.0; // clockwise
		}
		else if ((yawDiffAbs > PI_const) && (yawDiff > 0)){
			direction = -1.0; // rotate in clockwise direction
			yawDiffAbs = 2 * PI_const - yawDiffAbs;
		}
		else if ((yawDiffAbs > PI_const) && (yawDiff < 0)){
			direction = 1.0; // counter clockwise
			yawDiffAbs = 2 * PI_const - yawDiffAbs;
		}

		double endTime = 0.0;
		if (this->desiredAngularVel_ > 1e-6){
			endTime = yawDiffAbs / this->desiredAngularVel_;
		}
		if (endTime <= 0.0){
			endTime = 0.1;
		}

		nav_msgs::msg::Path rotationPath;
		std::vector<geometry_msgs::msg::PoseStamped> rotationPathVec;
		geometry_msgs::msg::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		for (double t = 0.0; t <= endTime; t += 0.1){
			double currYawTgt = yawCurr + direction * (t / endTime) * yawDiffAbs;
			geometry_msgs::msg::Quaternion quatT = trajPlanner::quaternion_from_rpy(0, 0, currYawTgt);
			geometry_msgs::msg::PoseStamped psT = ps;
			psT.pose.orientation = quatT;
			rotationPathVec.push_back(psT);
		}
		rotationPath.poses = rotationPathVec;
		geometry_msgs::msg::PoseStamped goal;
		goal.pose = this->odom_.pose.pose;
		goal.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, yawTgt);
		this->startAction(rotationPath, endTime, goal, true);
	}

	bool dynamicInspection::isActionDone(){
		if (!this->actionActive_){
			return true;
		}
		const bool reach = this->isReach(this->actionGoal_, false);
		const bool timeDone = this->td_.getRemainTime() <= 0.0;
		const double elapsed = (this->node_->now() - this->actionStartTime_).seconds();
		return (reach && timeDone) || (elapsed >= this->actionTimeoutSec_);
	}

	bool dynamicInspection::checkSurroundingsStep(){
		if (this->checkState_ == CheckSurroundState::INIT){
			const double currHeight = this->odom_.pose.pose.position.z;
			double heightTemp = currHeight;
			while (heightTemp < this->inspectionHeight_){
				this->checkHeights_.push_back(heightTemp);
				heightTemp += this->ascendStep_;
			}
			this->checkHeights_.push_back(this->inspectionHeight_);
			this->checkHeightIdx_ = 0;
			this->checkState_ = CheckSurroundState::LEFT_CHECK;
			this->checkLeftSuccess_ = false;
			this->checkRightSuccess_ = false;
		}

		if (this->checkHeightIdx_ >= this->checkHeights_.size()){
			return true;
		}

		const double height = this->checkHeights_[this->checkHeightIdx_];
		Eigen::Vector3d pHeight(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, height);
		const double maxRayLength = 7.0;

		if (this->checkState_ == CheckSurroundState::LEFT_CHECK){
			if (!this->checkLeftSuccess_){
				this->checkLeftSuccess_ = this->map_->castRay(pHeight, Eigen::Vector3d(0, 1, 0), this->checkLeftEnd_, maxRayLength);
				if (!this->checkLeftSuccess_){
					geometry_msgs::msg::PoseStamped pStart, pGoal;
					pStart.pose = this->odom_.pose.pose;
					pGoal = pStart;
					pGoal.pose.position.y += 1.0;
					std::vector<geometry_msgs::msg::PoseStamped> pathVec{pStart, pGoal};
					double duration = this->makePWLTraj(pathVec, this->pwlTrajMsg_);
					this->startAction(this->pwlTrajMsg_, duration, pGoal, true);
					return false;
				}
			}
			this->checkState_ = CheckSurroundState::RIGHT_CHECK;
		}

		if (this->checkState_ == CheckSurroundState::RIGHT_CHECK){
			if (!this->checkRightSuccess_){
				this->checkRightSuccess_ = this->map_->castRay(pHeight, Eigen::Vector3d(0, -1, 0), this->checkRightEnd_, maxRayLength);
				if (!this->checkRightSuccess_){
					geometry_msgs::msg::PoseStamped pStart, pGoal;
					pStart.pose = this->odom_.pose.pose;
					pGoal = pStart;
					pGoal.pose.position.y -= 1.0;
					std::vector<geometry_msgs::msg::PoseStamped> pathVec{pStart, pGoal};
					double duration = this->makePWLTraj(pathVec, this->pwlTrajMsg_);
					this->startAction(this->pwlTrajMsg_, duration, pGoal, true);
					return false;
				}
			}
			this->checkState_ = CheckSurroundState::CENTER_MOVE;
		}

		if (this->checkState_ == CheckSurroundState::CENTER_MOVE){
			Eigen::Vector3d pCenter = (this->checkRightEnd_ + this->checkLeftEnd_) / 2.0;
			geometry_msgs::msg::PoseStamped pGoal = this->eigen2ps(pCenter);
			geometry_msgs::msg::PoseStamped pStart;
			pStart.pose = this->odom_.pose.pose;
			std::vector<geometry_msgs::msg::PoseStamped> pathVec{pStart, pGoal};
			double duration = this->makePWLTraj(pathVec, this->inspectionVel_, this->pwlTrajMsg_);
			this->startAction(this->pwlTrajMsg_, duration, pGoal, false);
			this->checkState_ = CheckSurroundState::NEXT_HEIGHT;
			return false;
		}

		if (this->checkState_ == CheckSurroundState::NEXT_HEIGHT){
			if (this->checkHeightIdx_ + 1 < this->checkHeights_.size()){
				Eigen::Vector3d pNextHeight(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->checkHeights_[this->checkHeightIdx_ + 1]);
				geometry_msgs::msg::PoseStamped pGoal = this->eigen2ps(pNextHeight);
				geometry_msgs::msg::PoseStamped pStart;
				pStart.pose = this->odom_.pose.pose;
				std::vector<geometry_msgs::msg::PoseStamped> pathVec{pStart, pGoal};
				double duration = this->makePWLTraj(pathVec, this->inspectionVel_, this->pwlTrajMsg_);
				this->startAction(this->pwlTrajMsg_, duration, pGoal, false);
			}
			this->checkHeightIdx_++;
			this->checkLeftSuccess_ = false;
			this->checkRightSuccess_ = false;
			this->checkState_ = CheckSurroundState::LEFT_CHECK;
			return false;
		}

		return false;
	}

	void dynamicInspection::inspectTimerCB(){
		if (!this->inspectionActive_ || this->flightState_ != FLIGHT_STATE::INSPECT){
			return;
		}

		if (this->actionActive_){
			if (!this->isActionDone()){
				return;
			}
			this->actionActive_ = false;
			this->useYaw_ = false;
		}

		switch (this->inspectPhase_){
			case InspectPhase::CHECK_SURROUNDINGS: {
				if (this->inspectionGoalGiven_ && this->inspectionGoalIdx_ < this->inspectionGoals_.size()){
					this->inspectionGoal_ = this->inspectionGoals_[this->inspectionGoalIdx_];
					this->inspectionOrientation_ = this->inspectionOrientations_[this->inspectionGoalIdx_];
					this->inspectionHeight_ = this->inspectionHeights_[this->inspectionGoalIdx_];
					this->inspectionWidth_ = this->inspectionWidths_[this->inspectionGoalIdx_];
					this->inspectPhase_ = InspectPhase::MOVE_TO_GOAL;
					break;
				}

				if (!this->inspectionGoalGiven_ && !this->inspectionWidthGiven_){
					if (this->checkSurroundingsStep()){
						this->inspectPhase_ = InspectPhase::ZIGZAG;
					}
					break;
				}

				// for width-given: move to center line first
				if (this->inspectionWidthGiven_ && !this->inspectionGoalGiven_){
					Eigen::Vector3d centerPos(this->odom_.pose.pose.position.x, 0.0, this->takeoffHgt_);
					geometry_msgs::msg::PoseStamped pGoal = this->eigen2ps(centerPos);
					std::vector<geometry_msgs::msg::PoseStamped> pathVec{this->eigen2ps(Eigen::Vector3d(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z)), pGoal};
					double duration = this->makePWLTraj(pathVec, this->inspectionVel_, this->pwlTrajMsg_);
					this->startAction(this->pwlTrajMsg_, duration, pGoal, false);
					this->inspectPhase_ = InspectPhase::ZIGZAG;
					break;
				}
				this->inspectPhase_ = InspectPhase::ZIGZAG;
				break;
			}
			case InspectPhase::MOVE_TO_GOAL: {
				geometry_msgs::msg::PoseStamped pGoal = this->eigen2ps(this->inspectionGoal_);
				std::vector<geometry_msgs::msg::PoseStamped> pathVec{this->eigen2ps(Eigen::Vector3d(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z)), pGoal};
				double duration = this->makePWLTraj(pathVec, this->inspectionVel_, this->pwlTrajMsg_);
				this->startAction(this->pwlTrajMsg_, duration, pGoal, false);
				this->inspectPhase_ = InspectPhase::ORIENT_GOAL;
				break;
			}
			case InspectPhase::ORIENT_GOAL: {
				geometry_msgs::msg::Quaternion quat = AutoFlight::quaternion_from_rpy(0, 0, this->inspectionOrientation_);
				geometry_msgs::msg::PoseStamped pGoal;
				pGoal.pose = this->odom_.pose.pose;
				pGoal.pose.orientation = quat;
				geometry_msgs::msg::PoseStamped pStart;
				pStart.pose = this->odom_.pose.pose;
				std::vector<geometry_msgs::msg::PoseStamped> pathVec{pStart, pGoal};
				double duration = this->makePWLTraj(pathVec, this->pwlTrajMsg_);
				this->startAction(this->pwlTrajMsg_, duration, pGoal, true);
				this->inspectPhase_ = InspectPhase::ZIGZAG;
				break;
			}
			case InspectPhase::ZIGZAG: {
				if (!this->zigzagPlanned_){
					if (this->inspectionWidthGiven_ || this->inspectionGoalGiven_){
						this->inspectZigZagRange();
					}
					else{
						this->inspectZigZag();
					}
					this->zigzagPlanned_ = true;
					break;
				}
				if (!this->actionActive_){
					this->inspectPhase_ = this->fringeInspection_ ? InspectPhase::FRINGE : InspectPhase::COMPLETE;
				}
				break;
			}
			case InspectPhase::FRINGE: {
				if (!this->fringePlanned_){
					if (this->inspectionWidthGiven_ || this->inspectionGoalGiven_){
						this->inspectFringeRange();
					}
					else{
						this->inspectFringe();
					}
					this->fringePlanned_ = true;
					this->fringeStep_ = 0;
				}
				if (this->fringeStep_ == 0 && !this->actionActive_){
					this->startAction(this->fringePath1_, this->fringeDuration1_, this->fringeGoal1_, false);
					this->fringeStep_ = 1;
					break;
				}
				if (this->fringeStep_ == 1 && !this->actionActive_){
					this->startAction(this->fringePath2_, this->fringeDuration2_, this->fringeGoal2_, false);
					this->fringeStep_ = 2;
					break;
				}
				if (this->fringeStep_ >= 2 && !this->actionActive_){
					this->inspectPhase_ = InspectPhase::COMPLETE;
				}
				break;
			}
			case InspectPhase::COMPLETE: {
				this->inspectionGoalIdx_++;
				if (this->inspectionGoalGiven_ && this->inspectionGoalIdx_ < this->inspectionGoals_.size()){
					this->zigzagPlanned_ = false;
					this->fringeStep_ = 0;
					this->inspectPhase_ = InspectPhase::MOVE_TO_GOAL;
					break;
				}

				this->inspectionActive_ = false;
				this->prevState_ = this->flightState_;
				this->changeState(FLIGHT_STATE::BACKWARD);
				cout << "[AutoFlight]: Switch from inspection to backward." << endl;
				break;
			}
			default:
				break;
		}
	}

	void dynamicInspection::checkWallCB(){
		// use current robot position, check the occcupancy of the predefined wall size bounding box

		// const double maxWallWidth = 10.0; // The maximum width of the wall
		// const double maxWallHeight = 10.0; // the maximum height of the wall
		const double maxThickness = 3.0; // The maximum thickness of the wall

		// 1. find the occupied point in front of the robot
		Eigen::Vector3d currPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d direction (1, 0, 0);
		Eigen::Vector3d endPos;
		double maxRayLength = 5.0;
		bool castRaySuccess = this->map_->castRay(currPos, direction, endPos, maxRayLength);
		
		if (castRaySuccess){
			// search for the wall region
			double minX = endPos(0) - this->map_->getRes();
			double maxX = endPos(0) + maxThickness;
			double minY = endPos(1);
			double maxY = endPos(1);
			double minZ = endPos(2);
			double maxZ = endPos(2);			

			const double searchResolution = 0.3;
			for (double xs=endPos(0); xs<endPos(0)+maxThickness; xs+=searchResolution){
				Eigen::Vector3d p (xs, endPos(1), endPos(2));
				Eigen::Vector3d pYP, pYN, pZP, pZN;
				const double maxRayLengthWall = 7.0;
				
				// cast ray in +y direction
				bool ypSuccess = this->castRayOccupied(p, Eigen::Vector3d (0, 1, 0), pYP, maxRayLengthWall);

				// cast ray in -y direction
				bool ynSuccess = this->castRayOccupied(p, Eigen::Vector3d (0, -1, 0), pYN, maxRayLengthWall);

				// cast ray in +z direction
				bool zpSuccess = this->castRayOccupied(p, Eigen::Vector3d (0, 0, 1), pZP, maxRayLengthWall);

				// cast ray in -z direction
				bool znSuccess = this->castRayOccupied(p, Eigen::Vector3d (0, 0, -1), pZN, maxRayLengthWall);

				// update range
				if (pYP(1) > maxY){
					maxY = pYP(1);
				}

				if (pYN(1) < minY){
					minY = pYN(1);
				}

				if (pZP(2) > maxZ){
					maxZ = pZP(2);
				}

				if (not znSuccess){
					minZ = 0.0;
				}
				else{
					if (pZN(2) < minZ){
						minZ = pZN(2);
					}
				}

				bool noWall = (not ypSuccess) and (not ynSuccess) and (not zpSuccess) and (not znSuccess);
				if (noWall){
					maxX = xs;
					break;
				}
			}

			std::vector<double> wallRange {minX, maxX, minY, maxY, minZ, maxZ};
			this->updateWallRange(wallRange);
		}
		else{
			std::vector<double> wallRange {0, 0, 0, 0, 0, 0};	
			this->updateWallRange(wallRange);
		}
	}

	void dynamicInspection::collisionCheckCB(){
		if (this->td_.currTrajectory.poses.size() == 0) return;
		nav_msgs::msg::Path currTrajectory = this->td_.currTrajectory;

		// we only check for static obstacle collision
		for (geometry_msgs::PoseStamped ps : currTrajectory.poses){
			Eigen::Vector3d p (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			if (this->map_->isInflatedOccupied(p)){
				this->trajValid_ = false;
				return;
			}
		}
		this->trajValid_ = true;
	}

	void dynamicInspection::replanCB(){
		/*
			Replan if
			1. collision detected
			2. new goal point assigned
			3. fixed distance
		*/

		if (this->trajectoryReady_ and this->flightState_ != FLIGHT_STATE::INSPECT){
			if (not this->wallDetected_){
				if (this->isWallDetected()){
					this->replan_ = true;
					this->wallDetected_ = true;
					cout << "[AutoFlight]: Replan for wall detection." << endl;
					return;
				}
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

	void dynamicInspection::visCB(){
			this->goalPub_->publish(this->goal_);
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

		if (this->wallRange_.size() != 0){
			this->getWallVisMsg(this->wallVisMsg_);
				this->wallVisPub_->publish(this->wallVisMsg_);
		}
	}


	geometry_msgs::PoseStamped dynamicInspection::getForwardGoal(){
		// if user has specified the inspection goal location, follow the goal
		if (this->inspectionGoalGiven_){
			geometry_msgs::PoseStamped goal;
			goal.pose.position.x = this->inspectionGoal_(0);
			goal.pose.position.y = this->inspectionGoal_(1);
			goal.pose.position.z = this->inspectionGoal_(2);
			this->goal_ = goal;
			return goal;
		}

		// if not meet wall, the goal will always be aggresive (+10 meter in front of the robot)
		bool wallDetected = this->isWallDetected();
		geometry_msgs::PoseStamped goal;
		goal.pose = this->odom_.pose.pose;
		
		if (wallDetected){
			double maxRayLength = 7.0;
			Eigen::Vector3d pStart (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
			Eigen::Vector3d pEnd;
			this->map_->castRay(pStart, Eigen::Vector3d (1, 0, 0), pEnd, maxRayLength);
			goal.pose.position.x = std::max(goal.pose.position.x, pEnd(0)-this->safeDistance_);
		}
		else{
			goal.pose.position.x += 10.0;
			this->goal_ = goal;
		}
		return goal;
	}

	nav_msgs::msg::Path dynamicInspection::getRestGlobalPath(){
		nav_msgs::msg::Path currPath;

		int nextIdx = this->rrtPathMsg_.poses.size()-1;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->rrtPathMsg_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->rrtPathMsg_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::PoseStamped psNext = this->rrtPathMsg_.poses[i+1];
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


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->rrtPathMsg_.poses.size(); ++i){
			currPath.poses.push_back(this->rrtPathMsg_.poses[i]);
		}
		return currPath;		
	}


	void dynamicInspection::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions){
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


	void dynamicInspection::changeState(const FLIGHT_STATE& flightState){
		this->flightState_ = flightState;
		if (flightState != FLIGHT_STATE::EXPLORE){
			this->lookAroundActive_ = false;
			this->lookAroundStep_ = 0;
		}
		if (flightState != FLIGHT_STATE::BACKWARD){
			this->backwardTurnPending_ = false;
		}
		if (flightState != FLIGHT_STATE::FORWARD){
			this->bsplineFailureWaitActive_ = false;
		}
		if (flightState == FLIGHT_STATE::FORWARD or flightState == FLIGHT_STATE::BACKWARD or flightState == FLIGHT_STATE::EXPLORE){
			this->trajectoryReady_ = false;
			this->replan_ = true;
		}
		else{
			this->td_.init = false;
			this->replan_ = false;
		}
	}

	bool dynamicInspection::moveToPosition(const geometry_msgs::msg::Point& position){
		geometry_msgs::PoseStamped psStart, psGoal;
		psGoal.pose.position = position;
		psGoal.pose.orientation = this->odom_.pose.pose.orientation;
		psStart.pose = this->odom_.pose.pose;

		std::vector<geometry_msgs::PoseStamped> linePathVec;
		linePathVec.push_back(psStart);
		linePathVec.push_back(psGoal);
		nav_msgs::msg::Path linePath;
		linePath.poses = linePathVec;

		this->pwlTraj_->updatePath(linePath, this->desiredVel_);
		this->pwlTraj_->makePlan(this->pwlTrajMsg_, 0.1);	
		this->td_.updateTrajectory(this->pwlTrajMsg_, this->pwlTraj_->getDuration());

		rclcpp::Rate r(100.0);
		while (rclcpp::ok() and not this->isReach(psGoal, false)){
			rclcpp::spin_some(this->node_);
			r.sleep();
		}
		return true;
	}

	bool dynamicInspection::moveToPosition(const geometry_msgs::msg::Point& position, double vel){
		geometry_msgs::PoseStamped psStart, psGoal;
		psGoal.pose.position = position;
		psGoal.pose.orientation = this->odom_.pose.pose.orientation;
		psStart.pose = this->odom_.pose.pose;

		std::vector<geometry_msgs::PoseStamped> linePathVec;
		linePathVec.push_back(psStart);
		linePathVec.push_back(psGoal);
		nav_msgs::msg::Path linePath;
		linePath.poses = linePathVec;

		this->pwlTraj_->updatePath(linePath, vel);
		this->pwlTraj_->makePlan(this->pwlTrajMsg_, 0.1);
		this->td_.updateTrajectory(this->pwlTrajMsg_, this->pwlTraj_->getDuration());

		rclcpp::Rate r(100.0);
		while (rclcpp::ok() and not this->isReach(psGoal, false)){
			rclcpp::spin_some(this->node_);
			r.sleep();
		}
		return true;
	}

	bool dynamicInspection::moveToPosition(const Eigen::Vector3d& position){
		geometry_msgs::msg::Point p;
		p.x = position(0);
		p.y = position(1);
		p.z = position(2);
		this->moveToPosition(p);
		return true;
	}

	bool dynamicInspection::moveToPosition(const Eigen::Vector3d& position, double vel){
		geometry_msgs::msg::Point p;
		p.x = position(0);
		p.y = position(1);
		p.z = position(2);
		this->moveToPosition(p, vel);
		return true;
	}
	
	bool dynamicInspection::moveToOrientation(const geometry_msgs::Quaternion& orientation){
		double yawTgt = AutoFlight::rpy_from_quaternion(orientation);
		double yawCurr = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		ps.pose.orientation = orientation;

		double yawDiff = yawTgt - yawCurr; // difference between yaw
		double direction = 0;
		double yawDiffAbs = std::abs(yawDiff);
		if ((yawDiffAbs <= PI_const) and (yawDiff>0)){
			direction = 1.0; // counter clockwise
		} 
		else if ((yawDiffAbs <= PI_const) and (yawDiff<0)){
			direction = -1.0; // clockwise
		}
		else if ((yawDiffAbs > PI_const) and (yawDiff>0)){
			direction = -1.0; // rotate in clockwise direction
			yawDiffAbs = 2 * PI_const - yawDiffAbs;
		}
		else if ((yawDiffAbs > PI_const) and (yawDiff<0)){
			direction = 1.0; // counter clockwise
			yawDiffAbs = 2 * PI_const - yawDiffAbs;
		}


		double t = 0.0;
		double startTime = 0.0; double endTime = yawDiffAbs/this->desiredAngularVel_;

		nav_msgs::msg::Path rotationPath;
		std::vector<geometry_msgs::PoseStamped> rotationPathVec;
		while (t <= endTime){
			double currYawTgt = yawCurr + (double) direction * (t-startTime)/(endTime-startTime) * yawDiffAbs;
			geometry_msgs::Quaternion quatT = trajPlanner::quaternion_from_rpy(0, 0, currYawTgt);
			geometry_msgs::PoseStamped psT = ps;
			psT.pose.orientation = quatT;
			rotationPathVec.push_back(psT);
			t += 0.1;
		}
		rotationPath.poses = rotationPathVec;
		this->useYaw_ = true;
		this->td_.updateTrajectory(rotationPath, endTime);

		rclcpp::Rate r(100.0);
		while (rclcpp::ok() and not this->isReach(ps)){
			rclcpp::spin_some(this->node_);
			r.sleep();
		}
		this->useYaw_ = false;
		return true;
	}

	bool dynamicInspection::moveToOrientation(double yaw){
		geometry_msgs::Quaternion quat = AutoFlight::quaternion_from_rpy(0, 0, yaw);
		this->moveToOrientation(quat);
		return true;
	}

	bool dynamicInspection::moveToOrientationStep(double yaw){
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		double angleDiff = AutoFlight::getAngleDiff(yaw, currYaw);
		double delta = yaw - currYaw;
		double direction = 1;
		if ((delta >= 0) and (std::abs(delta) <= PI_const)){
			direction = 1;
		}
		else if ((delta >=0) and (std::abs(delta) >= PI_const)){
			direction = -1;
		}
		else if ((delta < 0) and (std::abs(delta) <= PI_const)){
			direction = -1;
		}
		else if ((delta < 0) and (std::abs(delta) >= PI_const)){
			direction = 1;
		}

		int count = 1; double divider = 1.0;
		double maxRotationAngle = PI_const/3.0;
		double newAngleDiff = angleDiff;
		while (rclcpp::ok() and newAngleDiff > maxRotationAngle){
			count += 1;
			divider += 1.0;
			newAngleDiff = angleDiff / divider;
		}

        if (angleDiff >= this->confirmMaxAngle_){
            cout << "\033[1;32m[AutoFlight]: Turning...Wait for a few seconds. Then PRESS ENTER to continue or PRESS CTRL+C to land.\033[0m" << endl;
            std::cin.clear();
            fflush(stdin);
            std::cin.get();
        }

		for (int i=1; i<=count; ++i){
			this->moveToOrientation(currYaw + direction * newAngleDiff * i);
			// wait for some time
			rclcpp::Rate r(100.0);
			rclcpp::Time startTime = this->node_->now();
			rclcpp::Time endTime;
			while (rclcpp::ok()){
				endTime = this->node_->now();
				if ((endTime - startTime).seconds() > 1.0){
					break;
				}
				r.sleep();
			}	
			if (angleDiff >= this->confirmMaxAngle_){
				cout << "\033[1;32m[AutoFlight]: Turning...Wait for a few seconds. Then PRESS ENTER to continue or PRESS CTRL+C to land.\033[0m" << endl;
				std::cin.clear();
				fflush(stdin);
				std::cin.get();			
			}
		}
		return true;
	}

	double dynamicInspection::makePWLTraj(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints, nav_msgs::msg::Path& resultPath){
		nav_msgs::msg::Path waypointsMsg;
		waypointsMsg.poses = waypoints;
		this->pwlTraj_->updatePath(waypointsMsg, true);
		this->pwlTraj_->makePlan(resultPath, 0.1);
		return this->pwlTraj_->getDuration();
	}


	double dynamicInspection::makePWLTraj(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints, double desiredVel, nav_msgs::msg::Path& resultPath){
		nav_msgs::msg::Path waypointsMsg;
		waypointsMsg.poses = waypoints;
		this->pwlTraj_->updatePath(waypointsMsg, desiredVel, true);
		this->pwlTraj_->makePlan(resultPath, 0.1);
		return this->pwlTraj_->getDuration();
	}

	double dynamicInspection::getPathLength(const nav_msgs::msg::Path& path){
		double length = 0.0;
		for (size_t i=0; i<path.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps1 = path.poses[i];
			geometry_msgs::PoseStamped ps2 = path.poses[i+1];
			Eigen::Vector3d p1 (ps1.pose.position.x, ps1.pose.position.y, ps1.pose.position.z);
			Eigen::Vector3d p2 (ps2.pose.position.x, ps2.pose.position.y, ps2.pose.position.z);
			length += (p1 - p2).norm();
		}
		return length;
	}

	bool dynamicInspection::getBestViewPoint(Eigen::Vector3d& bestPoint){
		int bestUnknown = 0;
		for (int i=0; i<this->exploreSampleNum_; ++i){
			Eigen::Vector3d pRandom;
			bool sampleSuccess = this->randomSample(pRandom);
			if (not sampleSuccess){
				return false;
			}
			int countUnknown = this->countUnknownFOV(pRandom, 0);
			if (countUnknown > bestUnknown){
				bestUnknown = countUnknown;
				bestPoint = pRandom;
			}
		}
		return true;
	}

	bool  dynamicInspection::randomSample(Eigen::Vector3d& pSample){
		Eigen::Vector3d mapSizeMin, mapSizeMax;
		this->map_->getMapRange(mapSizeMin, mapSizeMax);

		rclcpp::Time startTime = this->node_->now();
		for (int iter = 0; iter < 200; ++iter){
			if ((this->node_->now() - startTime).seconds() > 1.0){
				cout << "[AutoFlight]: Explore random sample timeout." << endl;
				return false;
			}
			pSample(0) = AutoFlight::randomNumber(this->odom_.pose.pose.position.x, mapSizeMax(0));
			pSample(1) = AutoFlight::randomNumber(mapSizeMin(1), mapSizeMax(1));
			pSample(2) = this->takeoffHgt_;
			
			if (not this->map_->isInflatedOccupied(pSample) and not this->map_->isUnknown(pSample) and this->satisfyWallDistance(pSample)){
				return true;			
			}
		}
		return false;
	}

	bool dynamicInspection::satisfyWallDistance(const Eigen::Vector3d& p){
		// check front and side
		Eigen::Vector3d pEndFront, pEndLeft, pEndRight;
		bool ignoreUnknown = false; 
		bool castSuccessFront = this->map_->castRay(p, Eigen::Vector3d (1, 0, 0), pEndFront, this->safeDistance_, ignoreUnknown);
		// bool castSuccessLeft = this->map_->castRay(p, Eigen::Vector3d (0, 1, 0), pEndLeft, this->sideSafeDistance_, ignoreUnknown);
		// bool castSuccessRight = this->map_->castRay(p, Eigen::Vector3d (0, -1, 0), pEndRight, this->sideSafeDistance_, ignoreUnknown);
		bool castSuccessLeft = false;
		bool castSuccessRight = false;


		if (castSuccessFront or castSuccessRight or castSuccessLeft){
			return false;
		}
		else{
			return true;
		}
	}

	int dynamicInspection::countUnknownFOV(const Eigen::Vector3d& p, double yaw){
		double height = 2 * tan(this->sensorAngleV_/2.0) * this->sensorRange_;
		double width = 2 * tan(this->sensorAngleH_/2.0) * this->sensorRange_;

		// create bounding box for search
		double xmin = p(0);              double xmax = p(0) + this->sensorRange_;
		double ymin = p(1) - width/2.0;  double ymax = p(1) + width/2.0;
		double zmin = p(2) - height/2.0; double zmax = p(2) + height/2.0;
		
		Eigen::Vector3d faceDirection (cos(yaw), sin(yaw), 0);
		int countUnknown = 0;
		for (double x=xmin; x<=xmax; x+=this->map_->getRes()){
			for (double y=ymin; y<=ymax; y+=this->map_->getRes()){
				for (double z=zmin; z<=zmax; z+=this->map_->getRes()){
					Eigen::Vector3d pQuery(x, y, z);
					if (not this->map_->isInMap(pQuery)){
						continue;
					}
					else{
						if (this->map_->isUnknown(pQuery)){ // need to be unknown voxel
							Eigen::Vector3d direction = pQuery - p;
							Eigen::Vector3d directionH = direction; directionH(2) = 0;
							Eigen::Vector3d directionV = direction; directionV(1) = 0;

							double angleH = trajPlanner::angleBetweenVectors(directionH, faceDirection);
							double angleV = trajPlanner::angleBetweenVectors(directionV, faceDirection);

							if (angleH <= this->sensorAngleH_/2.0 and angleV <= this->sensorAngleV_/2.0){
								++countUnknown;
							}
						}
					}
				}
			}
		}
		return countUnknown;
	}

	void dynamicInspection::setStartPositionFree(){
		Eigen::Vector3d lowerPos, upperPos;
		Eigen::Vector3d centerPos (0 , 0, this->takeoffHgt_);
		lowerPos = centerPos - Eigen::Vector3d (0.5, 0.5, 0.5);
		upperPos = centerPos + Eigen::Vector3d (0.5, 0.5, 0.5);
		this->map_->freeRegion(lowerPos, upperPos);
	}

	bool dynamicInspection::castRayOccupied(const Eigen::Vector3d& start, const Eigen::Vector3d& direction, Eigen::Vector3d& end, double maxRayLength){
		// return true if raycasting successfully find the endpoint, otherwise return false
		if (not this->map_->isInflatedOccupied(start)){ // 
			end = start;
			return false;
		}

		Eigen::Vector3d directionNormalized = direction/direction.norm(); // normalize the direction vector
		int num = ceil(maxRayLength/this->map_->getRes());
		for (int i=1; i<num; ++i){
			Eigen::Vector3d point = this->map_->getRes() * directionNormalized * i + start;
			if (not this->map_->isInflatedOccupied(point)){
				end = point;
				return true;
			}
		}
		end = start;
		return false;
	}

	bool dynamicInspection::isWallDetected(){
		if (this->wallRange_.size() == 0){
			return false;
		}
		double area = (this->wallRange_[3] - this->wallRange_[2]) * (this->wallRange_[5] - this->wallRange_[4]);
		if (area > this->minWallArea_){
			return true;
		}
		else{
			return false;
		}
	}

	double dynamicInspection::getWallDistance(){
		return std::abs(this->wallRange_[0] - this->odom_.pose.pose.position.x);
	}

	void dynamicInspection::updateWallRange(const std::vector<double>& wallRange){
		this->wallRange_ = wallRange;
	}


	visualization_msgs::Marker dynamicInspection::getLineMarker(double x1, double y1, double z1, 
										  						double x2, double y2, double z2,
										  						int id, bool isWall){
		std::vector<geometry_msgs::msg::Point> lineVec;
		geometry_msgs::msg::Point p1, p2;
		p1.x = x1; p1.y = y1; p1.z = z1;
		p2.x = x2; p2.y = y2; p2.z = z2;
		lineVec.push_back(p1); 
		lineVec.push_back(p2);

		visualization_msgs::Marker lineMarker;
		lineMarker.header.frame_id = "map";
		lineMarker.header.stamp = this->node_->now();
		lineMarker.ns = "inspection_target";
		lineMarker.id = id;
		lineMarker.type = visualization_msgs::Marker::LINE_LIST;
		lineMarker.action = visualization_msgs::Marker::ADD;
		lineMarker.points = lineVec;
		lineMarker.lifetime = rclcpp::Duration::from_seconds(2.0);
		lineMarker.scale.x = 0.1;
		lineMarker.scale.y = 0.1;
		lineMarker.scale.z = 0.1;
		lineMarker.color.a = 1.0; // Don't forget to set the alpha!
		if (isWall){
			lineMarker.color.r = 1.0;
			lineMarker.color.g = 0.0;
			lineMarker.color.b = 0.0;
		}
		else{
			lineMarker.color.r = 1.0;
			lineMarker.color.g = 0.0;
			lineMarker.color.b = 1.0;
		}
		
		return lineMarker;
	}

	void dynamicInspection::getWallVisMsg(visualization_msgs::MarkerArray& msg){
		msg.markers.clear();
		double xmin, xmax, ymin, ymax, zmin, zmax;
		xmin = this->wallRange_[0]; xmax = this->wallRange_[1];
		ymin = this->wallRange_[2]; ymax = this->wallRange_[3];
		zmin = this->wallRange_[4]; zmax = this->wallRange_[5];

		bool isWall = this->isWallDetected();
		std::vector<visualization_msgs::Marker> visVec;
		visualization_msgs::Marker l1 = this->getLineMarker(xmin, ymin, zmin, xmin, ymax, zmin, 1, isWall);
		visualization_msgs::Marker l2 = this->getLineMarker(xmin, ymax, zmin, xmax, ymax, zmin, 2, isWall);
		visualization_msgs::Marker l3 = this->getLineMarker(xmax, ymax, zmin, xmax, ymin, zmin, 3, isWall);
		visualization_msgs::Marker l4 = this->getLineMarker(xmin, ymin, zmin, xmax, ymin, zmin, 4, isWall);
		visualization_msgs::Marker l5 = this->getLineMarker(xmin, ymin, zmin, xmin, ymin, zmax, 5, isWall);
		visualization_msgs::Marker l6 = this->getLineMarker(xmin, ymax, zmin, xmin, ymax, zmax, 6, isWall);
		visualization_msgs::Marker l7 = this->getLineMarker(xmax, ymax, zmin, xmax, ymax, zmax, 7, isWall);
		visualization_msgs::Marker l8 = this->getLineMarker(xmax, ymin, zmin, xmax, ymin, zmax, 8, isWall);
		visualization_msgs::Marker l9 = this->getLineMarker(xmin, ymin, zmax, xmin, ymax, zmax, 9, isWall);
		visualization_msgs::Marker l10 = this->getLineMarker(xmin, ymax, zmax, xmax, ymax, zmax, 10, isWall);
		visualization_msgs::Marker l11 = this->getLineMarker(xmax, ymax, zmax, xmax, ymin, zmax, 11, isWall);
		visualization_msgs::Marker l12 = this->getLineMarker(xmin, ymin, zmax, xmax, ymin, zmax, 12, isWall);

		// line 1-12
		visVec.push_back(l1);
		visVec.push_back(l2);
		visVec.push_back(l3);
		visVec.push_back(l4);
		visVec.push_back(l5);
		visVec.push_back(l6);
		visVec.push_back(l7);
		visVec.push_back(l8);
		visVec.push_back(l9);
		visVec.push_back(l10);
		visVec.push_back(l11);
		visVec.push_back(l12);

		msg.markers = visVec;
	}


	void dynamicInspection::checkSurroundings(){
		// 1. find all the height level
		std::vector<double> heightLevels;
		double currHeight = this->odom_.pose.pose.position.z;
		double heightTemp = currHeight;
		while (rclcpp::ok() and heightTemp < this->inspectionHeight_){
			heightLevels.push_back(heightTemp);
			heightTemp += this->ascendStep_;
		}
		heightLevels.push_back(this->inspectionHeight_);

		// 2. for each height level check left and right
		double maxRayLength = 7.0;
		rclcpp::Rate r(100.0);

		for (size_t i=0; i<heightLevels.size(); ++i){
			Eigen::Vector3d pHeight (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, heightLevels[i]);

			// a. turn left
			Eigen::Vector3d leftEnd;
			bool castLeftSuccess = this->map_->castRay(pHeight, Eigen::Vector3d (0, 1, 0), leftEnd, maxRayLength);
			
		
			if (not castLeftSuccess){
				this->moveToOrientationStep(PI_const/2);
				// translation
				while (rclcpp::ok() and not castLeftSuccess){
					geometry_msgs::PoseStamped pStart, pGoal;
					pStart.pose = this->odom_.pose.pose;
					pGoal = pStart; pGoal.pose.position.y += 1.0;
					std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
					double duration = this->makePWLTraj(pathVec, this->pwlTrajMsg_);
					this->td_.updateTrajectory(this->pwlTrajMsg_, duration);

					Eigen::Vector3d pCurr(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
					castLeftSuccess = this->map_->castRay(pCurr, Eigen::Vector3d (0, 1, 0), leftEnd, maxRayLength);
					rclcpp::spin_some(this->node_);
					r.sleep();
				}

				this->moveToOrientationStep(0);
			}
			

			// b. turn right
			Eigen::Vector3d rightEnd;
			bool castRightSuccess = this->map_->castRay(pHeight, Eigen::Vector3d(0, -1, 0), rightEnd, maxRayLength);
			if (not castRightSuccess){
				this->moveToOrientation(-PI_const/2);
				while (rclcpp::ok() and not castRightSuccess){
					geometry_msgs::PoseStamped pStart, pGoal;
					pStart.pose = this->odom_.pose.pose;
					pGoal = pStart; pGoal.pose.position.y -= 1.0;
					std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
					double duration = this->makePWLTraj(pathVec, this->pwlTrajMsg_);
					this->td_.updateTrajectory(this->pwlTrajMsg_, duration);
					
					Eigen::Vector3d pCurr(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
					castRightSuccess = this->map_->castRay(pCurr, Eigen::Vector3d (0, -1, 0), rightEnd, maxRayLength);
					rclcpp::spin_some(this->node_);
					r.sleep();
				}

				this->moveToOrientationStep(0);
			}

			// c. back to the center of the wall
			Eigen::Vector3d pCenter = (rightEnd + leftEnd)/2.0;
			this->moveToPosition(pCenter, this->inspectionVel_);
			
			// d. move to next height
			if (i != heightLevels.size()-1){
				double height  = heightLevels[i+1];
				Eigen::Vector3d pNextHeight (pCenter(0), pCenter(1), height);
				this->moveToPosition(pNextHeight, this->inspectionVel_);
			}

		}
	}

	void dynamicInspection::inspectZigZag(){
		if (this->inspectionConfirm_){
			cout << "\033[1;32m[AutoFlight]: Check flight conditions. Then PRESS ENTER to continue ZIG-ZAG or PRESS CTRL+C to land.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();
		}
		cout << "[AutoFlight]: Start Zig-Zag Inspection..." << endl;
		std::vector<geometry_msgs::PoseStamped> zigzagPathVec;

		// 1. find all height levels
		std::vector<double> heightLevels;
		
		double currHeight = this->odom_.pose.pose.position.z;
		double heightTemp = currHeight;
		while (rclcpp::ok() and heightTemp > this->takeoffHgt_){
			heightLevels.push_back(heightTemp);
			heightTemp -= this->descendStep_;
		}
		heightLevels.push_back(this->takeoffHgt_);


		// 2. cast rays to two sides
		double currX = this->odom_.pose.pose.position.x;
		double currY = this->odom_.pose.pose.position.y;
		double maxRayLength = 7.0;
		
		Eigen::Vector3d pStart (currX, currY, currHeight);
		geometry_msgs::PoseStamped psStart = this->eigen2ps(pStart);
		psStart.pose.orientation.w = 1.0;
		zigzagPathVec.push_back(psStart);

		int count = 0;
		bool ignoreUnknown = true;
		for (double height : heightLevels){
			Eigen::Vector3d pCurr (currX, currY, height);

			// cast to left
			Eigen::Vector3d pLeftEnd;
			this->map_->castRay(pCurr, Eigen::Vector3d (0, 1, 0), pLeftEnd, maxRayLength, ignoreUnknown);
			pLeftEnd(1) = std::max(pLeftEnd(1) - this->sideSafeDistance_, currY); 
			geometry_msgs::PoseStamped psLeft = this->eigen2ps(pLeftEnd);
			psLeft.pose.orientation.w = 1.0;

			// cast to right
			Eigen::Vector3d pRightEnd;
			this->map_->castRay(pCurr, Eigen::Vector3d (0, -1, 0), pRightEnd, maxRayLength, ignoreUnknown);
			pRightEnd(1) = std::min(pRightEnd(1) + this->sideSafeDistance_, currY);
			geometry_msgs::PoseStamped psRight = this->eigen2ps(pRightEnd);
			psRight.pose.orientation.w = 1.0;

			if (count % 2 == 0){
				if (count != 0){
					geometry_msgs::PoseStamped psPrev = zigzagPathVec.back();
					psPrev.pose.position.z =  height;
					zigzagPathVec.push_back(psPrev);
				}
				if (this->leftFirst_){
					zigzagPathVec.push_back(psLeft);
					zigzagPathVec.push_back(psRight);
				}
				else{
					zigzagPathVec.push_back(psRight);
					zigzagPathVec.push_back(psLeft);			
				}
			}
			else{
				geometry_msgs::PoseStamped psPrev = zigzagPathVec.back();
				psPrev.pose.position.z =  height;
				zigzagPathVec.push_back(psPrev);
				if (this->leftFirst_){
					zigzagPathVec.push_back(psRight);
					zigzagPathVec.push_back(psLeft);
				}	
				else{
					zigzagPathVec.push_back(psLeft);
					zigzagPathVec.push_back(psRight);					
				}	
			}
			++count;
		}

		Eigen::Vector3d pEnd (currX, currY, this->takeoffHgt_);
		geometry_msgs::PoseStamped psEnd = this->eigen2ps(pEnd);
		psEnd.pose.orientation.w = 1.0;
		zigzagPathVec.push_back(psEnd);


		double duration = this->makePWLTraj(zigzagPathVec, this->inspectionVel_, this->pwlTrajMsg_);
		this->startAction(this->pwlTrajMsg_, duration, psEnd, false);
	}

	void dynamicInspection::inspectZigZagRange(){
		if (this->inspectionConfirm_){
			cout << "\033[1;32m[AutoFlight]: Check flight conditions. Then PRESS ENTER to continue ZIG-ZAG or PRESS CTRL+C to land.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();
		}
		cout << "[AutoFlight]: Start Zig-Zag Inspection..." << endl;
		std::vector<geometry_msgs::PoseStamped> zigzagPathVec;

		// 1. use the given range to construct the path
		double inspectionOrientation = 0;
		if (this->inspectionGoalGiven_){
			inspectionOrientation = this->inspectionOrientation_;
		}
		geometry_msgs::Quaternion quat = AutoFlight::quaternion_from_rpy(0, 0, inspectionOrientation);
		geometry_msgs::msg::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		psCurr.pose.orientation = quat;
		zigzagPathVec.push_back(psCurr);
		if (std::abs(psCurr.pose.position.z - this->inspectionHeight_) > 1e-3){
			geometry_msgs::msg::PoseStamped psHeight = psCurr;
			psHeight.pose.position.z = this->inspectionHeight_;
			zigzagPathVec.push_back(psHeight);
		}

		// 2. find all height levels
		std::vector<double> heightLevels;
		
		double currHeight = this->inspectionHeight_;
		double heightTemp = currHeight;
		while (rclcpp::ok() and heightTemp > this->takeoffHgt_){
			heightLevels.push_back(heightTemp);
			heightTemp -= this->descendStep_;
		}
		heightLevels.push_back(this->takeoffHgt_);
		
		double currX = this->odom_.pose.pose.position.x;
		double currY = this->odom_.pose.pose.position.y;
	
		int count = 0;
		for (double height : heightLevels){
			Eigen::Vector3d pLeft (currX - this->inspectionWidth_/2.0 * sin(inspectionOrientation), currY + this->inspectionWidth_/2.0 * cos(inspectionOrientation), height);
			Eigen::Vector3d pRight (currX + this->inspectionWidth_/2.0 * sin(inspectionOrientation), currY - this->inspectionWidth_/2.0 * cos(inspectionOrientation), height);
			geometry_msgs::PoseStamped psLeft = this->eigen2ps(pLeft);
			psLeft.pose.orientation = quat;
			geometry_msgs::PoseStamped psRight = this->eigen2ps(pRight);
			psRight.pose.orientation = quat;

			if (count % 2 == 0){
				if (count != 0){
					geometry_msgs::PoseStamped psPrev = zigzagPathVec.back();
					psPrev.pose.position.z =  height;
					zigzagPathVec.push_back(psPrev);					
				}
				if (this->leftFirst_){
					zigzagPathVec.push_back(psLeft);
					zigzagPathVec.push_back(psRight);
				}
				else{
					zigzagPathVec.push_back(psRight);
					zigzagPathVec.push_back(psLeft);			
				}
			}
			else{
				geometry_msgs::PoseStamped psPrev = zigzagPathVec.back();
				psPrev.pose.position.z =  height;
				zigzagPathVec.push_back(psPrev);
				if (this->leftFirst_){
					zigzagPathVec.push_back(psRight);
					zigzagPathVec.push_back(psLeft);
				}	
				else{
					zigzagPathVec.push_back(psLeft);
					zigzagPathVec.push_back(psRight);					
				}			
			}
			++count;
		}

		Eigen::Vector3d pEnd (currX, currY, this->takeoffHgt_);
		geometry_msgs::PoseStamped psEnd = this->eigen2ps(pEnd);
		psEnd.pose.orientation = quat;
		zigzagPathVec.push_back(psEnd);

		double duration = this->makePWLTraj(zigzagPathVec, this->inspectionVel_, this->pwlTrajMsg_);
		this->startAction(this->pwlTrajMsg_, duration, psEnd, false);
	}

	void dynamicInspection::inspectFringe(){
		if (this->inspectionConfirm_){
			cout << "\033[1;32m[AutoFlight]: Check flight conditions. Then PRESS ENTER to continue FRINGE or PRESS CTRL+C to land.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();
		}
		cout << "[AutoFlight]: Start Fringe Inspection..." << endl;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d pHeight (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->inspectionHeight_);

		// double currX = this->odom_.pose.pose.position.x;
		double currY = this->odom_.pose.pose.position.y;
		double maxRayLength = 7.0;
		bool ignoreUnknown = true;


		// cast to left for current point
		Eigen::Vector3d pLeftEnd;
		this->map_->castRay(pCurr, Eigen::Vector3d (0, 1, 0), pLeftEnd, maxRayLength, ignoreUnknown);
		pLeftEnd(1) = std::max(pLeftEnd(1) - this->sideSafeDistance_, currY); 
		geometry_msgs::PoseStamped psLeft = this->eigen2ps(pLeftEnd);
		psLeft.pose.orientation.w = 1.0;

		// cast to right for current point
		Eigen::Vector3d pRightEnd;
		this->map_->castRay(pCurr, Eigen::Vector3d (0, -1, 0), pRightEnd, maxRayLength, ignoreUnknown);
		pRightEnd(1) = std::min(pRightEnd(1) + this->sideSafeDistance_, currY); 
		geometry_msgs::PoseStamped psRight = this->eigen2ps(pRightEnd);
		psRight.pose.orientation.w = 1.0;	


		// cast to left for height point
		Eigen::Vector3d pLeftEndHeight;
		this->map_->castRay(pHeight, Eigen::Vector3d (0, 1, 0), pLeftEndHeight, maxRayLength, ignoreUnknown);
		pLeftEndHeight(1) = std::max(pLeftEndHeight(1) - this->sideSafeDistance_, currY); 
		geometry_msgs::PoseStamped psLeftHeight = this->eigen2ps(pLeftEndHeight);
		psLeftHeight.pose.orientation.w = 1.0;

		// cast to right for height point
		Eigen::Vector3d pRightEndHeight;
		this->map_->castRay(pHeight, Eigen::Vector3d (0, -1, 0), pRightEndHeight, maxRayLength, ignoreUnknown);
		pRightEndHeight(1) = std::min(pRightEndHeight(1) + this->sideSafeDistance_, currY); 
		geometry_msgs::PoseStamped psRightHeight = this->eigen2ps(pRightEndHeight);
		psRightHeight.pose.orientation.w = 1.0;


		// start pose
		geometry_msgs::PoseStamped psStart = this->eigen2ps(pCurr);
		psStart.pose.orientation.w = 1.0;
		geometry_msgs::PoseStamped psHeight = this->eigen2ps(pHeight);
		psHeight.pose.orientation.w = 1.0;


		std::vector<geometry_msgs::PoseStamped> pathVec1;
		if (this->leftFirst_){
			std::vector<geometry_msgs::PoseStamped> pathVecTemp {psStart, psLeft, psLeftHeight, psHeight};
			pathVec1 = pathVecTemp;
		}
		else{
			std::vector<geometry_msgs::PoseStamped> pathVecTemp {psStart, psRight, psRightHeight, psHeight};
			pathVec1 = pathVecTemp;
		}
		this->fringePath1_.poses = pathVec1;
		this->fringeDuration1_ = this->makePWLTraj(pathVec1, this->inspectionVel_, this->fringePath1_);
		this->fringeGoal1_ = psHeight;

		std::vector<geometry_msgs::PoseStamped> pathVec2;
		if (this->leftFirst_){
			std::vector<geometry_msgs::PoseStamped> pathVecTemp {psHeight, psRightHeight, psRight, psStart};
			pathVec2 = pathVecTemp;
		}
		else{
			std::vector<geometry_msgs::PoseStamped> pathVecTemp {psHeight, psLeftHeight, psLeft, psStart};
			pathVec2 = pathVecTemp;
		}
		this->fringePath2_.poses = pathVec2;
		this->fringeDuration2_ = this->makePWLTraj(pathVec2, this->inspectionVel_, this->fringePath2_);
		this->fringeGoal2_ = psStart;
	}

	void dynamicInspection::inspectFringeRange(){
		if (this->inspectionConfirm_){
			cout << "\033[1;32m[AutoFlight]: Check flight conditions. Then PRESS ENTER to continue FRINGE or PRESS CTRL+C to land.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();
		}
		cout << "[AutoFlight]: Start Fringe Inspection..." << endl;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d pHeight (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->inspectionHeight_);		
		
		double currX = this->odom_.pose.pose.position.x;
		double currY = this->odom_.pose.pose.position.y;


		double inspectionOrientation = 0;
		if (this->inspectionGoalGiven_){
			inspectionOrientation = this->inspectionOrientation_;
		}

		Eigen::Vector3d pLeft (currX - this->inspectionWidth_/2.0 * sin(inspectionOrientation), currY + this->inspectionWidth_/2.0 * cos(inspectionOrientation), pCurr(2));
		geometry_msgs::PoseStamped psLeft = this->eigen2ps(pLeft);
		psLeft.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, inspectionOrientation + PI_const/4.0);

		Eigen::Vector3d pRight (currX + this->inspectionWidth_/2.0 * sin(inspectionOrientation), currY - this->inspectionWidth_/2.0 * cos(inspectionOrientation), pCurr(2));
		geometry_msgs::PoseStamped psRight = this->eigen2ps(pRight);
		psRight.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, inspectionOrientation - PI_const/4.0);

		geometry_msgs::PoseStamped psLeftHeight = psLeft;
		psLeftHeight.pose.position.z = this->inspectionHeight_;

		geometry_msgs::PoseStamped psRightHeight = psRight;
		psRightHeight.pose.position.z = this->inspectionHeight_;

		geometry_msgs::PoseStamped psCurr1 = this->eigen2ps(pCurr);
		psCurr1.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, inspectionOrientation + PI_const/4.0);

		geometry_msgs::PoseStamped psCurr2 = this->eigen2ps(pCurr);
		psCurr2.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, inspectionOrientation - PI_const/4.0);

		geometry_msgs::PoseStamped psHeight1 = this->eigen2ps(pHeight);
		psHeight1.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, inspectionOrientation + PI_const/4.0);

		geometry_msgs::PoseStamped psHeight2 = this->eigen2ps(pHeight);
		psHeight2.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, inspectionOrientation - PI_const/4.0);


		std::vector<geometry_msgs::PoseStamped> pathVec1;
		if (this->leftFirst_){
			std::vector<geometry_msgs::PoseStamped> pathVecTemp {psCurr1, psLeft, psLeftHeight, psHeight1};
			pathVec1 = pathVecTemp;
			this->fringeGoal1_ = psHeight1;
		}
		else{
			std::vector<geometry_msgs::PoseStamped> pathVecTemp {psCurr2, psRight, psRightHeight, psHeight2};
			pathVec1 = pathVecTemp;
			this->fringeGoal1_ = psHeight2;
		}
		this->fringePath1_.poses = pathVec1;
		this->fringeDuration1_ = this->makePWLTraj(pathVec1, this->inspectionVel_, this->fringePath1_);


		std::vector<geometry_msgs::PoseStamped> pathVec2;
		if (this->leftFirst_){
			std::vector<geometry_msgs::PoseStamped> pathVecTemp {psHeight2, psRightHeight, psRight, psCurr2};
			pathVec2 = pathVecTemp;
			this->fringeGoal2_ = psCurr2;
		}
		else{
			std::vector<geometry_msgs::PoseStamped> pathVecTemp {psHeight1, psLeftHeight, psLeft, psCurr1};
			pathVec2 = pathVecTemp;
			this->fringeGoal2_ = psCurr1;
		}
		this->fringePath2_.poses = pathVec2;
		this->fringeDuration2_ = this->makePWLTraj(pathVec2, this->inspectionVel_, this->fringePath2_);

	}

	bool dynamicInspection::hasCollision(){
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

	bool dynamicInspection::hasDynamicCollision(){
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

	double dynamicInspection::computeExecutionDistance(){
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

	bool dynamicInspection::replanForDynamicObstacle(){
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

	nav_msgs::msg::Path dynamicInspection::getCurrentTraj(double dt){
	nav_msgs::msg::Path currentTraj;
		currentTraj.header.frame_id = "map";
		currentTraj.header.stamp = this->node_->now();
	
		if (this->trajectoryReady_){
			// include the current pose
			// geometry_msgs::PoseStamped psCurr;
			// psCurr.pose = this->odom_.pose.pose;
			// currentTraj.poses.push_back(psCurr);
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=dt){
				Eigen::Vector3d pos = this->trajectory_.at(t);
				geometry_msgs::PoseStamped ps;
				ps.pose.position.x = pos(0);
				ps.pose.position.y = pos(1);
				ps.pose.position.z = pos(2);
				currentTraj.poses.push_back(ps);
			}		
		}
		return currentTraj;
	}

	geometry_msgs::PoseStamped dynamicInspection::eigen2ps(const Eigen::Vector3d& p){
		geometry_msgs::PoseStamped ps;
		ps.pose.position.x = p(0);
		ps.pose.position.y = p(1);
		ps.pose.position.z = p(2);
		return ps;
	}
}