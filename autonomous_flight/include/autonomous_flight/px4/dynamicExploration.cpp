/*
	FILE: dynamicExploration.cpp
	-----------------------------
	Implementation of dynamic exploration
*/

#include <autonomous_flight/px4/dynamicExploration.h>
#include <std_srvs/srv/trigger.hpp>
#include <rmw/qos_profiles.h>
#include <limits>
#include <cmath>

namespace AutoFlight{
	namespace {
		inline bool isFinitePosePosition(const geometry_msgs::msg::Pose& pose){
			return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && std::isfinite(pose.position.z);
		}
	}

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

		this->node_->declare_parameter<double>("collision_replan_cooldown_sec", 0.30);
		this->node_->get_parameter("collision_replan_cooldown_sec", this->collisionReplanCooldownSec_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Collision replan cooldown is set to: %.2fs.", this->collisionReplanCooldownSec_);

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

		this->node_->declare_parameter<double>("min_waypoint_distance", 0.2);
		this->node_->get_parameter("min_waypoint_distance", this->minWaypointDistance_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Min waypoint distance is set to: %.2fm.", this->minWaypointDistance_);

		this->node_->declare_parameter<bool>("replan_on_finish_or_fail", true);
		this->node_->get_parameter("replan_on_finish_or_fail", this->replanOnFinishOrFail_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Replan on finish/fail is set to: %s.", this->replanOnFinishOrFail_ ? "true" : "false");

		this->node_->declare_parameter<bool>("replan_on_collision_fail", true);
		this->node_->get_parameter("replan_on_collision_fail", this->replanOnCollisionFail_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Replan on collision fail is set to: %s.", this->replanOnCollisionFail_ ? "true" : "false");

		this->node_->declare_parameter<bool>("stabilize_before_rotate", true);
		this->node_->get_parameter("stabilize_before_rotate", this->stabilizeBeforeRotate_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Stabilize before rotate is set to: %s.", this->stabilizeBeforeRotate_ ? "true" : "false");

		this->node_->declare_parameter<bool>("require_operator_confirmation", false);
		this->node_->get_parameter("require_operator_confirmation", this->operatorConfirm_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Operator confirmation is set to: %s.", this->operatorConfirm_ ? "true" : "false");

		this->node_->declare_parameter<double>("end_mission_max_segment_distance", 1.5);
		this->node_->get_parameter("end_mission_max_segment_distance", this->endMissionMaxSegmentDistance_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: End mission max segment distance is set to: %.2fm.", this->endMissionMaxSegmentDistance_);
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

	void dynamicExploration::clearWaypointPlan(){
		this->newWaypoints_ = false;
		this->waypoints_.poses.clear();
		this->waypointIdx_ = 1;
	}

	nav_msgs::msg::Path dynamicExploration::buildTwoPointPath(double x, double y, double z) const{
		nav_msgs::msg::Path path;
		path.header.frame_id = this->mapFrameId_;
		path.header.stamp = this->node_->now();

		geometry_msgs::msg::PoseStamped psCurr;
		psCurr.header = path.header;
		psCurr.pose = this->odom_.pose.pose;
		path.poses.push_back(psCurr);

		geometry_msgs::msg::PoseStamped psGoal;
		psGoal.header = path.header;
		psGoal.pose = this->odom_.pose.pose;
		psGoal.pose.position.x = x;
		psGoal.pose.position.y = y;
		psGoal.pose.position.z = z;
		path.poses.push_back(psGoal);
		return path;
	}

	void dynamicExploration::requestExplorationReplan(bool enabled){
		if (!enabled){
			return;
		}
		this->clearWaypointPlan();
		this->explorationReplan_ = true;
	}

	void dynamicExploration::endMission(){
		this->trajectoryReady_ = false;
		this->replan_ = false;
		this->waypointRotatePending_ = false;
		this->explorationReplan_ = false;
		this->clearWaypointPlan();

		nav_msgs::msg::Path missionPath;
		this->expPlanner_->setMap(this->map_);
		const bool depReady = this->expPlanner_->makePlan();
		const Eigen::Vector3d homeQuery(0.0, 0.0, this->odom_.pose.pose.position.z);
		const bool hasRoadmapHomePath = depReady && this->expPlanner_->getRoadmapPathToPosition(homeQuery, missionPath);

		if (!hasRoadmapHomePath){
			missionPath = this->buildTwoPointPath(0.0, 0.0, this->odom_.pose.pose.position.z);
			RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: DEP roadmap return path unavailable. Falling back to direct home segment.");
		}

		this->waypoints_ = missionPath;
		this->waypointIdx_ = 1;
		this->newWaypoints_ = true;
		this->endMissionActive_ = true;
		this->landingPhase_ = false;
		this->missionEnded_ = false;
		this->endMissionLastRetrySec_ = -1.0;

		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: End mission requested. Returning to roadmap node closest to home (x=0.0, y=0.0).");
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
		std::scoped_lock<std::mutex> lock(this->navStateMutex_);

		if (this->replan_){
			if (!isFinitePosePosition(this->odom_.pose.pose) || !isFinitePosePosition(this->goal_.pose)){
				RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: Invalid (NaN/Inf) start or goal pose detected. Skipping this replan iteration.");
				this->replan_ = false;
				this->trajectoryReady_ = false;
				if (this->endMissionActive_){
					this->newWaypoints_ = true;
				}
				return;
			}

			if (!this->isGoalValid()){
				RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: Current goal is invalid before local planning. Skipping this replan iteration.");
				this->replan_ = false;
				this->trajectoryReady_ = false;
				if (this->endMissionActive_){
					this->newWaypoints_ = true;
				}
				return;
			}

			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			nav_msgs::msg::Path inputTraj;
			std::vector<Eigen::Vector3d> startEndConditions;
			this->getStartEndConditions(startEndConditions);

			nav_msgs::msg::Path simplePath;
			geometry_msgs::msg::PoseStamped pStart, pGoal;
			pStart.pose = this->odom_.pose.pose;
			pGoal = this->goal_;
			simplePath.poses = {pStart, pGoal};
			this->pwlTraj_->updatePath(simplePath, false);
			this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
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

					this->trajectoryReady_ = true;
					this->replan_ = false;
					cout << "\033[1;32m[AutoFlight]: Trajectory generated successfully.\033[0m " << endl;
				}
				else{
					if (this->hasCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails." << endl;
						this->replan_ = false;
						this->requestExplorationReplan(this->replanOnCollisionFail_);
					}
					else if (this->hasDynamicCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails. Replan for dynamic obstacles." << endl;
						this->replan_ = true;
						this->requestExplorationReplan(this->replanOnCollisionFail_);
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
							this->requestExplorationReplan(this->replanOnCollisionFail_);
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
		std::unique_lock<std::mutex> lock(this->navStateMutex_);

		if (this->endMissionRequested_){
			this->endMissionRequested_ = false;
			this->endMission();
		}

		if (this->endMissionActive_){
			if (this->newWaypoints_){
				if (this->waypoints_.poses.size() < 2){
					RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: End mission path invalid (size=%zu). Holding position.", this->waypoints_.poses.size());
					this->newWaypoints_ = false;
					this->replan_ = false;
					this->trajectoryReady_ = false;
					this->stop();
					return;
				}

				size_t startIdx = static_cast<size_t>(std::max(1, this->waypointIdx_));
				startIdx = std::min(startIdx, this->waypoints_.poses.size() - 1);

				const auto& currPos = this->odom_.pose.pose.position;
				int chosenIdx = -1;
				double bestDist = std::numeric_limits<double>::infinity();
				for (size_t i = startIdx; i < this->waypoints_.poses.size(); ++i){
					const auto& ps = this->waypoints_.poses[i];
					if (!isFinitePosePosition(ps.pose)){
						continue;
					}
					Eigen::Vector3d pGoal(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
					if (!this->expPlanner_->isPosValid(pGoal)){
						continue;
					}
					const auto& pose = ps.pose.position;
					const double dx = pose.x - currPos.x;
					const double dy = pose.y - currPos.y;
					const double dz = pose.z - currPos.z;
					const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
					if (dist < bestDist){
						bestDist = dist;
						chosenIdx = static_cast<int>(i);
					}
					if (dist <= this->endMissionMaxSegmentDistance_){
						chosenIdx = static_cast<int>(i);
					}
				}

				if (chosenIdx < 0){
					nav_msgs::msg::Path refreshedPath;
					this->expPlanner_->setMap(this->map_);
					const bool depReady = this->expPlanner_->makePlan();
					const Eigen::Vector3d homeQuery(0.0, 0.0, this->odom_.pose.pose.position.z);
					const bool refreshed = depReady && this->expPlanner_->getRoadmapPathToPosition(homeQuery, refreshedPath);
					if (refreshed && refreshedPath.poses.size() >= 2){
						this->waypoints_ = refreshedPath;
						this->waypointIdx_ = 1;
						this->newWaypoints_ = true;
						this->replan_ = false;
						RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: End mission roadmap refreshed. Retrying closer waypoint selection.");
						return;
					}

					RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: No valid roadmap waypoint candidate for end mission. Holding and waiting for next retry.");
					this->newWaypoints_ = false;
					this->replan_ = false;
					this->trajectoryReady_ = false;
					this->stop();
					return;
				}

				this->waypointIdx_ = chosenIdx;
				this->goal_ = this->waypoints_.poses[static_cast<size_t>(this->waypointIdx_)];
				this->newWaypoints_ = false;
				this->replan_ = true;
				++this->waypointIdx_;
				RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: End mission: planning to roadmap waypoint idx=%d (segment<=%.2fm target).", chosenIdx, this->endMissionMaxSegmentDistance_);
				return;
			}

			if (this->trajectoryReady_){
				const bool reached_segment_goal = this->isReach(this->goal_, this->reachGoalDistance_, false);
				const double exec_dt = (this->node_->now() - this->trajStartTime_).seconds();
				const double traj_dt = this->trajectory_.getDuration();
				const bool segment_time_done = (traj_dt <= 1e-3) || (exec_dt >= traj_dt + 0.2);
				if (reached_segment_goal || segment_time_done){
					this->trajectoryReady_ = false;
					this->replan_ = false;
					this->stop();
					RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: End mission segment complete. Advancing mission phase.");
				}
				else{
					return;
				}
			}

			if (this->replan_){
				return;
			}

			const auto scheduleEndMissionRetry = [this](const char* phase_msg){
				const double now_sec = this->node_->now().seconds();
				const bool allow_retry =
					(this->endMissionLastRetrySec_ < 0.0) ||
					((now_sec - this->endMissionLastRetrySec_) >= 0.5);
				if (allow_retry){
					this->replan_ = false;
					this->newWaypoints_ = true;
					this->endMissionLastRetrySec_ = now_sec;
					RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: End mission %s plan failed previously. Selecting a closer roadmap waypoint and retrying.", phase_msg);
				}
			};

			if (this->isReach(this->goal_, this->reachGoalDistance_, false)){
				if (this->waypointIdx_ < static_cast<int>(this->waypoints_.poses.size())){
					this->newWaypoints_ = true;
					this->endMissionLastRetrySec_ = -1.0;
					RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: End mission: waypoint reached. Advancing to next roadmap waypoint.");
				}
				else{
					this->stop();
					this->endMissionActive_ = false;
					this->missionEnded_ = true;
					this->landingPhase_ = false;
					this->clearWaypointPlan();
					RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: End mission finished at roadmap node closest to home.");
				}
			}
			else{
				scheduleEndMissionRetry("return-home");
			}
			return;
		}

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
				double rotateYaw = this->waypointRotateYaw_;
				lock.unlock();
				this->moveToOrientation(rotateYaw, this->desiredAngularVel_);
				lock.lock();
				cout << "[AutoFlight]: Finish rotation." << endl;

				// change current goal
				if (this->waypointIdx_ < int(this->waypoints_.poses.size())){
					this->goal_ = this->waypoints_.poses[this->waypointIdx_];
				}
				if (this->waypointIdx_ + 1 > int(this->waypoints_.poses.size())){
					cout << "\033[1;32m[AutoFlight]: Finishing entire path. Wait for new path to replan.\033[0m" << endl;
					this->replan_ = false;
					if (this->replanOnFinishOrFail_){
						this->explorationReplan_ = true;
					}
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

		if (this->replan_){
			return;
		}

		if (this->newWaypoints_){
			if (this->waypoints_.poses.empty()){
				RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: Ignoring new waypoints: insufficient path size (%zu).", this->waypoints_.poses.size());
				this->newWaypoints_ = false;
				this->requestExplorationReplan(this->replanOnCollisionFail_);
				return;
			}
			this->replan_ = false;
			this->trajectoryReady_ = false;
			int nextIdx = -1;
			const auto& currPos = this->odom_.pose.pose.position;
			for (size_t i = this->waypointIdx_; i < this->waypoints_.poses.size(); ++i){
				const auto& pose = this->waypoints_.poses[i].pose.position;
				const double dx = pose.x - currPos.x;
				const double dy = pose.y - currPos.y;
				const double dz = pose.z - currPos.z;
				const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
				if (dist >= this->minWaypointDistance_){
					nextIdx = static_cast<int>(i);
					break;
				}
			}
			if (nextIdx < 0){
				RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: No waypoint beyond min distance %.2f m. Requesting new path.", this->minWaypointDistance_);
				this->newWaypoints_ = false;
				this->requestExplorationReplan(this->replanOnCollisionFail_);
				return;
			}
			this->waypointIdx_ = nextIdx;
			double yaw = atan2(this->waypoints_.poses[static_cast<size_t>(this->waypointIdx_)].pose.position.y - currPos.y,
				this->waypoints_.poses[static_cast<size_t>(this->waypointIdx_)].pose.position.x - currPos.x);
			(void)yaw;
			this->replan_ = true;
			this->newWaypoints_ = false;
			if (this->waypointIdx_ < int(this->waypoints_.poses.size())){
				this->goal_ = this->waypoints_.poses[this->waypointIdx_];
			}
			++this->waypointIdx_;
			cout << "[AutoFlight]: Replan for new waypoints." << endl; 

			return;
		}

		if (this->waypoints_.poses.size() != 0 and this->isReach(this->goal_, this->reachGoalDistance_, false) and this->waypointIdx_ <= int(this->waypoints_.poses.size())){
			this->replan_ = false;
			this->trajectoryReady_ = false;
			geometry_msgs::msg::Quaternion quat = this->goal_.pose.orientation;
			double yaw = AutoFlight::rpy_from_quaternion(quat);
			this->waypointRotateYaw_ = yaw;
			if (this->stabilizeBeforeRotate_){
				cout << "[AutoFlight]: Stabilizing before rotate and replan..." << endl;
				this->waypointRotateReadyTime_ = this->node_->now() + rclcpp::Duration::from_seconds(std::max(0.0, this->wpStablizeTime_));
			}
			else{
				this->waypointRotateReadyTime_ = this->node_->now();
			}
			this->waypointRotatePending_ = true;
			return;		
		}
		else if (this->waypoints_.poses.size() != 0 and this->isReach(this->goal_, this->reachGoalDistance_, true) and (this->replan_ or this->trajectoryReady_)){
			cout << "\033[[AutoFlight]: Finishing entire path. Wait for new path to replan.\033[0m" << endl;
			this->replan_ = false;
			this->trajectoryReady_ = false;
			this->requestExplorationReplan(this->replanOnFinishOrFail_);
			return;		
		}

		if (this->waypoints_.poses.size() != 0){
			if (not this->isGoalValid() and (this->replan_ or this->trajectoryReady_)){
				this->replan_ = false;
				this->trajectoryReady_ = false;
				this->requestExplorationReplan(this->replanOnCollisionFail_);
				cout << "\033[1;32m[AutoFlight]: Current goal is invalid. Need new path to replan.\033[0m" << endl;
				return;
			}
		}

		if (this->trajectoryReady_){
			if (not this->expPlanner_->isPosValid(this->trajectory_.at(this->trajectory_.getDuration()))){
				const Eigen::Vector3d unsafePos = this->trajectory_.at(this->trajectory_.getDuration());
				this->blacklistedGoalPositions_.push_back(unsafePos);
				RCLCPP_WARN(this->node_->get_logger(),
					"[AutoFlight]: Blacklisting unsafe goal at (%.2f, %.2f, %.2f). Will re-run frontier.",
					unsafePos(0), unsafePos(1), unsafePos(2));
				this->trajectoryReady_ = false;
				this->replan_ = false;
				this->stop();
				cout << "\033[1;32m[AutoFlight]: the goal of current local trajectory is not safe. Need replan.\033[0m" << endl;
				this->requestExplorationReplan(this->replanOnFinishOrFail_ || this->replanOnCollisionFail_);
				return;
			}

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

			if (this->computeExecutionDistance() >= 0.3 and this->hasDynamicCollision()){
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
		std::scoped_lock<std::mutex> lock(this->navStateMutex_);
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
		std::scoped_lock<std::mutex> lock(this->navStateMutex_);
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
		//this->takeoff();

		if (this->operatorConfirm_){
			cout << "\033[1;32m[AutoFlight]: Takeoff succeed. Continuing in non-blocking mode (CTRL+C to abort).\033[0m" << endl;
			RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: require_operator_confirmation=true, but post-takeoff flow proceeds non-blocking.");
		}
		else{
			cout << "\033[1;32m[AutoFlight]: Takeoff succeed. Continuing automatically.\033[0m" << endl;
		}

		// int temp1 = system("mkdir -p ~/rosbag_exploration_info &");
		// int temp2 = system("mv ~/rosbag_exploration_info/exploration_info ~/rosbag_exploration_info/previous &");
		// if (temp1==-1 or temp2==-1 or temp3==-1){
		// 	cout << "[AutoFlight]: Recording fails." << endl;
		// }

		if (!this->startExplorationCbGroup_){
			this->startExplorationCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		}
		if (!this->startExplorationSrv_){
			this->startExplorationSrv_ = this->node_->create_service<std_srvs::srv::Trigger>(
				"dynamic_exploration/start",
				[this](
					const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
					std::shared_ptr<std_srvs::srv::Trigger::Response> response){
					{
						std::scoped_lock<std::mutex> lock(this->navStateMutex_);
						if (this->explorationStarted_){
							response->success = false;
							response->message = "exploration already started";
							return;
						}
						this->startExplorationRequested_ = true;
						response->success = true;
						response->message = "exploration start requested";
					}
				},
				rmw_qos_profile_services_default,
				this->startExplorationCbGroup_);
			RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Waiting for /dynamic_exploration/start service trigger to begin planning.");
		}
		if (!this->endMissionSrv_){
			this->endMissionSrv_ = this->node_->create_service<std_srvs::srv::Trigger>(
				"dynamic_exploration/end_mission",
				[this](
					const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
					std::shared_ptr<std_srvs::srv::Trigger::Response> response){
					std::scoped_lock<std::mutex> lock(this->navStateMutex_);
					if (this->missionEnded_){
						response->success = false;
						response->message = "mission already ended";
						return;
					}
					if (this->endMissionActive_ || this->endMissionRequested_){
						response->success = false;
						response->message = "end mission already in progress";
						return;
					}
					this->endMissionRequested_ = true;
					response->success = true;
					response->message = "end mission requested";
				},
				rmw_qos_profile_services_default,
				this->startExplorationCbGroup_);
			RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Service ready: /dynamic_exploration/end_mission");
		}
		if (!this->startExplorationTimer_){
			this->startExplorationTimer_ = this->node_->create_wall_timer(
				std::chrono::milliseconds(50),
				[this](){
					bool shouldStart = false;
					{
						std::scoped_lock<std::mutex> lock(this->navStateMutex_);
						if (this->startExplorationRequested_ && !this->explorationStarted_){
							this->startExplorationRequested_ = false;
							this->explorationStarted_ = true;
							shouldStart = true;
						}
					}
					if (!shouldStart){
						return;
					}
					this->initExplore();
					if (this->operatorConfirm_){
						cout << "\033[1;32m[AutoFlight]: Start planning in non-blocking mode (CTRL+C to abort).\033[0m" << endl;
						RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: require_operator_confirmation=true, but planning start proceeds non-blocking.");
					}
					else{
						cout << "\033[1;32m[AutoFlight]: Start planning.\033[0m" << endl;
					}
					this->registerCallback();
				},
				this->startExplorationCbGroup_);
		}
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
		std::scoped_lock<std::mutex> lock(this->navStateMutex_);
		if (this->endMissionActive_ || this->missionEnded_){
			return;
		}
		// if (!this->explorationReplan_){ // @TODO:check
		// 	return;
		// }
		if (this->newWaypoints_ || this->replan_ || this->waypointRotatePending_){
			return;
		}
		if (this->trajectoryReady_){
			return;
		}
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
			if (this->waypoints_.poses.size() >= 2){
				// Validate endpoint: reject if unsafe or previously blacklisted
				const auto& lastPose = this->waypoints_.poses.back().pose.position;
				const Eigen::Vector3d lastPos(lastPose.x, lastPose.y, lastPose.z);
				bool isBlacklisted = false;
				for (const auto& bl : this->blacklistedGoalPositions_){
					if ((lastPos - bl).norm() < goalBlacklistRadius_){
						isBlacklisted = true;
						break;
					}
				}
				if (!this->expPlanner_->isPosValid(lastPos) || isBlacklisted){
					RCLCPP_WARN(this->node_->get_logger(),
						"[AutoFlight]: DEP goal (%.2f, %.2f, %.2f) is %s. Re-running frontier on next tick.",
						lastPos(0), lastPos(1), lastPos(2),
						isBlacklisted ? "blacklisted" : "unsafe");
					// Do not accept; exploreReplan will fire again in 300 ms
				}
				else{
					const auto& startPose = this->waypoints_.poses.front().pose.position;
					const auto& nextPose = this->waypoints_.poses[1].pose.position;
					const double dx = nextPose.x - startPose.x;
					const double dy = nextPose.y - startPose.y;
					const double dz = nextPose.z - startPose.z;
					const double waypointDist = std::sqrt(dx * dx + dy * dy + dz * dz);
					if (waypointDist >= this->minWaypointDistance_){
						this->blacklistedGoalPositions_.clear();
						this->newWaypoints_ = true;
						this->waypointIdx_ = 1;
						this->explorationReplan_ = false;
					}
					else{
						RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: DEP returned too-short waypoint segment (%.3f m). Retrying.", waypointDist);
					}
				}
			}
			else{
				RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: DEP returned insufficient waypoints (%zu). Retrying.", this->waypoints_.poses.size());
			}
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