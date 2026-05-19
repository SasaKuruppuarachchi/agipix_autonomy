/*
	FILE: flightBase.cpp
	---------------------------
	real world flight implementation
*/
#include <autonomous_flight/px4/flightBase.h>
#include <autonomous_flight/px4/target_qos.h>

namespace AutoFlight{
	flightBase::flightBase(const rclcpp::Node::SharedPtr& node) : node_(node){
    	// parameters 
		

		this->node_->declare_parameter<double>("takeoff_height", 1.0);
		this->node_->get_parameter("takeoff_height", this->takeoffHgt_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Takeoff Height: %.2fm.", this->takeoffHgt_);

		this->node_->declare_parameter<bool>("wait_for_topics_ready", true);
		this->node_->get_parameter("wait_for_topics_ready", this->waitForTopicsReady_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Wait for required startup topics: %s.", this->waitForTopicsReady_ ? "true" : "false");

		this->node_->declare_parameter<double>("takeoff_wait_timeout_sec", 8.0);
		this->node_->get_parameter("takeoff_wait_timeout_sec", this->takeoffWaitTimeoutSec_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Takeoff wait timeout: %.2fs.", this->takeoffWaitTimeoutSec_);

		this->node_->declare_parameter<bool>("require_takeoff_feedback", false);
		this->node_->get_parameter("require_takeoff_feedback", this->requireTakeoffFeedback_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Require takeoff feedback: %s.", this->requireTakeoffFeedback_ ? "true" : "false");

		this->node_->declare_parameter<std::string>("frame_id", "map");
		this->node_->get_parameter("frame_id", this->mapFrameId_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Frame ID: %s.", this->mapFrameId_.c_str());
		
		mapFrameId_ = AutoFlight::getNamespacedFrameId(node_->get_namespace(), this->mapFrameId_);
		baseLinkFrameId_ = AutoFlight::getNamespacedFrameId(node_->get_namespace(), "base_link");

		this->node_->declare_parameter<std::string>("odom_topic", "sensor_measurements/odom");
		this->node_->get_parameter("odom_topic", this->odomTopic_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Odom topic: %s.", this->odomTopic_.c_str());

		this->node_->declare_parameter<std::string>("goal_topic", "goal_pose");
		this->node_->get_parameter("goal_topic", this->goalTopic_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Goal topic: %s.", this->goalTopic_.c_str());

		this->node_->declare_parameter<bool>("subscribe_legacy_goal_topic", true);
		this->node_->get_parameter("subscribe_legacy_goal_topic", this->subscribeLegacyGoalTopic_);
		this->node_->declare_parameter<std::string>("legacy_goal_topic", "move_base_simple/goal");
		this->node_->get_parameter("legacy_goal_topic", this->legacyGoalTopic_);
		if (this->subscribeLegacyGoalTopic_){
			RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Legacy goal topic enabled: %s.", this->legacyGoalTopic_.c_str());
		}

		this->node_->declare_parameter<bool>("skip_takeoff_if_flying", true);
		this->node_->get_parameter("skip_takeoff_if_flying", this->skipTakeoffIfFlying_);
		this->node_->declare_parameter<double>("flying_height_threshold", 0.35);
		this->node_->get_parameter("flying_height_threshold", this->flyingHeightThreshold_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Skip takeoff if flying: %s (|z| >= %.2fm).",
			this->skipTakeoffIfFlying_ ? "true" : "false", this->flyingHeightThreshold_);

		this->node_->declare_parameter<bool>("publish_target_marker", false);
		this->node_->get_parameter("publish_target_marker", this->publishTargetMarker_);
		this->node_->declare_parameter<std::string>("target_marker_topic", "autonomous_flight/target_state_marker");
		this->node_->get_parameter("target_marker_topic", this->targetMarkerTopic_);
		this->node_->declare_parameter<double>("target_marker_scale", 0.50);
		this->node_->get_parameter("target_marker_scale", this->targetMarkerScale_);
		RCLCPP_INFO(
			this->node_->get_logger(),
			"[AutoFlight]: Publish target marker: %s (topic='%s', scale=%.2f).",
			this->publishTargetMarker_ ? "true" : "false",
			this->targetMarkerTopic_.c_str(),
			this->targetMarkerScale_);

		// callback groups
		this->stateCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->odomCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->clickCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->targetPubCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->stateUpdateCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		rclcpp::SubscriptionOptions odomOptions;
		odomOptions.callback_group = this->odomCbGroup_;
		this->odomSub_ = this->node_->create_subscription<nav_msgs::msg::Odometry>(
			this->odomTopic_, rclcpp::SensorDataQoS(), std::bind(&flightBase::odomCB, this, std::placeholders::_1), odomOptions);

		rclcpp::SubscriptionOptions clickOptions;
		clickOptions.callback_group = this->clickCbGroup_;
		this->clickSub_ = this->node_->create_subscription<geometry_msgs::msg::PoseStamped>(
			this->goalTopic_, AutoFlight::buildTargetQos(1, "best_effort", "volatile"), std::bind(&flightBase::clickCB, this, std::placeholders::_1), clickOptions);

		if (this->subscribeLegacyGoalTopic_ && this->legacyGoalTopic_ != this->goalTopic_){
			this->clickSubLegacy_ = this->node_->create_subscription<geometry_msgs::msg::PoseStamped>(
				this->legacyGoalTopic_, AutoFlight::buildTargetQos(1, "best_effort", "volatile"), std::bind(&flightBase::clickCB, this, std::placeholders::_1), clickOptions);
		}
		
		this->node_->declare_parameter<int>("target_qos_depth", 1);
		this->node_->get_parameter("target_qos_depth", this->targetQosDepth_);
		this->node_->declare_parameter<std::string>("target_qos_reliability", "best_effort");
		this->node_->get_parameter("target_qos_reliability", this->targetQosReliability_);
		this->node_->declare_parameter<std::string>("target_qos_durability", "volatile");
		this->node_->get_parameter("target_qos_durability", this->targetQosDurability_);

	    // Publisher
		rclcpp::QoS targetQos = AutoFlight::buildTargetQos(
			this->targetQosDepth_,
			this->targetQosReliability_,
			this->targetQosDurability_);
		this->statePub_ = this->node_->create_publisher<autonomous_flight::msg::Target>(
			"autonomous_flight/target_state",
			targetQos);
		if (this->publishTargetMarker_){
			this->targetMarkerPub_ = this->node_->create_publisher<visualization_msgs::msg::Marker>(this->targetMarkerTopic_, 10);
		}


		// Wait for odometry to be ready
    	this->odomReceived_ = false;
		if (this->waitForTopicsReady_){
			rclcpp::Rate r (10);
			while (rclcpp::ok() && !this->odomReceived_){
				rclcpp::spin_some(this->node_);
				r.sleep();
			}
			RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Odom topic is ready.");
		}
		else{
			RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: Startup topic wait is disabled (wait_for_topics_ready=false).");
		}

		// Target publish timer (replaces detached blocking thread)
		this->targetPubTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(5), std::bind(&flightBase::publishTarget, this), this->targetPubCbGroup_);

		// state update callback (velocity and acceleration)
		this->stateUpdateTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(33), std::bind(&flightBase::stateUpdateCB, this), this->stateUpdateCbGroup_);	
	}

	void flightBase::publishTarget(){
		autonomous_flight::msg::Target target;
		{
			std::scoped_lock<std::mutex> lock(this->targetStateMutex_);
			if (!this->hasStateTarget_.load()) {
				return;
			}
			target = this->stateTgt_;
		}
		this->statePub_->publish(target);
		if (this->publishTargetMarker_ && this->targetMarkerPub_){
			this->publishTargetMarker(target);
		}
	}

	void flightBase::publishTargetMarker(const autonomous_flight::msg::Target& target){
		visualization_msgs::msg::Marker marker;
		marker.header.frame_id = this->mapFrameId_;
		marker.header.stamp = this->node_->now();
		marker.ns = "target_state";
		marker.id = 0;
		marker.type = visualization_msgs::msg::Marker::SPHERE;
		marker.action = visualization_msgs::msg::Marker::ADD;
		marker.pose.position.x = target.position.x;
		marker.pose.position.y = target.position.y;
		marker.pose.position.z = target.position.z;
		marker.pose.orientation.w = 1.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		const double scale = std::max(0.1, this->targetMarkerScale_);
		marker.scale.x = scale;
		marker.scale.y = scale;
		marker.scale.z = scale;
		marker.color.a = 1.0;
		marker.color.r = 0.2;
		marker.color.g = 0.2;
		marker.color.b = 1.0;
		marker.lifetime = rclcpp::Duration::from_seconds(0.15);
		this->targetMarkerPub_->publish(marker);
	}

	void flightBase::odomCB(const nav_msgs::msg::Odometry::SharedPtr odom){
		this->odom_ = *odom;
		this->currPos_(0) = this->odom_.pose.pose.position.x;
		this->currPos_(1) = this->odom_.pose.pose.position.y;
		this->currPos_(2) = this->odom_.pose.pose.position.z;
		if (not this->odomReceived_){
			this->odomReceived_ = true;
		}
	}

	void flightBase::clickCB(const geometry_msgs::msg::PoseStamped::SharedPtr cp){
		this->goal_ = *cp;
		if (std::abs(this->goal_.pose.position.z) < 1e-3){
			this->goal_.pose.position.z = this->takeoffHgt_;
		}
		if (not this->firstGoal_){
			this->firstGoal_ = true;
		}

		if (not this->goalReceived_){
			this->goalReceived_ = true;
		}
		RCLCPP_INFO(
			this->node_->get_logger(),
			"[AutoFlight]: Goal received (frame='%s'): x=%.2f, y=%.2f, z=%.2f",
			this->goal_.header.frame_id.c_str(),
			this->goal_.pose.position.x,
			this->goal_.pose.position.y,
			this->goal_.pose.position.z);
	}

	void flightBase::stateUpdateCB(){
		Eigen::Vector3d currVelBody (this->odom_.twist.twist.linear.x, this->odom_.twist.twist.linear.y, this->odom_.twist.twist.linear.z);
		Eigen::Vector4d orientationQuat (this->odom_.pose.pose.orientation.w, this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y, this->odom_.pose.pose.orientation.z);
		Eigen::Matrix3d orientationRot = AutoFlight::quat2RotMatrix(orientationQuat);
		this->currVel_ = orientationRot * currVelBody;	
		rclcpp::Time currTime = this->node_->now();	
		if (this->stateUpdateFirstTime_){
			this->currAcc_ = Eigen::Vector3d (0.0, 0.0, 0.0);
			this->prevStateTime_ = currTime;
			this->stateUpdateFirstTime_ = false;
		}
		else{
			double dt = (currTime - this->prevStateTime_).seconds();
			this->currAcc_ = (this->currVel_ - this->prevVel_)/dt;
			this->prevVel_ = this->currVel_; 
			this->prevStateTime_ = currTime;
		}
	}

	void flightBase::takeoff(){
		const double current_z = this->odom_.pose.pose.position.z;
		if (this->skipTakeoffIfFlying_ && std::abs(current_z) >= this->flyingHeightThreshold_){
			RCLCPP_WARN(
				this->node_->get_logger(),
				"[AutoFlight]: Skip takeoff because vehicle is already flying (z=%.2f m, threshold=%.2f m).",
				current_z,
				this->flyingHeightThreshold_);
			geometry_msgs::msg::PoseStamped hold_ps;
			hold_ps.header.frame_id = this->mapFrameId_;
			hold_ps.header.stamp = this->node_->now();
			hold_ps.pose = this->odom_.pose.pose;
			this->updateTarget(hold_ps);
			return;
		}

		// from cfg yaml read the flight height
		geometry_msgs::msg::PoseStamped ps;
		ps.header.frame_id = this->mapFrameId_;
		ps.header.stamp = this->node_->now();
		ps.pose.position.x = this->odom_.pose.pose.position.x;
		ps.pose.position.y = this->odom_.pose.pose.position.y;
		ps.pose.position.z = this->takeoffHgt_;
		ps.pose.orientation = this->odom_.pose.pose.orientation;
		this->updateTarget(ps);


		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Start taking off...");
		rclcpp::Rate r (30);
		rclcpp::Time takeoffStart = this->node_->now();
		while (rclcpp::ok() && std::abs(this->odom_.pose.pose.position.z - this->takeoffHgt_) >= 0.1){
			rclcpp::Time now = this->node_->now();
			double elapsed = (now - takeoffStart).seconds();
			if (this->takeoffWaitTimeoutSec_ > 0.0 && elapsed > this->takeoffWaitTimeoutSec_){
				if (this->requireTakeoffFeedback_){
					RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: Takeoff feedback timeout (%.2fs) but require_takeoff_feedback=true, continue waiting.", elapsed);
					takeoffStart = now;
				}
				else{
					RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: No takeoff feedback within %.2fs. Continue mission without blocking.", elapsed);
					break;
				}
			}
			r.sleep();
		}

		// Keep publishing hold-target briefly to settle after takeoff command.
		rclcpp::Time startTime = this->node_->now();
		while (rclcpp::ok()){
			rclcpp::Time currTime = this->node_->now();
			if ((currTime - startTime).seconds() >= 3){
				break;
			}
			r.sleep();
		}
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Takeoff succeed!");
	}

	void flightBase::circle(){
		// circle tracking parameters
		this->node_->declare_parameter<double>("circle_radius", 2.0);
		this->node_->get_parameter("circle_radius", this->radius_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Circle Radius: %.2fm.", this->radius_);

		this->node_->declare_parameter<int>("time_to_max_radius", 30);
		this->node_->get_parameter("time_to_max_radius", this->timeStep_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Time to Maximum Circle Radius: %ds.", this->timeStep_);

		this->node_->declare_parameter<bool>("yaw_control", false);
		this->node_->get_parameter("yaw_control", this->yawControl_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Yaw Control: %s", this->yawControl_ ? "true" : "false");
		
		this->node_->declare_parameter<double>("velocity", 0.5);
		this->node_->get_parameter("velocity", this->velocity_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Velocity: %.2fm/s.", this->velocity_);

        double x = 0;
        double y = 0;
        double z = 0;
        double vx = 0;
        double vy = 0;
        double vz = 0;
        double ax = 0;
        double ay = 0;
        double az = 0;
        double yaw=0;
        double theta = 0;
        double radius = 0;
		double velocity = 0.0;
		int rate = 100;
		rclcpp::Rate r(rate);
		double theta_start;
        double theta_end;
        double step = this->timeStep_*rate;
        int circle = 1;
		int terminate = 0;
		rclcpp::Time startTime = this->node_->now();
		rclcpp::Time endTime1, endTime2, endTime3;
		while(rclcpp::ok() && terminate == 0){
            x = radius * cos(theta);
            y = radius * sin(theta);
            vx = -velocity * sin(theta);
            vy = velocity * cos(theta);
			if (radius > 1e-6){
				ax = - velocity*velocity/radius * cos(theta);
				ay = - velocity*velocity/radius  * sin(theta);
			}
			else{
				ax = 0.0;
				ay = 0.0;
			}
            if (this->yawControl_ == true){
                yaw = theta + PI_const / 2;
            }
            else if (this->yawControl_ == false){
                yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
            }
            z = this->takeoffHgt_;
            vz = 0;
            az = 0;
            
			autonomous_flight::msg::Target target;
            target.position.x = x;
            target.position.y = y;
            target.position.z = z;
            target.velocity.x = vx;
            target.velocity.y = vy;
            target.velocity.z = vz;
			target.acceleration.x = ax;
            target.acceleration.y = ay;
            target.acceleration.z = az;
            target.yaw = yaw;

            this->updateTargetWithState(target);

            if (circle == 1){
                radius += this->radius_/step;
                velocity += this->velocity_/step;
                
                if (std::abs(radius-radius_)<=0.01){
                    theta_start = theta;
					endTime1 = this->node_->now();
                    circle += 1;
                }
            }
            else if (circle == 2){
                radius = this->radius_;
                velocity = this->velocity_;
	            theta_end = 3*2*PI_const;
                if (std::abs((theta-theta_start)- theta_end)<=0.1){
					endTime2 = this->node_->now();
                    circle += 1; 
                }
            }
            else if (circle == 3){
                radius -= this->radius_/step;
                velocity -= this->velocity_/step;
                if (std::abs(radius-0.0)<=0.01){
					endTime3 = this->node_->now();
                    circle += 1;
                }
            }
            else{
                terminate = 1;
                break;
            }
			rclcpp::Time currentTime = this->node_->now();
			double t = (currentTime-startTime).seconds();
			if (radius > 1e-6){
				theta = velocity/radius*t;
			}
			else{
				theta = 0.0;
			}	
            r.sleep();
        }

        if (this->yawControl_==true){
			while (rclcpp::ok() && std::abs(this->odom_.pose.pose.orientation.z-0.0)>=0.01){
                theta += (PI_const*2)/180;
                yaw = theta + PI_const / 2;

				autonomous_flight::msg::Target target;
                target.position.x = x;
                target.position.y = y;
                target.position.z = z;
                target.velocity.x = vx;
                target.velocity.y = vy;
                target.velocity.z = vz;
                target.acceleration.x = ax;
                target.acceleration.y = ay;
                target.acceleration.z = az;
				RCLCPP_INFO(this->node_->get_logger(), "Decreasing Radius...");
                updateTargetWithState(target);
                r.sleep();
            }
        }
    }

	void flightBase::run(){

		// flight test with circle
		double r; // radius
		double v; // circle velocity
    	
    	// track circle radius parameters    	
		this->node_->declare_parameter<double>("radius", 2.0);
		this->node_->get_parameter("radius", r);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Circle radius: %.2fm.", r);

    	// track circle velocity parameters    	
		this->node_->declare_parameter<double>("circle_velocity", 1.0);
		this->node_->get_parameter("circle_velocity", v);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Circle velocity: %.2fm/s.", v);

		double z = this->odom_.pose.pose.position.z;
		geometry_msgs::msg::PoseStamped startPs;
		startPs.pose.position.x = r;
		startPs.pose.position.y = 0.0;
		startPs.pose.position.z = z;
		this->updateTarget(startPs);
		
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Go to target point...");
		rclcpp::Rate rate (30);
		while (rclcpp::ok() && std::abs(this->odom_.pose.pose.position.x - startPs.pose.position.x) >= 0.1){
			rate.sleep();
		}
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Reach target point.");

		rclcpp::Time startTime = this->node_->now();
		while (rclcpp::ok()){
			rclcpp::Time currTime = this->node_->now();
			double t = (currTime - startTime).seconds();
			double rad = v * t / r;
			double x = r * cos(rad);
			double y = r * sin(rad);
			double vx = -v * sin(rad);
			double vy = v * cos(rad);
			double vz = 0.0;
			double aNorm = v*v/r;
			Eigen::Vector3d accVec (x, y, 0);
			accVec = -aNorm * accVec / accVec.norm();
			double ax = accVec(0);
			double ay = accVec(1);
			double az = 0.0;

			// state target message
			autonomous_flight::msg::Target target;
			target.position.x = x;
			target.position.y = y;
			target.position.z = z;
			target.velocity.x = vx;
			target.velocity.y = vy;
			target.velocity.z = vz;
			target.acceleration.x = ax;
			target.acceleration.y = ay;
			target.acceleration.z = az;
			this->updateTargetWithState(target);
			rate.sleep();
		}
	}

	void flightBase::stop(){
		geometry_msgs::msg::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		this->updateTarget(ps);
	}

	void flightBase::moveToOrientation(double yaw, double desiredAngularVel){
		double yawTgt = yaw;
		geometry_msgs::msg::Quaternion orientation = AutoFlight::quaternion_from_rpy(0, 0, yaw);
		double yawCurr = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);		
		geometry_msgs::msg::PoseStamped ps;
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

		double endTime = yawDiffAbs/desiredAngularVel;
		autonomous_flight::msg::Target target;
		geometry_msgs::msg::PoseStamped psT;
		psT.pose = ps.pose;
		rclcpp::Time startTime = this->node_->now();
		rclcpp::Time currTime = this->node_->now();
		rclcpp::Rate r (200);
		while (rclcpp::ok() && !this->isReach(ps)){
			currTime = this->node_->now();
			double t = (currTime - startTime).seconds();

			if (t >= endTime){ 
				psT = ps;
			}
			else{
				double currYawTgt = yawCurr + (double) direction * t/endTime * yawDiffAbs;
				geometry_msgs::msg::Quaternion quatT = AutoFlight::quaternion_from_rpy(0, 0, currYawTgt);
				psT.pose.orientation = quatT;
				
			}
			target.position.x = psT.pose.position.x;
			target.position.y = psT.pose.position.y;
			target.position.z = psT.pose.position.z;
			target.yaw = AutoFlight::rpy_from_quaternion(psT.pose.orientation);
			this->updateTargetWithState(target);
			r.sleep();
		}
	}

	void flightBase::updateTarget(const geometry_msgs::msg::PoseStamped& ps){
		this->poseTgt_ = ps;
		this->poseTgt_.header.frame_id = this->mapFrameId_;
		autonomous_flight::msg::Target target;
		target.position.x = ps.pose.position.x;
		target.position.y = ps.pose.position.y;
		target.position.z = ps.pose.position.z;
		target.velocity.x = 0.0;
		target.velocity.y = 0.0;
		target.velocity.z = 0.0;
		target.acceleration.x = 0.0;
		target.acceleration.y = 0.0;
		target.acceleration.z = 0.0;
		target.yaw = AutoFlight::rpy_from_quaternion(ps.pose.orientation);
		this->updateTargetWithState(target);
	}

	void flightBase::updateTargetWithState(const autonomous_flight::msg::Target& target){
		std::scoped_lock<std::mutex> lock(this->targetStateMutex_);
		this->stateTgt_ = target;
		this->stateTgt_.header.stamp = this->node_->now();
		this->stateTgt_.header.frame_id = this->mapFrameId_;
		this->hasStateTarget_.store(true);
	}
	
	bool flightBase::isReach(const geometry_msgs::msg::PoseStamped& poseTgt, bool useYaw){
		double targetX, targetY, targetZ, targetYaw, currX, currY, currZ, currYaw;
		targetX = poseTgt.pose.position.x;
		targetY = poseTgt.pose.position.y;
		targetZ = poseTgt.pose.position.z;
		targetYaw = AutoFlight::rpy_from_quaternion(poseTgt.pose.orientation);
		currX = this->odom_.pose.pose.position.x;
		currY = this->odom_.pose.pose.position.y;
		currZ = this->odom_.pose.pose.position.z;
		currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		
		bool reachX, reachY, reachZ, reachYaw;
		reachX = std::abs(targetX - currX) < 0.1;
		reachY = std::abs(targetY - currY) < 0.1;
		reachZ = std::abs(targetZ - currZ) < 0.15;
		if (useYaw){
			reachYaw = std::abs(targetYaw - currYaw) < 0.1;
		}
		else{
			reachYaw = true;
		}

		if (reachX and reachY and reachZ and reachYaw){
			return true;
		}
		else{
			return false;
		}
	}

	bool flightBase::isReach(const geometry_msgs::msg::PoseStamped& poseTgt, double dist, bool useYaw){
		double targetX, targetY, targetZ, targetYaw, currX, currY, currZ, currYaw;
		targetX = poseTgt.pose.position.x;
		targetY = poseTgt.pose.position.y;
		targetZ = poseTgt.pose.position.z;
		targetYaw = AutoFlight::rpy_from_quaternion(poseTgt.pose.orientation);
		currX = this->odom_.pose.pose.position.x;
		currY = this->odom_.pose.pose.position.y;
		currZ = this->odom_.pose.pose.position.z;
		currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		
		bool reachX, reachY, reachZ, reachYaw;
		reachX = std::abs(targetX - currX) < dist;
		reachY = std::abs(targetY - currY) < dist;
		reachZ = std::abs(targetZ - currZ) < dist;
		if (useYaw){
			reachYaw = std::abs(targetYaw - currYaw) < 0.1;
		}
		else{
			reachYaw = true;
		}

		// cout << "x: " << std::abs(targetX - currX) << " y: " << std::abs(targetY - currY) << " z: " << std::abs(targetZ - currZ) << endl;
		if (reachX and reachY and reachZ and reachYaw){
			return true;
		}
		else{
			return false;
		}
	}
}