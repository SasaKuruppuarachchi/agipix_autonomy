/*
	FILE: flightBase.cpp
	---------------------------
	real world flight implementation
*/
#include <autonomous_flight/px4/flightBase.h>

namespace AutoFlight{
	flightBase::flightBase(const rclcpp::Node::SharedPtr& node) : node_(node){
    	// parameters    	
		this->node_->declare_parameter<double>("takeoff_height", 1.0);
		this->node_->get_parameter("takeoff_height", this->takeoffHgt_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Takeoff Height: %.2fm.", this->takeoffHgt_);

		this->node_->declare_parameter<bool>("wait_for_topics_ready", true);
		this->node_->get_parameter("wait_for_topics_ready", this->waitForTopicsReady_);
		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Wait for MAVROS/odom topics at startup: %s.", this->waitForTopicsReady_ ? "true" : "false");

		// callback groups
		this->stateCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->odomCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->clickCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->stateUpdateCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		// Subscriber
		rclcpp::SubscriptionOptions stateOptions;
		stateOptions.callback_group = this->stateCbGroup_;
		this->stateSub_ = this->node_->create_subscription<mavros_msgs::msg::State>(
			"/mavros/state", rclcpp::QoS(1000), std::bind(&flightBase::stateCB, this, std::placeholders::_1), stateOptions);

		rclcpp::SubscriptionOptions odomOptions;
		odomOptions.callback_group = this->odomCbGroup_;
		this->odomSub_ = this->node_->create_subscription<nav_msgs::msg::Odometry>(
			"/mavros/local_position/odom", rclcpp::QoS(1000), std::bind(&flightBase::odomCB, this, std::placeholders::_1), odomOptions);

		rclcpp::SubscriptionOptions clickOptions;
		clickOptions.callback_group = this->clickCbGroup_;
		this->clickSub_ = this->node_->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/move_base_simple/goal", rclcpp::QoS(1000), std::bind(&flightBase::clickCB, this, std::placeholders::_1), clickOptions);
		
		// Service client
		this->armClient_ = this->node_->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
		this->setModeClient_ = this->node_->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");	

    	// Publisher
		this->posePub_ = this->node_->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 1000);
		this->statePub_ = this->node_->create_publisher<tracking_controller::msg::Target>("/autonomous_flight/target_state", 1000);


		// Wait for odometry and mavros to be ready
    	this->odomReceived_ = false;
    	this->mavrosStateReceived_ = false;
		if (this->waitForTopicsReady_){
			rclcpp::Rate r (10);
			while (rclcpp::ok() && !(this->odomReceived_ && this->mavrosStateReceived_)){
				rclcpp::spin_some(this->node_);
				r.sleep();
			}
			RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Odom and mavros topics are ready.");
		}
		else{
			RCLCPP_WARN(this->node_->get_logger(), "[AutoFlight]: Startup topic wait is disabled (wait_for_topics_ready=false).");
		}


    	// Tareget publish thread
		this->targetPubWorker_ = std::thread(&flightBase::publishTarget, this);
		this->targetPubWorker_.detach();

		// state update callback (velocity and acceleration)
		this->stateUpdateTimer_ = this->node_->create_wall_timer(
			std::chrono::milliseconds(33), std::bind(&flightBase::stateUpdateCB, this), this->stateUpdateCbGroup_);	
	}

	void flightBase::publishTarget(){
		rclcpp::Rate r (200);

		// warmup
		for(int i = 100; rclcpp::ok() && i > 0; --i){
	        this->poseTgt_.header.stamp = this->node_->now();
	        this->posePub_->publish(this->poseTgt_);
    	}

		auto offboardMode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
		offboardMode->custom_mode = "OFFBOARD";
		auto armCmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
		armCmd->value = true;
		rclcpp::Time lastRequest = this->node_->now();
		while (rclcpp::ok()){
			if (this->mavrosState_.mode != "OFFBOARD" && (this->node_->now() - lastRequest > rclcpp::Duration::from_seconds(5.0))){
				if (this->setModeClient_->service_is_ready()){
					this->setModeClient_->async_send_request(offboardMode,
						[this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future){
							if (future.get()->mode_sent){
								RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Offboard mode enabled.");
							}
						});
				}
				lastRequest = this->node_->now();
			} else {
				if (!this->mavrosState_.armed && (this->node_->now() - lastRequest > rclcpp::Duration::from_seconds(5.0))){
					if (this->armClient_->service_is_ready()){
						this->armClient_->async_send_request(armCmd,
							[this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future){
								if (future.get()->success){
									RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Vehicle armed.");
								}
							});
					}
					lastRequest = this->node_->now();
				}
			}

			if (this->poseControl_){
	        	// this->poseTgt_.header.stamp = this->node_->now();
	        	this->posePub_->publish(this->poseTgt_);
	        }
	        else{
				this->statePub_->publish(this->stateTgt_);
			}
			// ros::spinOnce();
			r.sleep();
		}	
	}

	void flightBase::stateCB(const mavros_msgs::msg::State::SharedPtr state){
		this->mavrosState_ = *state;
		if (not this->mavrosStateReceived_){
			this->mavrosStateReceived_ = true;
		}
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
		this->goal_.pose.position.z = 1.0;
		if (not this->firstGoal_){
			this->firstGoal_ = true;
		}

		if (not this->goalReceived_){
			this->goalReceived_ = true;
		}
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
		// from cfg yaml read the flight height
		geometry_msgs::msg::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = this->node_->now();
		ps.pose.position.x = this->odom_.pose.pose.position.x;
		ps.pose.position.y = this->odom_.pose.pose.position.y;
		ps.pose.position.z = this->takeoffHgt_;
		ps.pose.orientation = this->odom_.pose.pose.orientation;
		this->updateTarget(ps);


		RCLCPP_INFO(this->node_->get_logger(), "[AutoFlight]: Start taking off...");
		rclcpp::Rate r (30);
		while (rclcpp::ok() && std::abs(this->odom_.pose.pose.position.z - this->takeoffHgt_) >= 0.1){
			rclcpp::spin_some(this->node_);
			r.sleep();
		}

		// tracking_controller::Target psT;
		// // psT.type_mask = psT.IGNORE_ACC_VEL;
		// psT.header.frame_id = "map";
		// psT.header.stamp = ros::Time::now();
		// psT.position.x = this->odom_.pose.pose.position.x;
		// psT.position.y = this->odom_.pose.pose.position.y;
		// psT.position.z = this->takeoffHgt_;
		// psT.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		// this->updateTargetWithState(psT);
		
		// cout << "[AutoFlight]: Switch to tracking controller." << endl;
		rclcpp::Time startTime = this->node_->now();
		while (rclcpp::ok()){
			rclcpp::Time currTime = this->node_->now();
			if ((currTime - startTime).seconds() >= 3){
				break;
			}
			rclcpp::spin_some(this->node_);
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
            
			tracking_controller::msg::Target target;
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
			rclcpp::spin_some(this->node_);
            r.sleep();
        }

        if (this->yawControl_==true){
			while (rclcpp::ok() && std::abs(this->odom_.pose.pose.orientation.z-0.0)>=0.01){
                theta += (PI_const*2)/180;
                yaw = theta + PI_const / 2;

				tracking_controller::msg::Target target;
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
				rclcpp::spin_some(this->node_);
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
			rclcpp::spin_some(this->node_);
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
			tracking_controller::msg::Target target;
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
			rclcpp::spin_some(this->node_);
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
		tracking_controller::msg::Target target;
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
			// this->updateTarget(psT);
			target.position.x = psT.pose.position.x;
			target.position.y = psT.pose.position.y;
			target.position.z = psT.pose.position.z;
			target.yaw = AutoFlight::rpy_from_quaternion(psT.pose.orientation);
			this->updateTargetWithState(target);
			// cout << "here" << endl;
			rclcpp::spin_some(this->node_);
			r.sleep();
		}
	}

	void flightBase::updateTarget(const geometry_msgs::msg::PoseStamped& ps){
		this->poseTgt_ = ps;
		this->poseTgt_.header.frame_id = "map";
		this->poseControl_ = true;
	}

	void flightBase::updateTargetWithState(const tracking_controller::msg::Target& target){
		this->stateTgt_ = target;
		this->poseControl_ = false;
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