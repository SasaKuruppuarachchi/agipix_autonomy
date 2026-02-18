/*
	FILE: fakeDetector.cpp
	-----------------------
	Function definition of fake detector
*/

#include <onboard_detector/detectors/fakeDetector.h>
#include <chrono>
#include <functional>

namespace onboardDetector{
	fakeDetector::fakeDetector(const rclcpp::Node::SharedPtr& node) : node_(node){
		// load ros parameter:
		this->targetObstacle_ = this->node_->declare_parameter<std::vector<std::string>>(
			"target_obstacle",
			std::vector<std::string>{"person", "obstacle"});
		this->colorDistance_ = this->node_->declare_parameter<double>("color_distance", 5.0);
		std::string odomTopicName = this->node_->declare_parameter<std::string>(
			"odom_topic",
			"/CERLAB/quadcopter/odom");


		this->firstTime_ = true;
		using namespace std::chrono_literals;
		this->gazeboCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->odomCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->visCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		rclcpp::SubscriptionOptions gazeboOpts;
		gazeboOpts.callback_group = this->gazeboCbGroup_;
		this->gazeboSub_ = this->node_->create_subscription<gazebo_msgs::msg::ModelStates>(
			"/gazebo/model_states",
			rclcpp::QoS(10),
			std::bind(&fakeDetector::stateCB, this, std::placeholders::_1),
			gazeboOpts);

		rclcpp::SubscriptionOptions odomOpts;
		odomOpts.callback_group = this->odomCbGroup_;
		this->odomSub_ = this->node_->create_subscription<nav_msgs::msg::Odometry>(
			odomTopicName,
			rclcpp::SensorDataQoS(),
			std::bind(&fakeDetector::odomCB, this, std::placeholders::_1),
			odomOpts);
		// this->odomSub_ = this->nh_.subscribe("/mavros/local_position/odom", 10, &fakeDetector::odomCB, this);
		this->visPub_ = this->node_->create_publisher<visualization_msgs::msg::MarkerArray>(
			"onboard_detector/GT_obstacle_bbox",
			10);
		this->visTimer_ = this->node_->create_wall_timer(
			50ms,
			std::bind(&fakeDetector::visCB, this),
			this->visCbGroup_);
	}


	void fakeDetector::visCB(){
		this->publishVisualization();
	}

	void fakeDetector::stateCB(const gazebo_msgs::msg::ModelStates::SharedPtr allStates){
		bool update = false;
		if (this->firstTime_){
			this->targetIndex_ = this->findTargetIndex(allStates->name);
			this->firstTime_ = false;
		}
		
		std::vector<onboardDetector::box3D> obVec;
		onboardDetector::box3D ob;
		geometry_msgs::msg::Pose p;
		geometry_msgs::msg::Twist tw;
		for (int i : this->targetIndex_){
			std::string name = allStates->name[i];

			// 1. get position and velocity
			p = allStates->pose[i];
			tw = allStates->twist[i];
			ob.x = p.position.x;
			ob.y = p.position.y;
			ob.z = p.position.z;

			if (this->lastObVec_.size() == 0){
				ob.Vx = 0.0;
				ob.Vy = 0.0;
				rclcpp::Time lastTime = this->node_->now();
				this->lastTimeVec_.push_back(lastTime);
				this->lastTimeVel_.push_back(std::vector<double> {0, 0, 0});
				update = true;
			}
	
			else{
				rclcpp::Time currTime = this->node_->now();
				double dT = (currTime - this->lastTimeVec_[i]).seconds();
				if (dT >= 0.1){
					double vx = (ob.x - this->lastObVec_[i].x)/dT;
					double vy = (ob.y - this->lastObVec_[i].y)/dT;
					double vz = (ob.z - this->lastObVec_[i].z)/dT;
					ob.Vx = vx;
					ob.Vy = vy;
					this->lastTimeVel_[i][0] = vx;
					this->lastTimeVel_[i][1] = vy;
					this->lastTimeVel_[i][2] = vz;
					this->lastTimeVec_[i] = this->node_->now();
					update = true;
				}
				else{
					ob.Vx = this->lastTimeVel_[i][0];
					ob.Vy = this->lastTimeVel_[i][1];
				}
			}

			// 2. get size (gazebo name contains size):
			double xsize, ysize, zsize;
			int xsizeStartIdx = name.size() - 1 - 1 - 3 * 3;
			std::string xsizeStr = name.substr(xsizeStartIdx, 3);
			xsize = std::stod(xsizeStr);

			int ysizeStartIdx = name.size() - 1 - 3 * 2;
			std::string ysizeStr = name.substr(ysizeStartIdx, 3);
			ysize = std::stod(ysizeStr);

			int zsizeStartIdx = name.size() - 3;
			std::string zsizeStr = name.substr(zsizeStartIdx, 3);
			zsize = std::stod(zsizeStr);
			
			ob.x_width = xsize;
			ob.y_width = ysize;
			ob.z_width = zsize;
			obVec.push_back(ob);
		}
		if (update){
			this->lastObVec_ = obVec;
		}
		this->obstacleMsg_ = obVec;
		// ros::Rate r (60);
		// r.sleep();
	}

	void fakeDetector::odomCB(const nav_msgs::msg::Odometry::SharedPtr odom){
		this->odom_ = *odom;
	}

	std::vector<int>& fakeDetector::findTargetIndex(const std::vector<std::string>& modelNames){
		static std::vector<int> targetIndex;
		int countID = 0;
		for (std::string name : modelNames){
			for (std::string targetName : this->targetObstacle_){
				if (name.compare(0, targetName.size(), targetName) == 0){
					targetIndex.push_back(countID);
				}
			}
			++countID;
		}
		return targetIndex;
	}

	void fakeDetector::updateVisMsg(){
		std::vector<visualization_msgs::msg::Marker> bboxVec;
		int obIdx = 0;
		for (onboardDetector:: box3D obstacle : this->obstacleMsg_){

			// 12 lines for each obstacle
			geometry_msgs::msg::Point p1, p2, p3, p4, p5, p6, p7, p8;
			// upper four points
			p1.x = obstacle.x+obstacle.x_width/2; p1.y = obstacle.y+obstacle.y_width/2; p1.z = obstacle.z+obstacle.z_width;
			p2.x = obstacle.x-obstacle.x_width/2; p2.y = obstacle.y+obstacle.y_width/2; p2.z = obstacle.z+obstacle.z_width;
			p3.x = obstacle.x+obstacle.x_width/2; p3.y = obstacle.y-obstacle.y_width/2; p3.z = obstacle.z+obstacle.z_width;
			p4.x = obstacle.x-obstacle.x_width/2; p4.y = obstacle.y-obstacle.y_width/2; p4.z = obstacle.z+obstacle.z_width;

			p5.x = obstacle.x+obstacle.x_width/2; p5.y = obstacle.y+obstacle.y_width/2; p5.z = obstacle.z;
			p6.x = obstacle.x-obstacle.x_width/2; p6.y = obstacle.y+obstacle.y_width/2; p6.z = obstacle.z;
			p7.x = obstacle.x+obstacle.x_width/2; p7.y = obstacle.y-obstacle.y_width/2; p7.z = obstacle.z;
			p8.x = obstacle.x-obstacle.x_width/2; p8.y = obstacle.y-obstacle.y_width/2; p8.z = obstacle.z;

			std::vector<geometry_msgs::msg::Point> line1Vec {p1, p2};
			std::vector<geometry_msgs::msg::Point> line2Vec {p1, p3};
			std::vector<geometry_msgs::msg::Point> line3Vec {p2, p4};
			std::vector<geometry_msgs::msg::Point> line4Vec {p3, p4};
			std::vector<geometry_msgs::msg::Point> line5Vec {p1, p5};
			std::vector<geometry_msgs::msg::Point> line6Vec {p2, p6};
			std::vector<geometry_msgs::msg::Point> line7Vec {p3, p7};
			std::vector<geometry_msgs::msg::Point> line8Vec {p4, p8};
			std::vector<geometry_msgs::msg::Point> line9Vec {p5, p6};
			std::vector<geometry_msgs::msg::Point> line10Vec {p5, p7};
			std::vector<geometry_msgs::msg::Point> line11Vec {p6, p8};
			std::vector<geometry_msgs::msg::Point> line12Vec {p7, p8};

			std::vector<std::vector<geometry_msgs::msg::Point>> allLines{
				line1Vec,
				line2Vec,
				line3Vec,
				line4Vec,
				line5Vec,
				line6Vec,
				line7Vec,
				line8Vec,
				line9Vec,
				line10Vec,
				line11Vec,
				line12Vec
			};

			int count = 0;
			std::string name = "GT osbtacles" + std::to_string(obIdx);
			for (std::vector<geometry_msgs::msg::Point> lineVec: allLines){
				visualization_msgs::msg::Marker line;

				line.header.frame_id = "drone0/map";
				line.ns = name;
				line.points = lineVec;
				line.id = count;
				line.type = visualization_msgs::msg::Marker::LINE_LIST;
				line.lifetime = rclcpp::Duration::from_seconds(0.5).to_msg();
				line.scale.x = 0.05;
				line.scale.y = 0.05;
				line.scale.z = 0.05;
				line.color.a = 1.0;
				if (this->isObstacleInSensorRange(obstacle, PI_const)){
					line.color.r = 1;
					line.color.g = 0;
					line.color.b = 0;
				}
				else{
					line.color.r = 0;
					line.color.g = 1;
					line.color.b = 0;
				}
				++count;
				bboxVec.push_back(line);
			}
			++obIdx;
		}
		this->visMsg_.markers = bboxVec;
	}


	void fakeDetector::publishVisualization(){
		this->updateVisMsg();
		this->visPub_->publish(this->visMsg_);
	}

	bool fakeDetector::isObstacleInSensorRange(const onboardDetector::box3D& ob, double fov){
		Eigen::Vector3d pRobot (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d pObstacle (ob.x, ob.y, ob.z);	
		
		Eigen::Vector3d diff = pObstacle - pRobot;
		diff(2) = 0.0;
		double distance = diff.norm();
		double yaw = rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d direction (cos(yaw), sin(yaw), 0);

		double angle = angleBetweenVectors(direction, diff);
		if (angle <= fov/2 and distance <= this->colorDistance_){
			return true;
		}
		else{
			return false;
		}
	
	}

	void fakeDetector::getObstacles(std::vector<onboardDetector::box3D>& obstacles){
		obstacles = this->obstacleMsg_;
	}

	void fakeDetector::getObstaclesInSensorRange(double fov, std::vector<onboardDetector::box3D>& obstacles){
		obstacles.clear();
		for (onboardDetector::box3D obstacle : this->obstacleMsg_){
			if (this->isObstacleInSensorRange(obstacle, fov)){
				obstacles.push_back(obstacle);
			}
		}
	}
}