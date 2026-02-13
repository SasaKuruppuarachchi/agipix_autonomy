/*
*	File: rrtOctomap.h
*	---------------
*   RRT planner class based on Octomap.
*/
#ifndef RRTOCTOMAP_H
#define RRTOCTOMAP_H
#include <rclcpp/rclcpp.hpp>
#include <global_planner/rrtBase.h>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/msg/path.hpp>
#include <limits>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <mutex>
#include <global_planner/ros2_utils.h>


namespace globalPlanner{
	template <std::size_t N>
	class rrtOctomap : public rrtBase<N>{
	protected:
		rclcpp::Node::SharedPtr node_;
		rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr mapClient_;
		rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr mapSub_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr RRTVisPub_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pathVisPub_;
		rclcpp::CallbackGroup::SharedPtr mapCbGroup_;

		double mapRes_;
		double envLimit_[6];
		double sampleRegion_[6];
		std::shared_ptr<octomap::OcTree> map_ {NULL};
		
		visualization_msgs::msg::MarkerArray RRTVisMsg_;
		visualization_msgs::msg::MarkerArray pathVisMsg_;
		std::vector<visualization_msgs::msg::Marker> RRTVisvec_; // we update this
		std::vector<visualization_msgs::msg::Marker> pathVisVec_; // update this
		bool visRRT_;
		bool visPath_;
	
		bool ignoreUnknown_;
		bool notUpdateSampleRegion_;
		double maxShortcutThresh_;

	public:
		std::thread RRTVisWorker_;
		std::thread pathVisWorker_;

		// default constructor
		rrtOctomap();

		// constructor using ros param:
		rrtOctomap(const rclcpp::Node::SharedPtr &node);

		// constructor using point format
		rrtOctomap(const rclcpp::Node::SharedPtr &node, KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ=0.3, double dR=0.2, double connectGoalRatio=0.10, double timeout=1.0, bool visRRT=false, bool visPath=true);

		// constructor using vector format
		rrtOctomap(const rclcpp::Node::SharedPtr &node, std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ=0.3, double dR=0.2, double connectGoalRatio=0.10, double timeout=1.0, bool visRRT=false, bool visPath=true);

		// constructor without start and goal
		rrtOctomap(const rclcpp::Node::SharedPtr &node, std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ=0.3, double dR=0.2, double connectGoalRatio=0.10, double timeout=1.0, bool visRRT=false, bool visPath=true);
		
		// constructor without nh	
		rrtOctomap(std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ=0.3, double dR=0.2, double connectGoalRatio=0.10, double timeout=1.0, bool visRRT=false, bool visPath=true);


		// update octomap
		virtual void updateMap(); // update map use service
		void mapCB(const octomap_msgs::msg::Octomap::SharedPtr msg);
		void updateSampleRegion();// helper function for update sample region
		void updateEnvBox(const std::vector<double>& range);
		void clearEnvBox();
		
		// collision checking function based on map and collision box: TRUE => Collision
		virtual bool checkCollision(const KDTree::Point<N>& q);
		bool checkCollision(const octomap::point3d& p);
		bool checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown=true);
		bool checkCollisionLine(const KDTree::Point<N>& q1, const KDTree::Point<N>& q2);
		bool checkCollisionLine(const octomap::point3d& p1, const octomap::point3d& p2);

		// shortcut path
		void shortcutWaypointPaths(const std::vector<KDTree::Point<N>>& plan, std::vector<KDTree::Point<N>>& planSc);
		void shortcutWaypointPathsRandom(const std::vector<KDTree::Point<N>>& plan, std::vector<KDTree::Point<N>>& planSc);
		void randomShortcutOnce(std::vector<KDTree::Point<N>>& plan);
		void sampleWaypoint(const std::vector<KDTree::Point<N>>& plan, KDTree::Point<N>& sample, int& idx);

		// random sample in valid space (based on current map)
		virtual void randomConfig(KDTree::Point<N>& qRand);

		// *** Core function: make plan based on all input ***
		virtual void makePlan(std::vector<KDTree::Point<N>>& plan);
		virtual void makePlan(nav_msgs::msg::Path& plan);

		// Visualization
		void startVisModule();
		void publishRRTVisMsg();
		void publishPathVisMsg();
		void updateRRTVisVec(const KDTree::Point<N>& qNew, const KDTree::Point<N>& qNear, int id);
		void updatePathVisVec(const std::vector<KDTree::Point<N>> &plan);

		// Helper function for collision checking:
		void point2Octomap(const KDTree::Point<N>& q, octomap::point3d& p);

		// Helper function: convert to ROS message
		void pathMsgConverter(const std::vector<KDTree::Point<N>>& pathTemp, nav_msgs::msg::Path& path);


		double getMapRes();
		std::vector<double> getEnvBox();
	};


	// ===========================Function Definition=======================================
	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap() : rrtBase<N>(){}

	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(const rclcpp::Node::SharedPtr &node) : rrtBase<N>(), node_(node){
		// load parameters:
		// Map Resolution for Collisiong checking
			this->mapRes_ = declareAndGetParam<double>(this->node_, "map_resolution", 0.2);

		// Visualize RRT 
		this->visRRT_ = declareAndGetParam<bool>(this->node_, "vis_RRT", false);

		// Visualize Path
		this->visPath_ = declareAndGetParam<bool>(this->node_, "vis_path", true);

		// Collision Box
		this->collisionBox_ = declareAndGetParam<std::vector<double>>(this->node_, "collision_box", {1.0, 1.0, 0.6});

		// Environment Size (maximum)
		this->envBox_ = declareAndGetParam<std::vector<double>>(this->node_, "env_box", {-100, 100, -100, 100, 0, 1.5});

		// Incremental Distance (For RRT)
		this->delQ_ = declareAndGetParam<double>(this->node_, "rrt_incremental_distance", 0.3);

		// Goal Reach Distance
		this->dR_ = declareAndGetParam<double>(this->node_, "goal_reach_distance", 0.4);

		// RRT Connect Goal Ratio
		this->connectGoalRatio_ = declareAndGetParam<double>(this->node_, "rrt_connect_goal_ratio", 0.2);

		// Time out:
		this->timeout_ = declareAndGetParam<double>(this->node_, "timeout", 2.0);

		// ignore unknown voxel
		this->ignoreUnknown_ = declareAndGetParam<bool>(this->node_, "ignore_unknown", false);

		// maximum shortcut threshold
		this->maxShortcutThresh_ = declareAndGetParam<double>(this->node_, "max_shortcut_dist", 5.0);


		this->mapClient_ = this->node_->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");
		this->mapCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		rclcpp::SubscriptionOptions mapOptions;
		mapOptions.callback_group = this->mapCbGroup_;
		this->mapSub_ = this->node_->create_subscription<octomap_msgs::msg::Octomap>(
			"/octomap_full", 1, std::bind(&rrtOctomap::mapCB, this, std::placeholders::_1), mapOptions);

		rclcpp::Rate r(10);
		while (rclcpp::ok() and this->map_ == NULL){
			cout << "[RRTPlanner]: Wait for Map..." << endl;
			r.sleep();
		}
		cout << "[RRTPlanner]: Map Updated!" << endl;

		// Visualization:
		this->startVisModule();
		
		this->notUpdateSampleRegion_ = false;
	}

	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(const rclcpp::Node::SharedPtr &node, KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ, double dR, double connectGoalRatio, double timeout, bool visRRT, bool visPath)
	: node_(node), mapRes_(mapRes), visRRT_(visRRT), visPath_(visPath), rrtBase<N>(start, goal, collisionBox, envBox, delQ, dR, connectGoalRatio){
			this->mapClient_ = this->node_->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");
		// this->updateMap();

		// Visualization:
		this->startVisModule();
	}

	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(const rclcpp::Node::SharedPtr &node, std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ, double dR, double connectGoalRatio, double timeout, bool visRRT, bool visPath)
	: node_(node), mapRes_(mapRes), visRRT_(visRRT), visPath_(visPath), rrtBase<N>(start, goal, collisionBox, envBox, delQ, dR, connectGoalRatio, timeout){
			this->mapClient_ = this->node_->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");
		// this->updateMap();

		// Visualization:
		this->startVisModule();
	}

	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(const rclcpp::Node::SharedPtr &node, std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ, double dR, double connectGoalRatio, double timeout, bool visRRT, bool visPath)
	:  node_(node), mapRes_(mapRes), visRRT_(visRRT), visPath_(visPath), rrtBase<N>(collisionBox, envBox, delQ, dR, connectGoalRatio, timeout){
			this->mapClient_ = this->node_->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");
		// this->updateMap();
		
		// Visualization:
		this->startVisModule();
	}
	
	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ, double dR, double connectGoalRatio, double timeout, bool visRRT, bool visPath)
	: mapRes_(mapRes), visRRT_(visRRT), visPath_(visPath), rrtBase<N>(collisionBox, envBox, delQ, dR, connectGoalRatio, timeout){}




	template <std::size_t N>
	void rrtOctomap<N>::updateMap(){
		auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
		rclcpp::Rate rate(10);
		while (rclcpp::ok()){
			if (!this->mapClient_->wait_for_service(std::chrono::seconds(1))){
				RCLCPP_INFO(this->node_->get_logger(), "[RRTPlanner]: Wait for Octomap Service...");
				rate.sleep();
				continue;
			}
			auto future = this->mapClient_->async_send_request(request);
			if (rclcpp::spin_until_future_complete(this->node_, future) == rclcpp::FutureReturnCode::SUCCESS){
				auto response = future.get();
				octomap::AbstractOcTree* abtree = octomap_msgs::binaryMsgToMap(response->map);
				this->map_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(abtree));
				break;
			}
			RCLCPP_INFO(this->node_->get_logger(), "[RRTPlanner]: Wait for Octomap Service...");
			rate.sleep();
		}
		// this->map_->setResolution(this->mapRes_);
		double min_x, max_x, min_y, max_y, min_z, max_z;
		this->map_->getMetricMax(max_x, max_y, max_z);
		this->map_->getMetricMin(min_x, min_y, min_z);
		this->envLimit_[0] = min_x; this->envLimit_[1] = max_x; this->envLimit_[2] = min_y; this->envLimit_[3] = max_y; this->envLimit_[4] = min_z; this->envLimit_[5] = max_z;
		// this->updateSampleRegion();
		cout << "[RRTPlanner]: Map updated!" << endl;
	}

	template <std::size_t N>
	void rrtOctomap<N>::mapCB(const octomap_msgs::msg::Octomap::SharedPtr msg){
		octomap::OcTree* treePtr = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
		if (treePtr == nullptr){
			cout << "[RRTPlanner]: Failed to convert Octomap message (null tree)." << endl;
			return;
		}
	   	this->map_ = std::shared_ptr<octomap::OcTree>(treePtr);		
   	 	double min_x, max_x, min_y, max_y, min_z, max_z;
		this->map_->getMetricMax(max_x, max_y, max_z);
		this->map_->getMetricMin(min_x, min_y, min_z);
		this->envLimit_[0] = min_x; this->envLimit_[1] = max_x; this->envLimit_[2] = min_y; this->envLimit_[3] = max_y; this->envLimit_[4] = min_z; this->envLimit_[5] = max_z;
		// this->updateSampleRegion();
	}

	template <std::size_t N>
	void rrtOctomap<N>::updateSampleRegion(){
		double xmin = std::max(this->envBox_[0], this->envLimit_[0]); this->sampleRegion_[0] = xmin;
		double xmax = std::min(this->envBox_[1], this->envLimit_[1]); this->sampleRegion_[1] = xmax;
		double ymin = std::max(this->envBox_[2], this->envLimit_[2]); this->sampleRegion_[2] = ymin;
		double ymax = std::min(this->envBox_[3], this->envLimit_[3]); this->sampleRegion_[3] = ymax;
		double zmin = std::max(this->envBox_[4], this->envLimit_[4]); this->sampleRegion_[4] = zmin;
		double zmax = std::min(this->envBox_[5], this->envLimit_[5]); this->sampleRegion_[5] = zmax;
	}

	template <std::size_t N>
	void rrtOctomap<N>::updateEnvBox(const std::vector<double>& range){
		this->envBox_ = range;
		this->notUpdateSampleRegion_ = true;
	}

	template <std::size_t N>
	void rrtOctomap<N>::clearEnvBox(){
		this->envBox_ = declareAndGetParam<std::vector<double>>(this->node_, "env_box", {-100, 100, -100, 100, 0, 1.5});
		this->notUpdateSampleRegion_ = false;
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollision(const KDTree::Point<N>& q){
		octomap::point3d p;
		this->point2Octomap(q, p);
		return this->checkCollision(p);
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollision(const octomap::point3d& p){
		double xmin, xmax, ymin, ymax, zmin, zmax; // bounding box for collision checking
		xmin = p.x() - this->collisionBox_[0]/2; xmax = p.x() + this->collisionBox_[0]/2;
		ymin = p.y() - this->collisionBox_[1]/2; ymax = p.y() + this->collisionBox_[1]/2;
		zmin = p.z() - this->collisionBox_[2]/2; zmax = p.z() + this->collisionBox_[2]/2;

		int xNum = (xmax - xmin)/this->mapRes_;
		int yNum = (ymax - ymin)/this->mapRes_;
		int zNum = (zmax - zmin)/this->mapRes_;

		int xID, yID, zID;
		for (xID=0; xID<=xNum; ++xID){
			for (yID=0; yID<=yNum; ++yID){
				for (zID=0; zID<=zNum; ++zID){
					if (this->checkCollisionPoint(octomap::point3d(xmin+xID*this->mapRes_, ymin+yID*this->mapRes_, zmin+zID*this->mapRes_), this->ignoreUnknown_)){
						return true;
					}
				}
			}
		}
		return false;
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown){
		octomap::OcTreeNode* nptr = this->map_->search(p);
		if (nptr == NULL){
			if (not ignoreUnknown){
				return true;
			}
			else{
				return false;
			}
		}
		return this->map_->isNodeOccupied(nptr);
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollisionLine(const KDTree::Point<N>& q1, const KDTree::Point<N>& q2){
		octomap::point3d p1, p2;
		this->point2Octomap(q1, p1);
		this->point2Octomap(q2, p2);
		return this->checkCollisionLine(p1, p2);
	}
	
	template <std::size_t N>
	bool rrtOctomap<N>::checkCollisionLine(const octomap::point3d& p1, const octomap::point3d& p2){
		std::vector<octomap::point3d> ray;
		this->map_->computeRay(p1, p2, ray);
		if (this->checkCollision(p2)){
			return true;
		}

		for (octomap::point3d p: ray){
			if (this->checkCollision(p)){
				return true;
			}
		}
		return false;
	}

	template <std::size_t N>
	void rrtOctomap<N>::shortcutWaypointPaths(const std::vector<KDTree::Point<N>>& plan, std::vector<KDTree::Point<N>>& planSc){
		size_t ptr1 = 0; size_t ptr2 = 2;
		planSc.push_back(plan[ptr1]);

		if (plan.size() == 1){
			return;
		}

		if (plan.size() == 2){
			planSc.push_back(plan[1]);
			return;
		}

		while (rclcpp::ok()){
			// cout << "here" << endl;
			// cout << "total size: " << plan.size() << endl;
			if (ptr2 > plan.size()-1){
				// cout << "error" << endl;
				break;
			}
			KDTree::Point<N> p1 = plan[ptr1]; KDTree::Point<N> p2 = plan[ptr2];
			if (not checkCollisionLine(p1, p2) and KDTree:: Distance(p1, p2) <= this->maxShortcutThresh_){
				// cout << "has collision" << endl;
				if (ptr2 >= plan.size()-1){
					planSc.push_back(p2);
					break;
				}
				++ptr2;
			}
			else{
				// cout << "no collision" << endl;
				// cout << "ptr1: " << ptr1 << ", ptr2: " << ptr2 << endl;
				// cout << plan.size() << endl;
				planSc.push_back(plan[ptr2-1]);
				ptr1 = ptr2-1;
				ptr2 = ptr1+2;
			}
		}
	}

	template <std::size_t N>
	void rrtOctomap<N>::shortcutWaypointPathsRandom(const std::vector<KDTree::Point<N>>& plan, std::vector<KDTree::Point<N>>& planSc){
		planSc = plan;
		double timeoutSC = 0.1;
		rclcpp::Time startTime = this->node_->now();
		while (rclcpp::ok()){
			rclcpp::Time endTime = this->node_->now();
			if ((endTime - startTime).seconds() >= timeoutSC){
				break;
			}
			this->randomShortcutOnce(planSc);
		}
	}


	template <std::size_t N>
	void rrtOctomap<N>::randomShortcutOnce(std::vector<KDTree::Point<N>>& plan){
		KDTree::Point<N> p1, p2;
		int idx1, idx2;
		this->sampleWaypoint(plan, p1, idx1);
		this->sampleWaypoint(plan, p2, idx2);

		// if two points has no collision in between shorcut them
		if (idx1 != idx2 and not this->checkCollisionLine(p1, p2)){
			std::vector<KDTree::Point<N>> oldPlan = plan;
			plan.clear();
			for (int i=0; i<oldPlan.size(); ++i){
				if (idx1 > idx2){
					if (i <= idx2 or i >= idx1){
						plan.push_back(oldPlan[i]);
					}
					// else if (i == idx2){
					// 	plan.push_back(oldPlan[idx2]);
					// 	plan.push_back(p2);
					// 	plan.push_back(p1);
					// }
				}
				else{
					if (i <= idx1 or i >= idx2){
						plan.push_back(oldPlan[i]);
					}
					// else if (i == idx1){
					// 	plan.push_back(oldPlan[idx1]);
					// 	plan.push_back(p1);
					// 	plan.push_back(p2);
					// }
				}
			}
		}

	}

	template <std::size_t N>
	void rrtOctomap<N>::sampleWaypoint(const std::vector<KDTree::Point<N>>& plan, KDTree::Point<N>& sample, int& idx){
		idx = rand() % (plan.size());
		// double ratio = randomNumber(0, 1);
		KDTree::Point<N> p1 = plan[idx];
		// KDTree::Point<N> p2 = plan[idx+1];
		// sample = p1 * ratio + p2 * (1 - ratio);
		sample = p1;
	}




	template <std::size_t N>
	void rrtOctomap<N>::randomConfig(KDTree::Point<N>& qRand){
		bool valid = false;
		octomap::point3d p;
		while (rclcpp::ok() and not valid){
			p.x() = randomNumber(this->sampleRegion_[0], this->sampleRegion_[1]);
			p.y() = randomNumber(this->sampleRegion_[2], this->sampleRegion_[3]);
			p.z() = randomNumber(this->sampleRegion_[4], this->sampleRegion_[5]);
			valid = not this->checkCollision(p);
		}
		qRand[0] = p.x(); qRand[1] = p.y(); qRand[2] = p.z();
	}

	template <std::size_t N>
	void rrtOctomap<N>::makePlan(std::vector<KDTree::Point<N>>& plan){
		if (not this->notUpdateSampleRegion_){
			this->updateSampleRegion();
		}
		
		if (this->visRRT_){
			this->RRTVisvec_.clear();
			this->RRTVisMsg_.markers = this->RRTVisvec_;
		}

		if (this->visPath_){
			this->pathVisVec_.clear();
			this->pathVisMsg_.markers = this->pathVisVec_;
		}

		bool findPath = false;
		bool timeout = false;
		rclcpp::Time startTime = this->node_->now();
		double dT;
		int sampleNum = 0;
		KDTree::Point<N> qBack;

		cout << "[RRTPlanner]: Start planning!" << endl;
		double nearestDistance = std::numeric_limits<double>::max();  // if cannot find path to goal, find nearest way to goal
		KDTree::Point<N> nearestPoint = this->start_;
		double currentDistance = KDTree::Distance(nearestPoint, this->goal_);
		this->addVertex(this->start_);
		while (rclcpp::ok() and not findPath and not timeout){	
			rclcpp::Time currentTime = this->node_->now();
			dT = (currentTime - startTime).seconds();
			if (dT >= this->timeout_){
				timeout = true;
			}

			// 1. sample:
			KDTree::Point<N> qRand;
			double randomValue = randomNumber(0, 1);
			if (randomValue >= this->connectGoalRatio_){ // random sample trick
				this->randomConfig(qRand);
			}
			else{
				qRand = this->goal_;
			}

			// 2. find nearest neighbor:
			KDTree::Point<N> qNear;
			this->nearestVertex(qRand, qNear);

			// 3. new config by steering function:
			KDTree::Point<N> qNew;
			this->newConfig(qNear, qRand, qNew);

			// 4. Add new config to vertex and edge:
			if (this->hasNoEdge(qNear, qNew) and not this->checkCollisionLine(qNew, qNear)){ // no loop my parent is you and your parent is me
				this->addVertex(qNew);
				this->addEdge(qNear, qNew);
				++sampleNum;

				// 5. check whether goal has been reached
				findPath = this->isReach(qNew);
				if (findPath){
					qBack = qNew;
				}
				else{
					currentDistance = KDTree::Distance(qNew, this->goal_);
					if (currentDistance < nearestDistance){
						nearestDistance = currentDistance;
						nearestPoint = qNew;
					}
				}

				// visualization:
				if (this->visRRT_){
					this->updateRRTVisVec(qNew, qNear, sampleNum);
				}
			}
		}
		cout << "[RRTPlanner]: Finish planning. with sample number: " << sampleNum << endl;

		// final step: back trace using the last one
		std::vector<KDTree::Point<N>> planRaw;
		if (findPath){
			this->backTrace(qBack, planRaw);
			cout << "[RRTPlanner]: path found! Time: " << dT << "s."<< endl;
		}
		else{
			this->backTrace(nearestPoint, planRaw);
			if (planRaw.size() == 1){
				plan = planRaw;
				cout << "[RRTPlanner]: TIMEOUT! Start position might not be feasible!!" << endl;
				return;
			}
			else{
				cout << "[RRTPlanner]: TIMEOUT!"<< "(>" << this->timeout_ << "s)" << ", Return closest path. Distance: " << nearestDistance << " m." << endl;
			}
		}
		
		
		// this->shortcutWaypointPathsRandom(planRaw, plan);
		// planRaw = plan; plan.clear();
		this->shortcutWaypointPaths(planRaw, plan);

		// visualization
		if (this->visPath_){
			this->updatePathVisVec(plan);
			this->pathVisMsg_.markers = this->pathVisVec_;
		}
	}

	template <std::size_t N>
	void rrtOctomap<N>::makePlan(nav_msgs::msg::Path& plan){
		std::vector<KDTree::Point<N>> planTemp;
		this->makePlan(planTemp);
		this->pathMsgConverter(planTemp, plan);
	}

	template <std::size_t N>
	void rrtOctomap<N>::startVisModule(){
		if (this->visRRT_){
			this->RRTVisPub_ = this->node_->create_publisher<visualization_msgs::msg::MarkerArray>("/rrt_vis_array", 1);
			this->RRTVisWorker_ = std::thread(&rrtOctomap<N>::publishRRTVisMsg, this);
		}

		if (this->visPath_){
			this->pathVisPub_ = this->node_->create_publisher<visualization_msgs::msg::MarkerArray>("/rrt_planned_path", 10);
			this->pathVisWorker_ = std::thread(&rrtOctomap<N>::publishPathVisMsg, this);
		}
	}

	template <std::size_t N>
	void rrtOctomap<N>::publishRRTVisMsg(){
		rclcpp::Rate rate(5);
		while (rclcpp::ok()){
			this->RRTVisMsg_.markers = this->RRTVisvec_;
			this->RRTVisPub_->publish(this->RRTVisMsg_);
			rate.sleep();
		}
	}
	
	template <std::size_t N>
	void rrtOctomap<N>::publishPathVisMsg(){
		rclcpp::Rate rate(20);
		while (rclcpp::ok()){
			this->pathVisPub_->publish(this->pathVisMsg_);
			rate.sleep();
		}
	}

	template <std::size_t N>
	void rrtOctomap<N>::updateRRTVisVec(const KDTree::Point<N>& qNew, const KDTree::Point<N>& qNear, int id){
		visualization_msgs::msg::Marker point;
		visualization_msgs::msg::Marker line;
		geometry_msgs::msg::Point p1, p2;
		std::vector<geometry_msgs::msg::Point> lineVec;
		
		// point:
		point.header.frame_id = "map";
		point.ns = "RRT_point";
		point.id = id;
		point.type = visualization_msgs::msg::Marker::SPHERE;
		point.pose.position.x = qNew[0];
		point.pose.position.y = qNew[1];
		point.pose.position.z = qNew[2];
		point.lifetime = rclcpp::Duration::from_seconds(1);
		point.scale.x = 0.2;
		point.scale.y = 0.2;
		point.scale.z = 0.2;
		point.color.a = 0.8;
		point.color.r = 1;
		point.color.g = 0;
		point.color.b = 0;
		this->RRTVisvec_.push_back(point);

		// line:
		p1.x = qNew[0];
		p1.y = qNew[1];
		p1.z = qNew[2];
		p2.x = qNear[0];
		p2.y = qNear[1];
		p2.z = qNear[2]; 
		lineVec.push_back(p1);
		lineVec.push_back(p2);

		line.header.frame_id = "map";
		line.ns = "RRT_line";
		line.points = lineVec;
		line.id = id;
		line.type = visualization_msgs::msg::Marker::LINE_LIST;
		line.lifetime = rclcpp::Duration::from_seconds(1);
		line.scale.x = 0.05;
		line.scale.y = 0.05;
		line.scale.z = 0.05;
		line.color.a = 1.0;
		line.color.r = 1*0.5;
		line.color.g = 0;
		line.color.b = 1;
		this->RRTVisvec_.push_back(line);
	}

	template <std::size_t N>
	void rrtOctomap<N>::updatePathVisVec(const std::vector<KDTree::Point<N>> &plan){
		this->pathVisVec_.clear();
		visualization_msgs::msg::Marker waypoint;
		visualization_msgs::msg::Marker line;
		geometry_msgs::msg::Point p1, p2;
		std::vector<geometry_msgs::msg::Point> lineVec;
		for (size_t i=0; i < plan.size(); ++i){
			KDTree::Point<N> currentPoint = plan[i];
			if (i != plan.size() - 1){
				KDTree::Point<N> nextPoint = plan[i+1];
				p1.x = currentPoint[0];
				p1.y = currentPoint[1];
				p1.z = currentPoint[2];
				p2.x = nextPoint[0];
				p2.y = nextPoint[1];
				p2.z = nextPoint[2]; 
				lineVec.push_back(p1);
				lineVec.push_back(p2);
			}
			// waypoint
			waypoint.header.frame_id = "map";
			waypoint.id = 1+i;
			waypoint.ns = "rrt_path";
			waypoint.type = visualization_msgs::msg::Marker::SPHERE;
			waypoint.pose.position.x = currentPoint[0];
			waypoint.pose.position.y = currentPoint[1];
			waypoint.pose.position.z = currentPoint[2];
			waypoint.lifetime = rclcpp::Duration::from_seconds(0.5);
			waypoint.scale.x = 0.2;
			waypoint.scale.y = 0.2;
			waypoint.scale.z = 0.2;
			waypoint.color.a = 0.8;
			waypoint.color.r = 0.3;
			waypoint.color.g = 1;
			waypoint.color.b = 0.5;
			this->pathVisVec_.push_back(waypoint);
		}
		line.header.frame_id = "map";
		line.points = lineVec;
		line.ns = "rrt_path";
		line.id = 0;
		line.type = visualization_msgs::msg::Marker::LINE_LIST;
		line.lifetime = rclcpp::Duration::from_seconds(0.5);
		line.scale.x = 0.05;
		line.scale.y = 0.05;
		line.scale.z = 0.05;
		line.color.a = 1.0;
		line.color.r = 0.5;
		line.color.g = 0.1;
		line.color.b = 1;
		this->pathVisVec_.push_back(line);
	}
	

	template <std::size_t N>
	void rrtOctomap<N>::point2Octomap(const KDTree::Point<N>& q, octomap::point3d& p){
		p.x() = q[0];
		p.y() = q[1];
		p.z() = q[2];
	}

	template <std::size_t N>
	void rrtOctomap<N>::pathMsgConverter(const std::vector<KDTree::Point<N>>& pathTemp, nav_msgs::msg::Path& path){
		std::vector<geometry_msgs::msg::PoseStamped> pathVec;
		for (KDTree::Point<N> p: pathTemp){
			geometry_msgs::msg::PoseStamped ps;
			ps.header.stamp = this->node_->now();
			ps.header.frame_id = "map";
			ps.pose.position.x = p[0];
			ps.pose.position.y = p[1];
			ps.pose.position.z = p[2];
			pathVec.push_back(ps);
		}
		path.poses = pathVec;
		path.header.stamp = this->node_->now();
		path.header.frame_id = "map";
	}

	template <std::size_t N>
	double rrtOctomap<N>::getMapRes(){
		return this->mapRes_;
	}

	template <std::size_t N>
	std::vector<double> rrtOctomap<N>::getEnvBox(){
		return this->envBox_;
	}


	// =================Operator overload==============================
	template <std::size_t N>
	std::ostream &operator<<(std::ostream &os, rrtOctomap<N> &rrtplanner){
        os << "========================INFO========================\n";
        os << "[RRTPlanner]: RRT planner with octomap\n";
        os << "[Connect Ratio]: " << rrtplanner.getConnectGoalRatio() << "\n";
        // os << "[Start/Goal]:  " <<  rrtplanner.getStart() << "=>" <<  rrtplanner.getGoal() << "\n";
        std::vector<double> collisionBox = rrtplanner.getCollisionBox();
        os << "[Collision Box]: " << collisionBox[0] << " " << collisionBox[1] << " " <<  collisionBox[2] << "\n";
        os << "[Map Res]: " << rrtplanner.getMapRes() << "\n";
        os << "[Timeout]: " << rrtplanner.getTimeout() << "\n";
        os << "====================================================";
        return os;
    }
}

#endif