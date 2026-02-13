/*
*	File: rrtStarOctomap.h
*	---------------
*   RRT star planner class based on Octomap.
*/
#ifndef RRTSTATOCTOMAP_H
#define RRTSTAROCTOMAP_H
#include <global_planner/rrtOctomap.h>

namespace globalPlanner{
	template <std::size_t N>
	class rrtStarOctomap : public rrtOctomap<N>{
	private:
		rclcpp::Node::SharedPtr node_;
		double rNeighborhood_; // region for rewiring
		double maxNeighbors_; // maximum value for neighboorhood
		std::unordered_map<KDTree::Point<N>, double, KDTree::PointHasher> distance_;

	public:
		// default
		rrtStarOctomap();

		// constructor using ros param:
		rrtStarOctomap(const rclcpp::Node::SharedPtr &node);
	
		// constructor:
		rrtStarOctomap(const rclcpp::Node::SharedPtr &node, double rNeighborhood, double maxNeighbors, std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ=0.3, double dR=0.2, double connectGoalRatio=0.10, double timeout=1.0, bool visPath=true);
		
		void mapCB(const octomap_msgs::msg::Octomap::SharedPtr msg);

		// get neighborhood points from the vector
		void getNeighborhood(const KDTree::Point<N>& q, std::vector<KDTree::Point<N>>& neighborhood);

		// get distance to start if connecting to q
		double getDistanceToStart(const KDTree::Point<N>& qNew, const KDTree::Point<N>& q);

		// get distance to start point at specific node:
		double getDistanceToStart(const KDTree::Point<N>& q);

		// record the distance to start based on parent point and current point:
		void updateDistanceToStart(const KDTree::Point<N>& qNew, const KDTree::Point<N>& qParent);

		// *** Core function: make plan based on all input ***
		virtual void makePlan(std::vector<KDTree::Point<N>>& plan);
		virtual void makePlan(nav_msgs::msg::Path& plan);

		// return neighborhood radius:
		double getNeighborHoodRadius();
	};
	
	// ============function definition===================
	template <std::size_t N>
	rrtStarOctomap<N>::rrtStarOctomap(const rclcpp::Node::SharedPtr &node) : rrtOctomap<N>(), node_(node){
		this->rrtOctomap<N>::node_ = node;
		// Map Resolution for Collisiong checking
		this->mapRes_ = declareAndGetParam<double>(this->node_, "map_resolution", 0.2);

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


		// Neighborhood radius:
		this->rNeighborhood_ = declareAndGetParam<double>(this->node_, "neighborhood_radius", 1.0);

		// Maximum Number of Neighbors:
		this->maxNeighbors_ = declareAndGetParam<double>(this->node_, "max_num_neighbors", 10);


		this->visRRT_ = false;		

		this->mapClient_ = this->node_->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");
		// this->updateMap();

		this->mapCbGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		rclcpp::SubscriptionOptions mapOptions;
		mapOptions.callback_group = this->mapCbGroup_;
		this->mapSub_ = this->node_->create_subscription<octomap_msgs::msg::Octomap>(
			"/octomap_full", 1, std::bind(&rrtStarOctomap::mapCB, this, std::placeholders::_1), mapOptions);
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
	rrtStarOctomap<N>::rrtStarOctomap(const rclcpp::Node::SharedPtr &node, double rNeighborhood, double maxNeighbors, std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ, double dR, double connectGoalRatio, double timeout, bool visPath)
	: node_(node), rNeighborhood_(rNeighborhood), maxNeighbors_(maxNeighbors),  rrtOctomap<N>(collisionBox, envBox, mapRes, delQ, dR, connectGoalRatio, timeout, false, visPath){
			this->mapClient_ = this->node_->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");
		// this->updateMap();

		// Visualization:
		this->startVisModule();
	}

	template <std::size_t N>
	void rrtStarOctomap<N>::mapCB(const octomap_msgs::msg::Octomap::SharedPtr msg){
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
	void rrtStarOctomap<N>::getNeighborhood(const KDTree::Point<N>& q, std::vector<KDTree::Point<N>>& neighborhood){
		this->ktree_.boundedRangeSearch(q, this->rNeighborhood_, this->maxNeighbors_, neighborhood);
	}

	template <std::size_t N>
	double rrtStarOctomap<N>::getDistanceToStart(const KDTree::Point<N>& qNew, const KDTree::Point<N>& q){
		return this->distance_[q] + KDTree::Distance(qNew, q);
	}

	template <std::size_t N>
	double rrtStarOctomap<N>::getDistanceToStart(const KDTree::Point<N>& q){
		return this->distance_[q];
	}

	template <std::size_t N>
	void rrtStarOctomap<N>::updateDistanceToStart(const KDTree::Point<N>& qNew, const KDTree::Point<N>& qParent){
		this->distance_[qNew] = this->getDistanceToStart(qNew, qParent);
	}


	template <std::size_t N>
	void rrtStarOctomap<N>::makePlan(std::vector<KDTree::Point<N>>& plan){
		if (not this->notUpdateSampleRegion_){
			this->updateSampleRegion();
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
		this->distance_[this->start_] = 0; // init distance to start
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
			if (this->hasNoEdge(qNear, qNew) and not this->checkCollisionLine(qNew, qNear)){
				// RRT* rewiring 1;
				KDTree::Point<N> qBestParent = qNear;
				double minDistance = this->getDistanceToStart(qNew, qNear);
				std::vector<KDTree::Point<N>> neighborhood;
				this->getNeighborhood(qNew, neighborhood);
				for (KDTree::Point<N> qNeighbor: neighborhood){
					double neighborDistance = this->getDistanceToStart(qNew, qNeighbor);
					if (neighborDistance < minDistance){
						if (not this->checkCollisionLine(qNew, qNeighbor)){
							minDistance = neighborDistance;
							qBestParent = qNeighbor;
						}
					}
				}
				this->addVertex(qNew);
				this->addEdge(qBestParent, qNew);
				this->updateDistanceToStart(qNew, qBestParent); // record the distance

				// RRT* rewiring 2:
				for (KDTree::Point<N> qNeighbor: neighborhood){
					double qNeighborDistance = this->getDistanceToStart(qNeighbor);
					double qNeighborDistanceNew = this->getDistanceToStart(qNeighbor, qNew);
					if (qNeighborDistanceNew < qNeighborDistance){
						if (not this->checkCollisionLine(qNeighbor, qNew)){
							this->addEdge(qNew, qNeighbor);
							this->updateDistanceToStart(qNeighbor, qNew);
						}
					}
				} 


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
		this->shortcutWaypointPaths(planRaw, plan);

		// visualization
		if (this->visPath_){
			this->updatePathVisVec(plan);
			this->pathVisMsg_.markers = this->pathVisVec_;
		}
	}

	template <std::size_t N>
	void rrtStarOctomap<N>::makePlan(nav_msgs::msg::Path& plan){
		std::vector<KDTree::Point<N>> planTemp;
		this->makePlan(planTemp);
		this->pathMsgConverter(planTemp, plan);
	}

	template <std::size_t N>
	double rrtStarOctomap<N>::getNeighborHoodRadius(){
		return this->rNeighborhood_;
	}


	// overload
	template <std::size_t N>
	std::ostream &operator<<(std::ostream &os, rrtStarOctomap<N> &rrtStarPlanner){
        os << "========================INFO========================\n";
        os << "[RRTPlanner]: RRT* planner with octomap\n";
        os << "[Connect Ratio]: " << rrtStarPlanner.getConnectGoalRatio() << "\n";
        // os << "[Start/Goal]:  " <<  rrtStarPlanner.getStart() << "=>" <<  rrtStarPlanner.getGoal() << "\n";
        std::vector<double> collisionBox = rrtStarPlanner.getCollisionBox();
        os << "[Collision Box]: " << collisionBox[0] << " " << collisionBox[1] << " " <<  collisionBox[2] << "\n";
        os << "[Map Res]: " << rrtStarPlanner.getMapRes() << "\n";
        os << "[Timeout]: " << rrtStarPlanner.getTimeout() << "\n";
        os << "[Neighborhood Radius]: " << rrtStarPlanner.getNeighborHoodRadius() << "\n";
        os << "====================================================";
        return os;
    }
}	


#endif