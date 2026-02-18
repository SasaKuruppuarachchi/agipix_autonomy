/*
	File: polyTrajOctomap.cpp
	-------------------------
	Function definition of minimum snap polynomial trajectory planner
*/
#include <trajectory_planner/polyTrajOctomap.h>

namespace trajPlanner{
	polyTrajOctomap::polyTrajOctomap(){}

	polyTrajOctomap::polyTrajOctomap(const rclcpp::Node::SharedPtr& node): node_(node){
		this->clock_ = node_->get_clock();
		this->logger_ = node_->get_logger();
		// load parameters:
		auto declareParam = [this](const std::string &name, const auto &default_value, auto &out) {
			std::string dot_name = name;
			for (auto &ch : dot_name) {
				if (ch == '/') {
					ch = '.';
				}
			}

			std::string resolved_name = name;
			if (this->node_->has_parameter(name)) {
				resolved_name = name;
			} else if (dot_name != name && this->node_->has_parameter(dot_name)) {
				resolved_name = dot_name;
			} else {
				const std::string &declare_name = (dot_name != name) ? dot_name : name;
				this->node_->declare_parameter(declare_name, default_value);
				this->node_->get_parameter(declare_name, out);
				return true;
			}
			this->node_->get_parameter(resolved_name, out);
			return true;
		};
		// Collision Box
		if (not declareParam("collision_box", std::vector<double>{1.0, 1.0, 0.6}, this->collisionBox_)){
			std::vector<double> defaultCollisionBox {1.0, 1.0, 0.6};
			this->collisionBox_ = defaultCollisionBox;
			cout << "[Trajectory Planner INFO]: No Collision Box Parameter. Use default collision Box: [1.0, 1.0, 0.6]." << endl;
		}

		// Polynomial Degree
		if (not declareParam("polynomial_degree", 7, this->polyDegree_)){
			this->polyDegree_ = 7;
			cout << "[Trajectory Planner INFO]: No Polynomail Degree Parameter. Use default: 7." << endl;
		}

		// Differential Degree: (3: Min. Jerk, 4: Min. Snap)
		if (not declareParam("differential_degree", 4, this->diffDegree_)){
			this->diffDegree_ = 4;
			cout << "[Trajectory Planner INFO]: No Differential Degree Parameter. Use default: 4." << endl;
		}

		// Continuity Degree
		if (not declareParam("continuity_degree", 4, this->continuityDegree_)){
			this->continuityDegree_ = 4;
			cout << "[Trajectory Planner INFO]: No Continuity Degree Parameter. Use default: 4." << endl;
		}

		// Desired Velocity
		if (not declareParam("desired_velocity", 1.0, this->desiredVel_)){
			this->desiredVel_ = 1.0;
			cout << "[Trajectory Planner INFO]: No Desired Velocity Parameter. Use default: 1.0 m/s." << endl;
		}

		// Map Resolution (for collision checking):
		if (not declareParam("map_resolution", 0.2, this->mapRes_)){
			this->mapRes_ = 0.2;
			cout << "[Trajectory Planner INFO]: No Map Resolution Parameter. Use default: 0.2 m." << endl;
		}

		// Max Iteration (Iteratively generating collision free trajectory)
		if (not declareParam("maximum_iteration_num", 20, this->maxIter_)){
			this->maxIter_ = 20;
			cout << "[Trajectory Planner INFO]: No Maximum Iteration Parameter. Use default: 20." << endl;
		}
		
		// Max Solving Time (Iteratively generating collision free trajectory)
		if (not declareParam("traj_timeout", 0.1, this->timeout_)){
			this->timeout_ = 0.1;
			cout << "[Trajectory Planner INFO]: No Timeout Parameter. Use default: 0.1." << endl;
		}


		// Collision Avoidance Mode: (true: adding waypoint, false: corridor)
		if (not declareParam("mode", true, this->mode_)){
			this->mode_ = true;
			cout << "[Trajectory Planner INFO]: No Mode Parameter. Use adding waypoint method." << endl;
		}

		// delta T: for collision checking
		if (not declareParam("sample_delta_time", 0.1, this->delT_)){
			this->delT_ = 0.1;
			cout << "[Trajectory Planner INFO]: No Sample Time Parameter. Use default: 0.1." << endl;
		}


		// The following method is only used in Corridor Constraint Mode:
		if (this->mode_ == false){ // corridor constraint
			// Initial corridor radius
			if (not declareParam("initial_radius", 0.5, this->initR_)){
				this->initR_ = 0.5;
				cout << "[Trajectory Planner INFO]: No Initial Corridor Radius Parameter. Use default: 0.5 m" << endl;
			}

			// Factor of shrinking (Corridor size)
			if (not declareParam("shrinking_factor", 0.8, this->fs_)){
				this->fs_ = 0.8;
				cout << "[Trajectory Planner INFO]: No Shrinking Factor Parameter. Use default: 0.8." << endl; 
			}

			// Corridor Resolution
			if (not declareParam("corridor_res", 5.0, this->corridorRes_)){
				this->corridorRes_ = 5.0;
				cout << "[Trajectory Planenr INFO]: No Corridor Resolution Parameter. Use default: 5.0." << endl;
			}
		}

		// use soft constraint or not
		if (not declareParam("soft_constraint", false, this->softConstraint_)){
			this->softConstraint_ = false;
			cout << "[Trajectory Planner INFO]: No Soft Constraint Parameter. Use default: false." << endl;
		} 

		if (this->softConstraint_){
			if (not declareParam("constraint_radius", 0.5, this->softConstraintRadius_)){
				this->softConstraintRadius_ = 0.5;
				cout << "[Trajectory Planner INFO]: No Soft Constraint Radius Parameter. Use default: 0.5 m." << endl;
			}
		}



		this->findValidTraj_ = false;

		this->trajSolver_ = NULL;
		this->pwlTrajSolver_ = NULL;
		this->mapClient_ = this->node_->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");
		this->updateMap();

		// visualize:
		this->trajVisPub_= this->node_->create_publisher<nav_msgs::msg::Path>("/trajectory", rclcpp::QoS(1));
		this->samplePointVisPub_ = this->node_->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectory_sp", rclcpp::QoS(1));
		this->waypointVisPub_ = this->node_->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoint", rclcpp::QoS(1));
		this->trajVisWorker_ = std::thread(&polyTrajOctomap::publishTrajectory, this);
		this->samplePointVisWorker_ = std::thread(&polyTrajOctomap::publishSamplePoint, this);
		this->waypointVisWorker_ = std::thread(&polyTrajOctomap::publishWaypoint, this);
		// visualize corridor:
		if (this->mode_ == false){
			this->corridorVisPub_ = this->node_->create_publisher<visualization_msgs::msg::MarkerArray>("/corridor", rclcpp::QoS(1));
			this->corridorVisWorker_ = std::thread(&polyTrajOctomap::publishCorridor, this);
		}
	}

	void polyTrajOctomap::updateMap(){
		auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
		while (!this->mapClient_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()){
			RCLCPP_INFO(this->logger_, "[Trajectory Planner INFO]: Wait for Octomap Service...");
		}
		auto future = this->mapClient_->async_send_request(request);
		while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
			RCLCPP_INFO(this->logger_, "[Trajectory Planner INFO]: Wait for Octomap Service response...");
		}
		if (!rclcpp::ok()) {
			RCLCPP_WARN(this->logger_, "[Trajectory Planner INFO]: Octomap service call interrupted.");
			return;
		}
		auto mapSrv = future.get();
		octomap::AbstractOcTree* abtree = octomap_msgs::binaryMsgToMap(mapSrv->map);
		this->map_ = dynamic_cast<octomap::OcTree*>(abtree);
		cout << "[Trajectory Planner INFO]: Map updated!" << endl;
	}

	polyTrajSolver*  polyTrajOctomap::initSolver(){
		return new polyTrajSolver (this->polyDegree_, this->diffDegree_, this->continuityDegree_, this->desiredVel_);
	}

	void polyTrajOctomap::freeSolver(){
		delete this->trajSolver_;
		this->trajSolver_ = NULL;
	}

	pwlTraj* polyTrajOctomap::initPWLSolver(){
		return new pwlTraj (this->node_);
	}

	void polyTrajOctomap::freePWLSolver(){
		delete this->pwlTrajSolver_;
		this->pwlTrajSolver_ = NULL;
	}

	void polyTrajOctomap::updatePath(const nav_msgs::msg::Path& path){
		std::vector<trajPlanner::pose> trajPath;
		for (const geometry_msgs::msg::PoseStamped &p: path.poses){
			trajPlanner::pose trajP (p.pose.position.x, p.pose.position.y, p.pose.position.z);
			trajPath.push_back(trajP);
		}
		this->updatePath(trajPath);
	}

	void polyTrajOctomap::updatePath(const std::vector<pose>& path){
		this->path_ = path;
	}

	void polyTrajOctomap::insertWaypoint(const std::set<int>& seg){
		for (std::set<int>::reverse_iterator rit = seg.rbegin(); rit != seg.rend(); rit++){ // reverse order can solve the segment number change problem
			int idx = *rit;
			trajPlanner::pose p1 = this->path_[idx];
			trajPlanner::pose p2 = this->path_[idx+1];
			trajPlanner::pose pMid = trajPlanner::pose ((p1.x+p2.x)/2, (p1.y+p2.y)/2, (p1.z+p2.z)/2);
			this->path_.insert(this->path_.begin()+idx+1, pMid);
		}
	}

	void polyTrajOctomap::adjustCorridorSize(const std::set<int>& collisionSeg, std::vector<double>& corridorSizeVec){
		for (int collisionIdx: collisionSeg){
			corridorSizeVec[collisionIdx] = corridorSizeVec[collisionIdx] * this->fs_; 
		}
	}

	void polyTrajOctomap::updateInitVel(double vx, double vy, double vz){
		geometry_msgs::msg::Twist v;
		v.linear.x = vx;
		v.linear.y = vy;
		v.linear.z = vz;
		this->updateInitVel(v);
	}

	void polyTrajOctomap::updateInitVel(const geometry_msgs::msg::Twist& v){
		this->initVel_ = v;
	}

	void polyTrajOctomap::updateInitAcc(double ax, double ay, double az){
		geometry_msgs::msg::Twist a;
		a.linear.x = ax;
		a.linear.y = ay;
		a.linear.z = az;
		this->updateInitAcc(a);
	}


	void polyTrajOctomap::updateInitAcc(const geometry_msgs::msg::Twist& a){
		this->initAcc_ = a;
	}

	void polyTrajOctomap::setDefaultInit(){
		// initialize 
		this->updateInitVel(0, 0, 0);
		this->updateInitAcc(0, 0, 0); 
	}


	void polyTrajOctomap::makePlan(){
		rclcpp::Time startTime = this->clock_->now();
		if (this->mode_ == true){
			this->makePlanAddingWaypoint();
		}
		else{
			this->makePlanCorridorConstraint();
		}

		rclcpp::Time endTime = this->clock_->now();
		double dT = (endTime - startTime).seconds();
		cout << "[Trajectory Planner INFO]: Time: " << dT << "s."<< endl;
	}

	void polyTrajOctomap::makePlan(nav_msgs::msg::Path& trajectory, double delT){
		std::vector<pose> trajectoryTemp;
		this->makePlan(trajectoryTemp, delT);
		this->trajMsgConverter(trajectoryTemp, trajectory);
	}

	void polyTrajOctomap::makePlan(std::vector<pose>& trajectory, double delT){
		rclcpp::Time startTime = this->clock_->now();
		if (this->mode_){
			this->makePlanAddingWaypoint(trajectory, delT);
		}
		else{
			this->makePlanCorridorConstraint(trajectory, delT);
		}
		rclcpp::Time endTime = this->clock_->now();
		double dT = (endTime - startTime).seconds();
		cout << "[Trajectory Planner INFO]: Time: " << dT << "s."<< endl;
	}

	void polyTrajOctomap::makePlanAddingWaypoint(){
		this->findValidTraj_ = false;
		std::vector<trajPlanner::pose> trajectory;
		if (this->path_.size() == 1){
			trajectory = this->path_;
			this->updateTrajVisMsg(trajectory);
			this->findValidTraj_ = true;
			return;
		}

		if (this->trajSolver_ != NULL){
			this->freeSolver();
		}
		this->setDefaultInit(); // vel acc
		this->trajSolver_ = this->initSolver();
		this->trajSolver_->updateInitVel(this->initVel_);
		this->trajSolver_->updateInitAcc(this->initAcc_);
		this->trajSolver_->updatePath(this->path_);
		int countIter = 0;
		bool valid = false;
		rclcpp::Time startTime = this->clock_->now();
		while (rclcpp::ok() and not valid){
			rclcpp::Time endTime = this->clock_->now();
			if ((endTime - startTime).seconds() >= this->timeout_){
				cout << "[Trajectory Planner INFO]: Timeout." << endl;
				break;
			}

			// this->trajSolver_->updateInitVel(this->initVel_);
			// this->trajSolver_->updateInitAcc(this->initAcc_);
			// this->trajSolver_->updatePath(this->path_);
			if (this->softConstraint_){
				this->trajSolver_->setSoftConstraint(this->softConstraintRadius_, this->softConstraintRadius_, 0);
			}
			this->trajSolver_->solve();
			this->trajSolver_->getTrajectory(trajectory, this->delT_);
			std::set<int> collisionSeg;
			valid = not this->checkCollisionTraj(trajectory, this->delT_, collisionSeg);
			if (not valid){
				this->insertWaypoint(collisionSeg);
			}
			++countIter;
			if (countIter > this->maxIter_){
				break;
			}
		}
		
		if (valid){
			cout << "[Trajectory Planner INFO]: Found valid trajectory!" << endl;	
			this->findValidTraj_ = true;
		}
		else{
			cout << "[Trajectory Planner INFO]: Not found. Return the best. Please consider piecewise linear trajectory!!" << endl;	
			if (this->pwlTrajSolver_ != NULL){
				this->freePWLSolver();
			}
			this->pwlTrajSolver_ = this->initPWLSolver();
			this->pwlTrajSolver_->updatePath(this->path_);
			this->pwlTrajSolver_->makePlan(trajectory, this->delT_);
		}

		this->updateTrajVisMsg(trajectory);
	}

	void polyTrajOctomap::makePlanAddingWaypoint(std::vector<pose>& trajectory, double delT){	
		this->findValidTraj_ = false;
		if (this->path_.size() == 1){
			trajectory = this->path_;
			this->updateTrajVisMsg(trajectory);
			this->findValidTraj_ = true;
			return;
		}

		if (this->trajSolver_ != NULL){
			this->freeSolver();
		}

		this->setDefaultInit(); // vel acc
		this->trajSolver_ = this->initSolver();
		this->trajSolver_->updateInitVel(this->initVel_);
		this->trajSolver_->updateInitAcc(this->initAcc_);
		this->trajSolver_->updatePath(this->path_);
		int countIter = 0;
		bool valid = false;
		rclcpp::Time startTime = this->clock_->now();
		while (rclcpp::ok() and not valid){
			rclcpp::Time endTime = this->clock_->now();
			if ((endTime - startTime).seconds() >= this->timeout_){
				cout << "[Trajectory Planner INFO]: Timeout." << endl;
				break;
			}

			// this->trajSolver_->updateInitVel(this->initVel_);
			// this->trajSolver_->updateInitAcc(this->initAcc_);
			// this->trajSolver_->updatePath(this->path_);
			if (this->softConstraint_){
				this->trajSolver_->setSoftConstraint(this->softConstraintRadius_, this->softConstraintRadius_, 0);
			}
			this->trajSolver_->solve();
			this->trajSolver_->getTrajectory(trajectory, delT);
			std::set<int> collisionSeg;
			valid = not this->checkCollisionTraj(trajectory, delT, collisionSeg);
			if (not valid){
				this->insertWaypoint(collisionSeg);
			}
			++countIter;
			if (countIter > this->maxIter_){
				break;
			}
		}
		

		if (valid){
			cout << "[Trajectory Planner INFO]: Found valid trajectory!" << endl;	
			this->findValidTraj_ = true;
		}
		else{
			cout << "[Trajectory Planner INFO]: Not found. Return the best. Please consider piecewise linear trajectory!!" << endl;	
			if (this->pwlTrajSolver_ != NULL){
				this->freePWLSolver();
			}
			this->pwlTrajSolver_ = this->initPWLSolver();
			this->pwlTrajSolver_->updatePath(this->path_);
			this->pwlTrajSolver_->makePlan(trajectory, delT);
		}

		this->updateTrajVisMsg(trajectory);
	}

	void polyTrajOctomap::makePlanCorridorConstraint(){
		this->findValidTraj_ = false;
		std::vector<trajPlanner::pose> trajectory;
		if (this->path_.size() == 1){
			trajectory = this->path_;
			this->updateTrajVisMsg(trajectory);
			this->findValidTraj_ = true;
			return;
		}

		if (this->trajSolver_ != NULL){
			this->freeSolver();
		}

		this->setDefaultInit(); // vel acc
		this->trajSolver_ = this->initSolver();
		this->trajSolver_->updatePath(this->path_);
		this->trajSolver_->updateInitVel(this->initVel_);
		this->trajSolver_->updateInitAcc(this->initAcc_);

		double corridorSize = this->initR_;
		std::vector<double> corridorSizeVec;
		for (size_t i=0; i<this->path_.size()-1; ++i){
			corridorSizeVec.push_back(corridorSize);
		} 

		int countIter = 0;
		bool valid = false;
		rclcpp::Time startTime = this->clock_->now();
		while (rclcpp::ok() and not valid){
			rclcpp::Time endTime = this->clock_->now();
			if ((endTime - startTime).seconds() >= this->timeout_){
				cout << "[Trajectory Planner INFO]: Timeout." << endl;
				break;
			}
			// this->trajSolver_->updateInitVel(this->initVel_);
			// this->trajSolver_->updateInitAcc(this->initAcc_);
			this->trajSolver_->setCorridorConstraint(corridorSizeVec, this->corridorRes_);
			if (this->softConstraint_){
				this->trajSolver_->setSoftConstraint(this->softConstraintRadius_, this->softConstraintRadius_, 0);
			}
			this->trajSolver_->solve();
			this->trajSolver_->getTrajectory(trajectory, this->delT_);
			std::set<int> collisionSeg;
			valid  = not this->checkCollisionTraj(trajectory, this->delT_, collisionSeg);

			if (not valid){
				this->adjustCorridorSize(collisionSeg, corridorSizeVec);
			}
			++countIter;
			if (countIter > this->maxIter_){
				break;
			}
			// std::vector<double> corridorSizeVecVis;
			// std::vector<std::unordered_map<double, trajPlanner::pose>> segToTimePoseVis;
			// this->trajSolver_->getCorridor(segToTimePoseVis, corridorSizeVecVis);
			// this->updateCorridorVisMsg(segToTimePoseVis, corridorSizeVecVis);
			// this->updateTrajVisMsg(trajectory);
			// std::cin.get();

		}

		std::vector<double> corridorSizeVecVis;
		std::vector<std::unordered_map<double, trajPlanner::pose>> segToTimePoseVis;
		this->trajSolver_->getCorridor(segToTimePoseVis, corridorSizeVecVis);
		this->updateCorridorVisMsg(segToTimePoseVis, corridorSizeVecVis);

		if (valid){
			cout << "[Trajectory Planner INFO]: Found valid trajectory!" << endl;	
			this->findValidTraj_ = true;
		}
		else{
			cout << "[Trajectory Planner INFO]: Not found. Return the best. Please consider piecewise linear trajectory!!" << endl;	
			if (this->pwlTrajSolver_ != NULL){
				this->freePWLSolver();
			}
			this->pwlTrajSolver_ = this->initPWLSolver();
			this->pwlTrajSolver_->updatePath(this->path_);
			this->pwlTrajSolver_->makePlan(trajectory, this->delT_);
		}

		this->updateTrajVisMsg(trajectory);
	}

	void polyTrajOctomap::makePlanCorridorConstraint(std::vector<pose>& trajectory, double delT){
		this->findValidTraj_ = false;
		if (this->path_.size() == 1){
			trajectory = this->path_;
			this->updateTrajVisMsg(trajectory);
			this->findValidTraj_ = true;
			return;
		}

		if (this->trajSolver_ != NULL){
			this->freeSolver();
		};

		this->setDefaultInit(); // vel acc
		this->trajSolver_ = this->initSolver();
		this->trajSolver_->updatePath(this->path_);
		this->trajSolver_->updateInitVel(this->initVel_);
		this->trajSolver_->updateInitAcc(this->initAcc_);

		double corridorSize = this->initR_;
		std::vector<double> corridorSizeVec;
		for (size_t i=0; i<this->path_.size()-1; ++i){
			corridorSizeVec.push_back(corridorSize);
		} 

		int countIter = 0;
		bool valid = false;
		rclcpp::Time startTime = this->clock_->now();
		while (rclcpp::ok() and not valid){
			rclcpp::Time endTime = this->clock_->now();
			if ((endTime - startTime).seconds() >= this->timeout_){
				cout << "[Trajectory Planner INFO]: Timeout." << endl;
				break;
			}
			// this->trajSolver_->updateInitVel(this->initVel_);
			// this->trajSolver_->updateInitAcc(this->initAcc_);
			this->trajSolver_->setCorridorConstraint(corridorSizeVec, this->corridorRes_);
			if (this->softConstraint_){
				this->trajSolver_->setSoftConstraint(this->softConstraintRadius_, this->softConstraintRadius_, 0);
			}
			this->trajSolver_->solve();
			this->trajSolver_->getTrajectory(trajectory, delT);
			std::set<int> collisionSeg;
			valid  = not this->checkCollisionTraj(trajectory, delT, collisionSeg);
			if (not valid){
				this->adjustCorridorSize(collisionSeg, corridorSizeVec);
			}
			++countIter;
			if (countIter > this->maxIter_){
				break;
			}
		}

		
		std::vector<double> corridorSizeVecVis;
		std::vector<std::unordered_map<double, trajPlanner::pose>> segToTimePoseVis;
		this->trajSolver_->getCorridor(segToTimePoseVis, corridorSizeVecVis);
		this->updateCorridorVisMsg(segToTimePoseVis, corridorSizeVecVis);

		if (valid){
			cout << "[Trajectory Planner INFO]: Found valid trajectory!" << endl;	
			this->findValidTraj_ = true;
		}
		else{
			cout << "[Trajectory Planner INFO]: Not found. Return the best. Please consider piecewise linear trajectory!!" << endl;	
			if (this->pwlTrajSolver_ != NULL){
				this->freePWLSolver();
			}
			this->pwlTrajSolver_ = this->initPWLSolver();
			this->pwlTrajSolver_->updatePath(this->path_);
			this->pwlTrajSolver_->makePlan(trajectory, delT);
		}
		this->updateTrajVisMsg(trajectory);
	}

	bool polyTrajOctomap::checkCollision(const octomap::point3d& p){
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
					if (this->checkCollisionPoint(octomap::point3d(xmin+xID*this->mapRes_, ymin+yID*this->mapRes_, zmin+zID*this->mapRes_), false)){
						return true;
					}
				}
			}
		}
		return false;
	}


	bool polyTrajOctomap::checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown){
		double min_x, max_x, min_y, max_y, min_z, max_z;
		this->map_->getMetricMax(max_x, max_y, max_z);
		this->map_->getMetricMin(min_x, min_y, min_z);
		if (p.x() < min_x or p.x() > max_x or p.y() < min_y or p.y() > max_y or p.z() < min_z or p.z() > max_z){
			return true;
		}

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

	bool polyTrajOctomap::checkCollisionPoint(const pose& pTraj){
		octomap::point3d p;
		this->pose2Octomap(pTraj, p);
		return this->checkCollisionPoint(p);
	}

	bool polyTrajOctomap::checkCollisionLine(const octomap::point3d& p1, const octomap::point3d& p2){
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

	bool polyTrajOctomap::checkCollisionLine(const pose& pTraj1, const pose& pTraj2){
		octomap::point3d p1, p2;
		this->pose2Octomap(pTraj1, p1);
		this->pose2Octomap(pTraj2, p2);
		return this->checkCollisionLine(p1, p2);
	}

	bool polyTrajOctomap::checkCollisionTraj(const std::vector<pose>& trajectory, std::vector<int>& collisionIdx){
		octomap::point3d p;
		int count = 0;
		bool hasColllision = false;
		for (pose pTraj: trajectory){
			this->pose2Octomap(pTraj, p);
			if (this->checkCollision(p)){
				hasColllision = true;
				collisionIdx.push_back(count);
			}
			++count;
		}
		return hasColllision;
	}

	bool polyTrajOctomap::checkCollisionTraj(const std::vector<pose>& trajectory, double delT, std::set<int>& collisionSeg){
		collisionSeg.clear();
		std::vector<double> timeKnot = this->trajSolver_->getTimeKnot();
		octomap::point3d p;
		double t = 0;
		bool hasColllision = false;
		for (pose pTraj: trajectory){
			this->pose2Octomap(pTraj, p);
			if (this->checkCollision(p)){
				hasColllision = true;
				for (size_t i=0; i<timeKnot.size()-1; ++i){
					double startTime = timeKnot[i];
					double endTime = timeKnot[i+1];
					if ((t>=startTime) and (t<=endTime)){
						collisionSeg.insert(i);
						break;
					}
				}
			}
			t += delT;
		}
		return hasColllision;
	}

	geometry_msgs::msg::PoseStamped polyTrajOctomap::getPose(double t){
		if (t > this->getDuration()){
			t = this->getDuration();
		}
		geometry_msgs::msg::PoseStamped ps;
		if (this->findValidTraj_){
			trajPlanner::pose p = this->trajSolver_->getPose(t);
			ps.pose.position.x = p.x;
			ps.pose.position.y = p.y;
			ps.pose.position.z = p.z;
			geometry_msgs::msg::Quaternion quat = quaternion_from_rpy(0, 0, p.yaw);
			ps.pose.orientation = quat;
			ps.header.stamp = this->clock_->now();
			ps.header.frame_id = "drone0/map";
		}
		else{
			ps = this->pwlTrajSolver_->getPose(t);
		}
		return ps;
	}

	double polyTrajOctomap::getDuration(){
		if (this->path_.size() == 1){
			return 0.0; // no duration
		}
		if (this->findValidTraj_){
			return this->trajSolver_->getTimeKnot().back();
		}
		else{
			return this->pwlTrajSolver_->getTimeKnot().back();
		}
	}

	double polyTrajOctomap::getDegree(){
		return this->polyDegree_;
	}

	double polyTrajOctomap::getDiffDegree(){
		return this->diffDegree_;
	}

	double polyTrajOctomap::getContinuityDegree(){
		return this->continuityDegree_;
	}

	double polyTrajOctomap::getInitialRadius(){
		return this->initR_;
	}

	double polyTrajOctomap::getDesiredVel(){
		return this->desiredVel_;
	}

	double polyTrajOctomap::getShrinkFactor(){
		return this->fs_;
	}

	void polyTrajOctomap::updateTrajVisMsg(const std::vector<pose>& trajectory){
		std::vector<geometry_msgs::msg::PoseStamped> trajVisVec;
		for (pose p: trajectory){
			geometry_msgs::msg::PoseStamped ps;
			ps.header.frame_id = "drone0/map";
			ps.header.stamp = this->clock_->now();
			ps.pose.position.x = p.x;
			ps.pose.position.y = p.y;
			ps.pose.position.z = p.z; 
			trajVisVec.push_back(ps);
		}
		this->trajVisMsg_.poses = trajVisVec;

		std::vector<visualization_msgs::msg::Marker> samplePointVec;
		visualization_msgs::msg::Marker samplePoint;
		int count = 0;
		for (pose p: trajectory){
			samplePoint.header.frame_id = "drone0/map";
			samplePoint.header.stamp = this->clock_->now();
			samplePoint.ns = "sample_point";
			samplePoint.id = count;
			samplePoint.type = visualization_msgs::msg::Marker::ARROW;
			samplePoint.action = visualization_msgs::msg::Marker::ADD;
			samplePoint.pose.position.x = p.x;
			samplePoint.pose.position.y = p.y;
			samplePoint.pose.position.z = p.z;
			samplePoint.pose.orientation = quaternion_from_rpy(0, 0, p.yaw);
			samplePoint.lifetime = rclcpp::Duration::from_seconds(0.5);
			samplePoint.scale.x = 0.1;
			samplePoint.scale.y = 0.2;
			samplePoint.scale.z = 0.2;
			samplePoint.color.a = 0.5;
			samplePoint.color.r = 0.7;
			samplePoint.color.g = 1.0;
			samplePoint.color.b = 0.0;
			samplePointVec.push_back(samplePoint);
			++count;
		}
		this->samplePointMsg_.markers = samplePointVec;

		std::vector<visualization_msgs::msg::Marker> waypointVec;
		visualization_msgs::msg::Marker waypoint;
		int waypointCount = 0;
		for (pose p: this->path_){
			waypoint.header.frame_id = "drone0/map";
			waypoint.header.stamp = this->clock_->now();
			waypoint.ns = "waypoint";
			waypoint.id = waypointCount;
			waypoint.type = visualization_msgs::msg::Marker::SPHERE;
			waypoint.action = visualization_msgs::msg::Marker::ADD;
			waypoint.pose.position.x = p.x;
			waypoint.pose.position.y = p.y;
			waypoint.pose.position.z = p.z;
			waypoint.lifetime = rclcpp::Duration::from_seconds(0.5);
			waypoint.scale.x = 0.3;
			waypoint.scale.y = 0.3;
			waypoint.scale.z = 0.3;
			waypoint.color.a = 0.5;
			waypoint.color.r = 0.7;
			waypoint.color.g = 1.0;
			waypoint.color.b = 0.0;
			waypointVec.push_back(waypoint);
			++waypointCount;
		}
		this->waypointMsg_.markers = waypointVec;
	}

	void polyTrajOctomap::updateCorridorVisMsg(const std::vector<std::unordered_map<double, trajPlanner::pose>>& segToTimePose, const std::vector<double>& corridorSizeVec){
		std::vector<visualization_msgs::msg::Marker> corridorVec;
		visualization_msgs::msg::Marker box;
		int boxCount = 0;
		for (size_t i=0; i<this->path_.size()-1; ++i){
			std::unordered_map<double, trajPlanner::pose> timeToPose = segToTimePose[i];
			for (auto itr: timeToPose){
				trajPlanner::pose p = itr.second;
				box.header.frame_id = "drone0/map";
				box.header.stamp = this->clock_->now();
				box.ns = "corridor";
				box.id = boxCount;
				box.type = visualization_msgs::msg::Marker::CUBE;
				box.action = visualization_msgs::msg::Marker::ADD;
				box.pose.position.x = p.x;
				box.pose.position.y = p.y;
				box.pose.position.z = p.z;
				box.lifetime = rclcpp::Duration::from_seconds(0.5);
				box.scale.x = corridorSizeVec[i]*2;
				box.scale.y = corridorSizeVec[i]*2;
				box.scale.z = corridorSizeVec[i]*2;
				box.color.a = 0.3;
				box.color.r = 0.0;
				box.color.g = 0.5;
				box.color.b = 0.3;
				corridorVec.push_back(box);
				++boxCount;
			}
		}
		this->corridorMsg_.markers = corridorVec;
	}

	void polyTrajOctomap::publishTrajectory(){
		rclcpp::Rate r (10);
		while (rclcpp::ok()){
			this->trajVisMsg_.header.frame_id = "drone0/map";
			this->trajVisMsg_.header.stamp = this->clock_->now();
			this->trajVisPub_->publish(this->trajVisMsg_);
			r.sleep();
		}
	}

	void polyTrajOctomap::publishSamplePoint(){
		rclcpp::Rate r (10);
		while (rclcpp::ok()){
			this->samplePointVisPub_->publish(this->samplePointMsg_);
			r.sleep();
		}
	}

	void polyTrajOctomap::publishWaypoint(){
		rclcpp::Rate r (10);
		while (rclcpp::ok()){
			this->waypointVisPub_->publish(this->waypointMsg_);
			r.sleep();
		}
	}

	void polyTrajOctomap::publishCorridor(){
		rclcpp::Rate r (10);
		while (rclcpp::ok()){
			this->corridorVisPub_->publish(this->corridorMsg_);
			r.sleep();
		}
	}

	void polyTrajOctomap::pose2Octomap(const pose& pTraj, octomap::point3d& p){
		p.x() = pTraj.x;
		p.y() = pTraj.y;
		p.z() = pTraj.z;
	}

	void polyTrajOctomap::trajMsgConverter(const std::vector<trajPlanner::pose>& trajectoryTemp, nav_msgs::msg::Path& trajectory){
		std::vector<geometry_msgs::msg::PoseStamped> trajVec;
		for (trajPlanner::pose pTemp: trajectoryTemp){
			geometry_msgs::msg::PoseStamped ps;
			ps.header.stamp = this->clock_->now();
			ps.header.frame_id = "drone0/map";
			ps.pose.position.x = pTemp.x;
			ps.pose.position.y = pTemp.y;
			ps.pose.position.z = pTemp.z;

			geometry_msgs::msg::Quaternion quat = quaternion_from_rpy(0, 0, pTemp.yaw);
			ps.pose.orientation = quat;
			trajVec.push_back(ps);
		}
		trajectory.header.stamp = this->clock_->now();
		trajectory.header.frame_id = "drone0/map";
		trajectory.poses = trajVec;
	}
}
