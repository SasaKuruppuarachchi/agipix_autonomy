/*
	FILE: polyTrajOccMap.cpp
	---------------------------
	minimum snap polynomial trajectory planner implementation based on occupancy grid map
*/

#include <trajectory_planner/polyTrajOccMap.h>

namespace trajPlanner{
	polyTrajOccMap::polyTrajOccMap(const rclcpp::Node::SharedPtr& node) : node_(node){
		this->clock_ = node_->get_clock();
		this->logger_ = node_->get_logger();
		this->initParam();
		this->registerPub();
		this->registerCallback();
		if (this->usePWL_){
			this->initPWLSolver();
		}
		this->setDefaultInit(); // vel acc (this can be overwritten by user defined initial condition)
	}

	void polyTrajOccMap::initParam(){
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
		// Polynomial Degree
		if (not declareParam("poly_traj/polynomial_degree", 7, this->polyDegree_)){
			this->polyDegree_ = 7;
			cout << "[minSnapTraj]: No Polynomail Degree Parameter. Use default: 7." << endl;
		}
		else{
			cout << "[minSnapTraj]: Polynomial Degree: " << this->polyDegree_ << endl;
		}

		// Differential Degree: (3: Min. Jerk, 4: Min. Snap)
		if (not declareParam("poly_traj/differential_degree", 4, this->diffDegree_)){
			this->diffDegree_ = 4;
			cout << "[minSnapTraj]: No Differential Degree Parameter. Use default: 4." << endl;
		}
		else{
			cout << "[minSnapTraj]: Differential Degree: " << this->diffDegree_ << endl;
		}

		// Continuity Degree
		if (not declareParam("poly_traj/continuity_degree", 4, this->continuityDegree_)){
			this->continuityDegree_ = 4;
			cout << "[minSnapTraj]: No Continuity Degree Parameter. Use default: 4." << endl;
		}
		else{
			cout << "[minSnapTraj]: Continuity Degree: " << this->continuityDegree_ << endl;
		}

		// Desired Velocity
		if (not declareParam("poly_traj/desired_velocity", 1.0, this->desiredVel_)){
			this->desiredVel_ = 1.0;
			cout << "[minSnapTraj]: No Desired Velocity Parameter. Use default: 1.0 m/s." << endl;
		}	
		else{
			cout << "[minSnapTraj]: Desired Velocity: " << this->desiredVel_ << " m/s." << endl;
		}

		// Desired Acceleration
		if (not declareParam("poly_traj/desired_acceleration", 1.0, this->desiredAcc_)){
			this->desiredAcc_ = 1.0;
			cout << "[minSnapTraj]: No Desired Acceleration Parameter. Use default: 1.0 m/s^2." << endl;
		}	
		else{
			cout << "[minSnapTraj]: Desired Acceleration: " << this->desiredAcc_ << " m/s^2." << endl;
		}

		if (not declareParam("poly_traj/initial_radius", 0.5, this->initR_)){
			this->initR_ = 0.5;
			cout << "[minSnapTraj]: No Initial Corridor Radius Parameter. Use default: 0.5 m" << endl;
		} 
		else{
			cout << "[minSnapTraj]: Corridor Radius: " << this->initR_ << " m." << endl;
		}

		// Max Solving Time (Iteratively generating collision free trajectory)
		if (not declareParam("poly_traj/timeout", 0.1, this->timeout_)){
			this->timeout_ = 0.1;
			cout << "[minSnapTraj]: No Timeout Parameter. Use default: 0.1." << endl;
		}
		else{
			cout << "[minSnapTraj]: Time out is set to: " << this->timeout_ << " s." << endl;
		}

		// Corridor Resolution
		if (not declareParam("poly_traj/corridor_res", 5.0, this->corridorRes_)){
			this->corridorRes_ = 5.0;
			cout << "[Trajectory Planenr INFO]: No Corridor Resolution Parameter. Use default: 5.0." << endl;
		}
		else{
			cout << "[minSnapTraj]: Corridor Resolution: " << this->corridorRes_ << endl;
		}

		// Factor of shrinking (Corridor size)
		if (not declareParam("poly_traj/shrinking_factor", 0.8, this->fs_)){
			this->fs_ = 0.8;
			cout << "[minSnapTraj]: No Shrinking Factor Parameter. Use default: 0.8." << endl; 
		}
		else{
			cout << "[minSnapTraj]: Shrinking Factor: " << this->fs_ << endl;
		}

		// use soft constraint or not
		if (not declareParam("poly_traj/soft_constraint", false, this->softConstraint_)){
			this->softConstraint_ = false;
			cout << "[minSnapTraj]: No Soft Constraint Parameter. Use default: false." << endl;
		} 


		if (this->softConstraint_){
			if (not declareParam("poly_traj/constraint_radius", 0.5, this->softConstraintRadius_)){
				this->softConstraintRadius_ = 0.5;
				cout << "[minSnapTraj]: No Soft Constraint Radius Parameter. Use default: 0.5 m." << endl;
			}
		}

		// delta T: for collision checking
		if (not declareParam("poly_traj/sample_delta_time", 0.1, this->delT_)){
			this->delT_ = 0.1;
			cout << "[minSnapTraj]: No Sample Time Parameter. Use default: 0.1." << endl;
		}
		else{
			cout << "[minSnapTraj]: Sample Time: " << this->delT_ << endl;
		}

		// Max Iteration (Iteratively generating collision free trajectory)
		if (not declareParam("poly_traj/maximum_iteration_num", 20, this->maxIter_)){
			this->maxIter_ = 20;
			cout << "[minSnapTraj]: No Maximum Iteration Parameter. Use default: 20." << endl;
		}
		else{
			cout << "[minSnapTraj]: Maximum Iteration Number: " << this->maxIter_ << endl;
		}

		// use pwl traj as failsafe
		if (not declareParam("poly_traj/use_pwl_failsafe", false, this->usePWL_)){
			this->usePWL_ = false;
			cout << "[minSnapTraj]: No use pwl option. Use default: not use." << endl;
		}
 	}

	void polyTrajOccMap::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
	}

	void polyTrajOccMap::registerPub(){
		this->trajVisPub_ = this->node_->create_publisher<nav_msgs::msg::Path>("poly_traj/tajectory", rclcpp::QoS(10));
	}

	void polyTrajOccMap::registerCallback(){
		this->visCallbackGroup_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		this->visTimer_ = this->node_->create_wall_timer(
			std::chrono::duration<double>(0.1),
			std::bind(&polyTrajOccMap::visCB, this),
			this->visCallbackGroup_);
	}

	void polyTrajOccMap::initSolver(){
		this->trajSolver_.reset(new trajPlanner::polyTrajSolver (this->polyDegree_, this->diffDegree_, this->continuityDegree_, this->desiredVel_));
	}

	void polyTrajOccMap::initPWLSolver(){
		this->pwlTrajSolver_.reset(new trajPlanner::pwlTraj (this->node_));
	}

	void polyTrajOccMap::updateDesiredVel(double desiredVel){
		this->desiredVel_ = desiredVel;
	}

	void polyTrajOccMap::updateDesiredAcc(double desiredAcc){
		this->desiredAcc_ = desiredAcc;
	}

	void polyTrajOccMap::updatePath(const nav_msgs::msg::Path& path){
		std::vector<trajPlanner::pose> trajPath;
		for (const geometry_msgs::msg::PoseStamped &p : path.poses){
			trajPlanner::pose trajP (p.pose.position.x, p.pose.position.y, p.pose.position.z);
			trajPath.push_back(trajP);
		}
		this->updatePath(trajPath);
	}

	void polyTrajOccMap::updatePath(const nav_msgs::msg::Path& path, const std::vector<Eigen::Vector3d>& startEndCondition){
		this->updatePath(path);
		Eigen::Vector3d startVel = startEndCondition[0];
		Eigen::Vector3d endVel = startEndCondition[1];
		Eigen::Vector3d startAcc = startEndCondition[2];
		Eigen::Vector3d endAcc = startEndCondition[3];
		this->updateInitVel(startVel(0), startVel(1), startVel(2));
		this->updateEndVel(endVel(0), endVel(1), endVel(2));
		this->updateInitAcc(startAcc(0), startAcc(1), startAcc(2));
		this->updateEndAcc(endAcc(0), endAcc(1), endAcc(2));
	}


	void polyTrajOccMap::updatePath(const std::vector<pose>& path){
		this->path_ = path;
	}

	void polyTrajOccMap::updateInitVel(double vx, double vy, double vz){
		geometry_msgs::msg::Twist v;
		v.linear.x = vx;
		v.linear.y = vy;
		v.linear.z = vz;
		this->updateInitVel(v);
	}

	void polyTrajOccMap::updateInitVel(const geometry_msgs::msg::Twist& v){
		this->initVel_ = v;
	}

	void polyTrajOccMap::updateEndVel(double vx, double vy, double vz){
		geometry_msgs::msg::Twist v;
		v.linear.x = vx;
		v.linear.y = vy;
		v.linear.z = vz;
		this->updateEndVel(v);
	}

	void polyTrajOccMap::updateEndVel(const geometry_msgs::msg::Twist& v){
		this->endVel_ = v;
	}

	void polyTrajOccMap::updateInitAcc(double ax, double ay, double az){
		geometry_msgs::msg::Twist a;
		a.linear.x = ax;
		a.linear.y = ay;
		a.linear.z = az;
		this->updateInitAcc(a);
	}


	void polyTrajOccMap::updateInitAcc(const geometry_msgs::msg::Twist& a){
		this->initAcc_ = a;
	}

	void polyTrajOccMap::updateEndAcc(double ax, double ay, double az){
		geometry_msgs::msg::Twist a;
		a.linear.x = ax;
		a.linear.y = ay;
		a.linear.z = az;
		this->updateEndAcc(a);
	}


	void polyTrajOccMap::updateEndAcc(const geometry_msgs::msg::Twist& a){
		this->endAcc_ = a;
	}

	void polyTrajOccMap::setDefaultInit(){
		// initialize 
		this->updateInitVel(0, 0, 0);
		this->updateEndVel(0, 0, 0);
		this->updateInitAcc(0, 0, 0); 
		this->updateEndAcc(0, 0, 0);
	}

	bool polyTrajOccMap::makePlan(bool corridorConstraint){
		nav_msgs::msg::Path dummyPath;
		return this->makePlan(dummyPath, corridorConstraint);
	}

	bool polyTrajOccMap::makePlan(std::vector<pose>& trajectory){
		this->findValidTraj_ = false;
		if (this->path_.size() == 1){
			trajectory = this->path_;
			this->findValidTraj_ = true;
			return true;
		}

		this->initSolver(); // polynomial solver
		this->trajSolver_->updatePath(this->path_);
		this->trajSolver_->updateInitVel(this->initVel_);
		this->trajSolver_->updateEndVel(this->endVel_);
		this->trajSolver_->updateInitAcc(this->initAcc_);
		this->trajSolver_->updateEndAcc(this->endAcc_);

		double corridorSize = this->initR_;
		std::vector<double> corridorSizeVec (this->path_.size()-1, corridorSize);

		int countIter = 0;
		bool valid = false;
		rclcpp::Time startTime = this->clock_->now();
		rclcpp::Time endTime;
		while (rclcpp::ok() and not valid){
			endTime = this->clock_->now();
			if ((endTime - startTime).seconds() >= this->timeout_){
				cout << "[minSnapTraj]: Timeout!" << endl;
				break;
			}
			this->trajSolver_->setCorridorConstraint(corridorSizeVec, this->corridorRes_);
			if (this->softConstraint_){
				this->trajSolver_->setSoftConstraint(this->softConstraint_, this->softConstraint_, 0); // no z direction soft constraint
			}
			this->trajSolver_->solve();
			this->trajSolver_->getTrajectory(trajectory, this->delT_);

			std::set<int> collisionSeg;
			valid = not this->checkCollisionTraj(trajectory, this->delT_, collisionSeg);

			if (not valid){
				this->adjustCorridorSize(collisionSeg, corridorSizeVec);
			}
			++countIter;
			if (countIter > this->maxIter_){
				break;
			}
		}


		if (valid){
			// cout << "[minSnapTraj]: Found valid trajectory!" << endl;	
			this->findValidTraj_ = true;
		}
		else{
			cout << "[minSnapTraj]: Not found. Return the best. Please consider piecewise linear trajectory!!" << endl;	
			if (this->usePWL_){
				this->pwlTrajSolver_->updatePath(this->path_);
				this->pwlTrajSolver_->makePlan(trajectory, this->delT_);
			}
		}
		this->trajMsgConverter(trajectory, this->trajVisMsg_);

		if (valid){
			return true;
		}
		else{
			return false;
		}
	}

	bool polyTrajOccMap::makePlan(std::vector<pose>& trajectory, bool corridorConstraint){
		this->findValidTraj_ = false;
		if (this->path_.size() == 1){
			trajectory = this->path_;
			this->findValidTraj_ = true;
			return true;
		}

		this->initSolver(); // polynomial solver
		this->trajSolver_->updatePath(this->path_);
		this->trajSolver_->updateInitVel(this->initVel_);
		this->trajSolver_->updateEndVel(this->endVel_);
		this->trajSolver_->updateInitAcc(this->initAcc_);
		this->trajSolver_->updateEndAcc(this->endAcc_);

		double corridorSize = this->initR_;
		std::vector<double> corridorSizeVec (this->path_.size()-1, corridorSize);

		int countIter = 0;
		bool valid = false;
		rclcpp::Time startTime = this->clock_->now();
		rclcpp::Time endTime;
		while (rclcpp::ok() and not valid){
			endTime = this->clock_->now();
			if ((endTime - startTime).seconds() >= this->timeout_){
				cout << "[minSnapTraj]: Timeout!" << endl;
				break;
			}

			if (corridorConstraint){
				this->trajSolver_->setCorridorConstraint(corridorSizeVec, this->corridorRes_);
				if (this->softConstraint_){
					this->trajSolver_->setSoftConstraint(this->softConstraint_, this->softConstraint_, 0); // no z direction soft constraint
				}
				this->trajSolver_->solve();
				this->trajSolver_->getTrajectory(trajectory, this->delT_);

				std::set<int> collisionSeg;
				valid = not this->checkCollisionTraj(trajectory, this->delT_, collisionSeg);
				if (not valid){
					this->adjustCorridorSize(collisionSeg, corridorSizeVec);
				}
				++countIter;
				if (countIter > this->maxIter_){
					break;
				}
			}
			else{
				this->trajSolver_->solve();
				this->trajSolver_->getTrajectory(trajectory, this->delT_);				
				valid = true;
			}
		}

		if (valid){
			// cout << "[minSnapTraj]: Found valid trajectory!" << endl;	
			this->findValidTraj_ = true;
		}
		else{
			cout << "[minSnapTraj]: Not found. Return the best. Please consider piecewise linear trajectory!!" << endl;	
			if (this->usePWL_){
				this->pwlTrajSolver_->updatePath(this->path_);
				this->pwlTrajSolver_->makePlan(trajectory, this->delT_);
			}
		}
		this->trajMsgConverter(trajectory, this->trajVisMsg_);

		if (valid){
			return true;
		}
		else{
			return false;
		}
	}

	bool polyTrajOccMap::makePlan(nav_msgs::msg::Path& trajectory){
		std::vector<pose> trajTemp;
		bool valid = this->makePlan(trajTemp);
		this->trajMsgConverter(trajTemp, trajectory);
		if (valid){
			return true;
		}
		else{
			return false;
		}
	}

	bool polyTrajOccMap::makePlan(nav_msgs::msg::Path& trajectory, bool corridorConstraint){
		std::vector<pose> trajTemp;
		bool valid = this->makePlan(trajTemp, corridorConstraint);
		this->trajMsgConverter(trajTemp, trajectory);
		if (valid){
			return true;
		}
		else{
			return false;
		}
	}

	void polyTrajOccMap::visCB(){
		this->publishTrajVis();
	}

	void polyTrajOccMap::publishTrajVis(){
		this->trajVisPub_->publish(this->trajVisMsg_);
	}


	nav_msgs::msg::Path polyTrajOccMap::getTrajectory(double dt){
		nav_msgs::msg::Path trajectory;
		trajectory.header.frame_id = "drone0/map";
		for (double t=0; t<=this->getDuration(); t+=dt){
			Eigen::Vector3d pos = this->getPos(t);
			geometry_msgs::msg::PoseStamped ps;
			ps.pose.position.x = pos(0);
			ps.pose.position.y = pos(1);
			ps.pose.position.z = pos(2);
			trajectory.poses.push_back(ps);
		}
		return trajectory;
	}

	geometry_msgs::msg::PoseStamped polyTrajOccMap::getPose(double t){
		if (t > this->getDuration()){
			t = this->getDuration();
		}

		if (this->usePWL_){
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
		
		geometry_msgs::msg::PoseStamped ps;
		trajPlanner::pose p = this->trajSolver_->getPose(t);
		ps.pose.position.x = p.x;
		ps.pose.position.y = p.y;
		ps.pose.position.z = p.z;
		geometry_msgs::msg::Quaternion quat = quaternion_from_rpy(0, 0, p.yaw);
		ps.pose.orientation = quat;
		ps.header.stamp = this->clock_->now();
		ps.header.frame_id = "drone0/map";

		return ps;
	}

	Eigen::Vector3d polyTrajOccMap::getPos(double t){
		if (t > this->getDuration()){
			t = this->getDuration();
		}

		return this->trajSolver_->getPos(t);		
	}

	Eigen::Vector3d polyTrajOccMap::getVel(double t){
		if (t > this->getDuration()){
			t = this->getDuration();
		}

		return this->trajSolver_->getVel(t);		
	}

	Eigen::Vector3d polyTrajOccMap::getAcc(double t){
		if (t > this->getDuration()){
			t = this->getDuration();
		}

		return this->trajSolver_->getAcc(t);		
	}

	double polyTrajOccMap::getDuration(){
		if (this->path_.size() == 1){
			return 0.0; // no duration
		}
		if (this->usePWL_){
			if (this->findValidTraj_){
				return this->trajSolver_->getTimeKnot().back();
			}
			else{
				return this->pwlTrajSolver_->getTimeKnot().back();
			}
		}

		return this->trajSolver_->getTimeKnot().back();
	}

	bool polyTrajOccMap::checkCollisionTraj(const std::vector<pose>& trajectory, double delT, std::set<int>& collisionSeg){
		collisionSeg.clear();
		std::vector<double> timeKnot = this->trajSolver_->getTimeKnot();
		double t = 0;
		bool hasCollision = false;
		double startTime, endTime;
		for (pose p : trajectory){
			Eigen::Vector3d pEig (p.x, p.y, p.z);
			if (this->map_->isInflatedOccupied(pEig) and this->map_->isUnknown(pEig)){
				hasCollision = true;
				for (size_t i=0; i<timeKnot.size()-1; ++i){
					startTime = timeKnot[i];
					endTime = timeKnot[i+1];
					if ((t>=startTime) and (t<=endTime)){
						collisionSeg.insert(i);
						break;
					}
				}
			}
			t += delT;
		}
		return hasCollision;
	}

	void polyTrajOccMap::adjustCorridorSize(const std::set<int>& collisionSeg, std::vector<double>& corridorSizeVec){
		for (int collisionIdx: collisionSeg){
			corridorSizeVec[collisionIdx] = corridorSizeVec[collisionIdx] * this->fs_; 
		}
	}

	void polyTrajOccMap::trajMsgConverter(const std::vector<trajPlanner::pose>& trajectoryTemp, nav_msgs::msg::Path& trajectory){
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