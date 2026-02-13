/*
	file: polyTrajOctomap.h
	-----------------------
	minimum snap polynomial trajectory planner based on ocromap
*/

#ifndef POLYTRAJPLANNER_H
#define POLYTRAJPLANNER_H
#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_planner/polyTrajSolver.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <thread>
#include <mutex>

namespace trajPlanner{
	class polyTrajOctomap{
	private:
		rclcpp::Node::SharedPtr node_;
		rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr mapClient_; // call octomap server
		rclcpp::Clock::SharedPtr clock_;
		rclcpp::Logger logger_{rclcpp::get_logger("polyTrajOctomap")};

		int polyDegree_; // polynomial degree
		double desiredVel_; // desired velocity
		int diffDegree_; // differential degree
		int continuityDegree_; // continuity degree
		double regularizationWeights_; // paramters regularization
		double mapRes_;
		int maxIter_;
		double timeout_;
		bool mode_; // collision avoidance mode. (True: adding waypoint, False: corridor constraint) 
		bool softConstraint_;
		double softConstraintRadius_;
		double delT_;
		double initR_;
		double fs_; //factor of shrink
		double corridorRes_;
		bool findValidTraj_; // find valid solution

		// initial condition
		geometry_msgs::msg::Twist initVel_;
		geometry_msgs::msg::Twist initAcc_;

		polyTrajSolver* trajSolver_; // trajectory solver
		pwlTraj* pwlTrajSolver_; // piecewise linear trajectory solver
		std::vector<pose> path_; // waypoint path
		std::vector<double> collisionBox_;
		octomap::OcTree* map_;

		// visualization:
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajVisPub_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr samplePointVisPub_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypointVisPub_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridorVisPub_;
		nav_msgs::msg::Path trajVisMsg_;
		visualization_msgs::msg::MarkerArray samplePointMsg_;
		visualization_msgs::msg::MarkerArray waypointMsg_;
		visualization_msgs::msg::MarkerArray corridorMsg_;
		
	public:
		std::thread trajVisWorker_;
		std::thread samplePointVisWorker_;
		std::thread waypointVisWorker_;
		std::thread corridorVisWorker_;

		polyTrajOctomap();

		polyTrajOctomap(const rclcpp::Node::SharedPtr& node);
		
		// update octomap
		void updateMap();

		// initialize solver
		polyTrajSolver*  initSolver();
		void freeSolver();
		pwlTraj* initPWLSolver();
		void freePWLSolver();

		// update waypoint path
		void updatePath(const nav_msgs::msg::Path& path);
		void updatePath(const std::vector<pose>& path);
		void insertWaypoint(const std::set<int>& seg);
		void adjustCorridorSize(const std::set<int>& collisionSeg, std::vector<double>& collisionVec);


		// Initial condition
		void updateInitVel(double vx, double vy, double vz);
		void updateInitVel(const geometry_msgs::msg::Twist& v);
		void updateInitAcc(double ax, double ay, double az);
		void updateInitAcc(const geometry_msgs::msg::Twist& a);
		void setDefaultInit();

		// CORE FUNCTION:
		void makePlan(); // no return. Get trajectory by this object
		void makePlan(nav_msgs::msg::Path& trajectory, double delT=0.1);
		void makePlan(std::vector<pose>& trajectory, double delT=0.1);
		void makePlanAddingWaypoint();
		void makePlanAddingWaypoint(std::vector<pose>& trajectory, double delT); // adding point for collision avoidance
		void makePlanCorridorConstraint();
		void makePlanCorridorConstraint(std::vector<pose>& trajectory, double delT); // adding corridor constraint for collision avoidance

		// collision checking
		bool checkCollision(const octomap::point3d& p);
		bool checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown=true);
		bool checkCollisionPoint(const pose& pTraj);
		bool checkCollisionLine(const octomap::point3d& p1, const octomap::point3d& p2);
		bool checkCollisionLine(const pose& pTraj1, const pose& pTraj2);
		bool checkCollisionTraj(const std::vector<pose>& trajectory, std::vector<int>& collisionIdx);
		bool checkCollisionTraj(const std::vector<pose>& trajectory, double delT, std::set<int>& collisionSeg);

		// get poses at specific time:
		geometry_msgs::msg::PoseStamped getPose(double t); 
		double getDuration();

		double getDegree();
		double getDiffDegree();
		double getContinuityDegree();
		double getDesiredVel();
		double getInitialRadius();
		double getShrinkFactor();

		// visualizaton:
		void updateTrajVisMsg(const std::vector<pose>& trajectory);
		void updateCorridorVisMsg(const std::vector<std::unordered_map<double, trajPlanner::pose>>& segToTimePose, const std::vector<double>& corridorSizeVec);
		void publishTrajectory();
		void publishSamplePoint();
		void publishWaypoint();
		void publishCorridor();

		// conversion helper:
		void pose2Octomap(const pose& pTraj, octomap::point3d& p);
		void trajMsgConverter(const std::vector<trajPlanner::pose>& trajectoryTemp, nav_msgs::msg::Path& trajectory);
	};

	inline std::ostream &operator<<(std::ostream &os, polyTrajOctomap& polyPlanner){
        os << "========================INFO========================\n";
        os << "[Trajectory Planner INFO]: minimum snap polynomial trajectory planner with octomap\n";
        os << "[Polynomial Degree]: " << polyPlanner.getDegree() << "\n";
        os << "[Differential Degree]: " << polyPlanner.getDiffDegree() << "\n";
        os << "[Continuity Degree]: " << polyPlanner.getContinuityDegree() << "\n";
        os << "[Desired Velocity]: " << polyPlanner.getDesiredVel() << "\n";
        os << "====================================================";
        return os;
    }
}

#endif