/*
	FILE: dynamicInspection.h
	-----------------------------
	header of dynamic inspection
*/
#ifndef DYNAMIC_INSPECTION
#define DYNAMIC_INSPECTION
#include <autonomous_flight/px4/flightBase.h>
#include <map_manager/dynamicMap.h>
#include <global_planner/rrtOccMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <functional>


namespace AutoFlight{
	
	enum FLIGHT_STATE {FORWARD, EXPLORE, INSPECT, BACKWARD};

	class dynamicInspection : flightBase{
	private:
		rclcpp::Node::SharedPtr node_;
		rclcpp::TimerBase::SharedPtr plannerTimer_;
		rclcpp::TimerBase::SharedPtr trajExeTimer_;
		rclcpp::TimerBase::SharedPtr checkWallTimer_;
		rclcpp::TimerBase::SharedPtr collisionCheckTimer_;
		rclcpp::TimerBase::SharedPtr replanTimer_;
		rclcpp::TimerBase::SharedPtr visTimer_;
		rclcpp::TimerBase::SharedPtr inspectTimer_;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rrtPathPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr polyTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pwlTrajPub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bsplineTrajPub_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wallVisPub_;

		// callback groups
		rclcpp::CallbackGroup::SharedPtr plannerCbGroup_;
		rclcpp::CallbackGroup::SharedPtr trajExeCbGroup_;
		rclcpp::CallbackGroup::SharedPtr checkWallCbGroup_;
		rclcpp::CallbackGroup::SharedPtr collisionCbGroup_;
		rclcpp::CallbackGroup::SharedPtr replanCbGroup_;
		rclcpp::CallbackGroup::SharedPtr visCbGroup_;

		// Map
		std::shared_ptr<mapManager::dynamicMap> map_;

		// Planner
		std::shared_ptr<globalPlanner::rrtOccMap<3>> rrtPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;


		// state
		FLIGHT_STATE flightState_ = FLIGHT_STATE::FORWARD;
		FLIGHT_STATE prevState_ = FLIGHT_STATE::FORWARD;
		geometry_msgs::msg::PoseStamped goal_;

		enum class InspectPhase {IDLE, CHECK_SURROUNDINGS, MOVE_TO_GOAL, ORIENT_GOAL, ZIGZAG, FRINGE, COMPLETE};
		enum class CheckSurroundState {INIT, LEFT_CHECK, RIGHT_CHECK, CENTER_MOVE, NEXT_HEIGHT};
		InspectPhase inspectPhase_ = InspectPhase::IDLE;
		CheckSurroundState checkState_ = CheckSurroundState::INIT;
		bool inspectionActive_ = false;
		bool actionActive_ = false;
		geometry_msgs::msg::PoseStamped actionGoal_;
		bool actionUseYaw_ = false;
		rclcpp::Time actionStartTime_;
		double actionTimeoutSec_ = 0.0;
		std::function<void(bool)> actionDoneCb_;
		bool lookAroundActive_ = false;
		int lookAroundStep_ = 0;
		bool backwardTurnPending_ = false;
		bool bsplineFailureWaitActive_ = false;
		rclcpp::Time bsplineFailureWaitUntil_;

		// inspection sequence bookkeeping
		size_t inspectionGoalIdx_ = 0;
		std::vector<double> checkHeights_;
		size_t checkHeightIdx_ = 0;
		Eigen::Vector3d checkLeftEnd_;
		Eigen::Vector3d checkRightEnd_;
		bool checkLeftSuccess_ = false;
		bool checkRightSuccess_ = false;
		bool zigzagPlanned_ = false;
		int fringeStep_ = 0;
		bool fringePlanned_ = false;
		nav_msgs::msg::Path fringePath1_;
		nav_msgs::msg::Path fringePath2_;
		double fringeDuration1_ = 0.0;
		double fringeDuration2_ = 0.0;
		geometry_msgs::msg::PoseStamped fringeGoal1_;
		geometry_msgs::msg::PoseStamped fringeGoal2_;

		// inspection parameters
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		double inspectionVel_; 
		double minWallArea_;
		double safeDistance_;
		double sideSafeDistance_;
		std::vector<double> inspectionHeights_;
		double inspectionHeight_;
		double ascendStep_;
		double descendStep_;
		double sensorRange_;
		double sensorAngleH_;
		double sensorAngleV_;
		int exploreSampleNum_;
		// ***only used when we specify location***
		bool inspectionGoalGiven_ = false;
		std::vector<Eigen::Vector3d> inspectionGoals_;
		Eigen::Vector3d inspectionGoal_;
		std::vector<double> inspectionOrientations_; 
		double inspectionOrientation_;
		bool inspectionWidthGiven_ = false;
		std::vector<double> inspectionWidths_;
		double inspectionWidth_;
		bool zigzagInspection_;
		bool fringeInspection_;
		bool leftFirst_ = true;
		double confirmMaxAngle_;
		bool inspectionConfirm_;
		bool backwardNoTurn_;
		double replanTimeForDynamicObstacle_;
		double collisionReplanCooldownSec_{0.30};
		bool operatorConfirm_ = false;
		// ***only used when we specify location***

		// inspection data
		bool trajValid_ = false;
		AutoFlight::trajData td_;
		bool useYaw_ = false;
		std::vector<double> wallRange_;
		bool wallDetected_ = false;
		nav_msgs::msg::Path rrtPathMsg_;
		nav_msgs::msg::Path polyTrajMsg_;
		nav_msgs::msg::Path pwlTrajMsg_;
		nav_msgs::msg::Path bsplineTrajMsg_;
		visualization_msgs::msg::MarkerArray wallVisMsg_;
		bool trajectoryReady_ = false;
		bool replan_ = true;
		rclcpp::Time trajStartTime_;
		double trajTime_; // current trajectory time
		trajPlanner::bspline trajectory_; // trajectory data for navigation
		int countBsplineFailure_ = 0;
		rclcpp::Time lastDynamicObstacleTime_;
		double lastCollisionReplanSec_{-1.0};
		std::mutex navStateMutex_;

	public:
		dynamicInspection();
		explicit dynamicInspection(const rclcpp::Node::SharedPtr& node);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();
		
		void run();

		void plannerCB();
		void trajExeCB();
		void checkWallCB(); // check whether the front wall is reached
		void collisionCheckCB(); // online collision checking
		void replanCB(); // replan callback
		void visCB();
		void inspectTimerCB();
		bool checkSurroundingsStep();
		void startAction(const nav_msgs::msg::Path& path, double duration, const geometry_msgs::msg::PoseStamped& goal, bool useYaw);
		void startActionAsync(const nav_msgs::msg::Path& path, double duration, const geometry_msgs::msg::PoseStamped& goal, bool useYaw, const std::function<void(bool)>& doneCb);
		void startYawAction(double yaw);
		bool isActionDone();
		bool finalizeActionIfDone(bool& success);
		void resetInspectionSequence();
		bool moveToPositionAsync(const geometry_msgs::msg::Point& position, double vel, const std::function<void(bool)>& doneCb);
		bool moveToOrientationAsync(const geometry_msgs::msg::Quaternion& orientation, const std::function<void(bool)>& doneCb);

		geometry_msgs::msg::PoseStamped getForwardGoal();
		nav_msgs::msg::Path getRestGlobalPath();
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndCondition);
		void changeState(const FLIGHT_STATE& flightState);


		// basic operations
		bool moveToPosition(const geometry_msgs::msg::Point& position);
		bool moveToPosition(const geometry_msgs::msg::Point& position, double vel);
		bool moveToPosition(const Eigen::Vector3d& position);
		bool moveToPosition(const Eigen::Vector3d& position, double vel);
		bool moveToOrientation(const geometry_msgs::msg::Quaternion& orientation);
		bool moveToOrientation(double yaw);
		bool moveToOrientationStep(double yaw);
		double makePWLTraj(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints, nav_msgs::msg::Path& resultPath);
		double makePWLTraj(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints, double desiredVel, nav_msgs::msg::Path& resultPath);
		double getPathLength(const nav_msgs::msg::Path& path);


		// exploration module
		bool getBestViewPoint(Eigen::Vector3d& bestPoint);
		bool randomSample(Eigen::Vector3d& pSample);
		bool satisfyWallDistance(const Eigen::Vector3d& p);
		int countUnknownFOV(const Eigen::Vector3d& p, double yaw);
		void setStartPositionFree();


		// wall detection module
		bool castRayOccupied(const Eigen::Vector3d& start, const Eigen::Vector3d& direction, Eigen::Vector3d& end, double maxRayLength);
		bool isWallDetected();
		double getWallDistance();
		void updateWallRange(const std::vector<double>& wallRange);
		visualization_msgs::msg::Marker getLineMarker(double x1, double y1, double z1, double x2, double y2, double z2, int id, bool isWall);
		void getWallVisMsg(visualization_msgs::msg::MarkerArray& msg);


		// inspection module
		void checkSurroundings();
		void inspectZigZag();
		void inspectZigZagRange();
		void inspectFringe();
		void inspectFringeRange();

		// navigation
		bool hasCollision();
		bool hasDynamicCollision();
		double computeExecutionDistance();
		bool replanForDynamicObstacle();
		nav_msgs::msg::Path getCurrentTraj(double dt);
		
		// utils
		geometry_msgs::msg::PoseStamped eigen2ps(const Eigen::Vector3d& p);
	};
}

#endif
