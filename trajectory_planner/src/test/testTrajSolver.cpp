#include <rclcpp/rclcpp.hpp>
#include <trajectory_planner/polyTrajSolver.h>
#include <trajectory_planner/utils.h>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::cout;
using std::endl;

class TestTrajSolverNode : public rclcpp::Node {
public:
	TestTrajSolverNode() : rclcpp::Node("test_solver") {
		trajVisPub_ = this->create_publisher<nav_msgs::msg::Path>("path_vis", 1);
		wpVisPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoint", 1);
		clock_ = this->get_clock();

		this->declare_parameter<int>("waypoint_num", 0);
	}

	void run(){
		cout << "[Test Solver]: test" << endl;
		std::vector<trajPlanner::pose> waypointPath;

		int numWaypoints = this->get_parameter("waypoint_num").as_int();
		cout << "[Test Solver]: Number of waypoints: " << numWaypoints << endl;
		if (numWaypoints < 2){
			RCLCPP_WARN(this->get_logger(), "Need at least 2 waypoints to solve trajectory. Check waypoint.yaml.");
			return;
		}

		std::vector<visualization_msgs::msg::Marker> waypointVec;
		int waypointCount = 0;
		for (int i=1; i<=numWaypoints; ++i){
			std::string waypoint_name = "waypoint_" + std::to_string(i);
			if (!this->has_parameter(waypoint_name)){
				this->declare_parameter<std::vector<double>>(waypoint_name, std::vector<double>{0.0, 0.0, 0.0});
			}
			std::vector<double> pVec = this->get_parameter(waypoint_name).as_double_array();
			if (pVec.size() < 3){
				RCLCPP_WARN(this->get_logger(), "Waypoint %s missing, skipping.", waypoint_name.c_str());
				continue;
			}
			trajPlanner::pose p (pVec[0], pVec[1], pVec[2]);
			cout << "[Test Solver]: " << p << endl;
			waypointPath.push_back(p);

			visualization_msgs::msg::Marker waypoint;
			waypoint.header.frame_id = "drone0/map";
			waypoint.header.stamp = clock_->now();
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
		waypointMsg_.markers = waypointVec;

		int polyDegree = 6;
		int diffDegree = 3;
		int continuityDegree = 2;
		double desiredVel = 1.0;

		double vx = -1.0;
		double vy = 0;
		double vz = 0;

		trajPlanner::polyTrajSolver solver (polyDegree, diffDegree, continuityDegree, desiredVel);
		solver.updateInitVel(vx, vy, vz);
		solver.updatePath(waypointPath);
		solver.solve();

		std::vector<trajPlanner::pose> trajectory;
		solver.getTrajectory(trajectory, 0.1);
		trajMsgConverter(trajectory, trajVis_);

		publishTimer_ = this->create_wall_timer(
			std::chrono::duration<double>(0.1),
			std::bind(&TestTrajSolverNode::publishVis, this));
	}

private:
	void trajMsgConverter(const std::vector<trajPlanner::pose>& trajectoryTemp, nav_msgs::msg::Path& trajectory){
		std::vector<geometry_msgs::msg::PoseStamped> trajVec;
		for (trajPlanner::pose pTemp: trajectoryTemp){
			geometry_msgs::msg::PoseStamped ps;
			ps.header.stamp = clock_->now();
			ps.header.frame_id = "drone0/map";
			ps.pose.position.x = pTemp.x;
			ps.pose.position.y = pTemp.y;
			ps.pose.position.z = pTemp.z;

			geometry_msgs::msg::Quaternion quat = trajPlanner::quaternion_from_rpy(0, 0, pTemp.yaw);
			ps.pose.orientation = quat;
			trajVec.push_back(ps);
		}
		trajectory.header.stamp = clock_->now();
		trajectory.header.frame_id = "drone0/map";
		trajectory.poses = trajVec;
	}

	void publishVis(){
		trajVisPub_->publish(trajVis_);
		wpVisPub_->publish(waypointMsg_);
	}

	rclcpp::Clock::SharedPtr clock_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajVisPub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wpVisPub_;
	rclcpp::TimerBase::SharedPtr publishTimer_;
	nav_msgs::msg::Path trajVis_;
	visualization_msgs::msg::MarkerArray waypointMsg_;
};

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TestTrajSolverNode>();
	node->run();
	rclcpp::executors::MultiThreadedExecutor exec;
	exec.add_node(node);
	exec.spin();
	rclcpp::shutdown();
	return 0;
}