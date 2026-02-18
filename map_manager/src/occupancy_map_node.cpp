/*
	FILE: occupancy_map_node.cpp
	--------------------------------------
	occupancy map ROS2 Node
*/
 #include "rclcpp/rclcpp.hpp"
 #include <map_manager/occupancyMap.h>

 
 static const std::string kNodeName = "occupancy_map_node";
 static const bool kEnableDebugOutput = true;
 
 int main(int argc, char * argv[])
 {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(kNodeName);
	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  	executor.add_node(node);
  	
	mapManager::occMap m(node);
	m.initMap();

	executor.spin(); //rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
 }
 