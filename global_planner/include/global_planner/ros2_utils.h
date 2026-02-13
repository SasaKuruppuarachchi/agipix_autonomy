#ifndef GLOBAL_PLANNER_ROS2_UTILS_H
#define GLOBAL_PLANNER_ROS2_UTILS_H

#include <rclcpp/rclcpp.hpp>

namespace globalPlanner {
	template <typename T>
	T declareAndGetParam(const rclcpp::Node::SharedPtr &node, const std::string &name, const T &default_value) {
		if (!node->has_parameter(name)) {
			node->declare_parameter<T>(name, default_value);
		}
		T value = default_value;
		node->get_parameter(name, value);
		return value;
	}
}

#endif
