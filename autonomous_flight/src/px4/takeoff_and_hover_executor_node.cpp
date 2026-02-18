#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/components/node_with_mode.hpp>

#include <autonomous_flight/px4/flightBaseExecutor.h>

using ExecutorNode = px4_ros2::NodeWithModeExecutor<
  AutoFlight::flightBaseExecutor,
  AutoFlight::flightBaseTrackingMode>;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExecutorNode>("takeoff_and_hover_executor_node", true));
  rclcpp::shutdown();
  return 0;
}
