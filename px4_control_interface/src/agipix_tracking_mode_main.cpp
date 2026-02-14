#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/components/node_with_mode.hpp>

#include <px4_control_interface/agipix_tracking_mode.hpp>

using NodeWithTrackingMode = px4_ros2::NodeWithMode<px4_control_interface::AgipixTrackingMode>;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodeWithTrackingMode>("px4_tracking_mode_node", true));
  rclcpp::shutdown();
  return 0;
}
