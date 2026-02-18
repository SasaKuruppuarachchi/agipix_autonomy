#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("offboard_mode_node_deprecated");
    RCLCPP_WARN(
        node->get_logger(),
        "[AutoFlight]: offboard_mode_node is deprecated and disabled. Use DDS path via px4_control_interface.");

    rclcpp::shutdown();
    return 0;
}

