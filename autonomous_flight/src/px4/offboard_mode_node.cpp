/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>

static mavros_msgs::msg::State current_state;
static void state_cb(const mavros_msgs::msg::State::SharedPtr msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("offb_node");

    auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, state_cb);
    auto local_pos_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 10);
    auto arming_client = node->create_client<mavros_msgs::srv::CommandBool>(
            "mavros/cmd/arming");
    auto set_mode_client = node->create_client<mavros_msgs::srv::SetMode>(
            "mavros/set_mode");

    // the setpoint publishing rate MUST be faster than 2Hz
    rclcpp::Rate rate(20.0);

    // wait for FCU connection
    while(rclcpp::ok() && !current_state.connected){
        rclcpp::spin_some(node);
        rate.sleep();
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // send a few setpoints before starting
    for(int i = 100; rclcpp::ok() && i > 0; --i){
        local_pos_pub->publish(pose);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    auto offb_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    offb_set_mode->custom_mode = "OFFBOARD";

    auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_cmd->value = true;

    rclcpp::Time last_request = node->now();

    while(rclcpp::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (node->now() - last_request > rclcpp::Duration::from_seconds(5.0))){
            if (set_mode_client->service_is_ready()){
                set_mode_client->async_send_request(offb_set_mode,
                    [node](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future){
                        if (future.get()->mode_sent){
                            RCLCPP_INFO(node->get_logger(), "Offboard enabled");
                        }
                    });
            }
            last_request = node->now();
        } else {
            if( !current_state.armed &&
                (node->now() - last_request > rclcpp::Duration::from_seconds(5.0))){
                if (arming_client->service_is_ready()){
                    arming_client->async_send_request(arm_cmd,
                        [node](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future){
                            if (future.get()->success){
                                RCLCPP_INFO(node->get_logger(), "Vehicle armed");
                            }
                        });
                }
                last_request = node->now();
            }
        }

        local_pos_pub->publish(pose);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

