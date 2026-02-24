#include <rclcpp/rclcpp.hpp>

#include <autonomous_flight/px4/rlNavigation.h>

namespace {
class RlNavigationRunner {
public:
  RlNavigationRunner(
    const rclcpp::Node::SharedPtr & node,
    AutoFlight::rlNavigation * navigation)
  : node_(node), navigation_(navigation)
  {
    start_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    start_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        start_timer_->cancel();
        RCLCPP_INFO(node_->get_logger(), "[AutoFlight]: Start rl_navigation mission.");
        navigation_->run();
      },
      start_cb_group_);
  }

private:
  rclcpp::Node::SharedPtr node_;
  AutoFlight::rlNavigation * navigation_;
  rclcpp::CallbackGroup::SharedPtr start_cb_group_;
  rclcpp::TimerBase::SharedPtr start_timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rl_navigation_node");

  AutoFlight::rlNavigation navigation(node);
  RlNavigationRunner runner(node, &navigation);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
