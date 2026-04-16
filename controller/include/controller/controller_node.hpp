#ifndef CONTROLLER__CONTROLLER_NODE_HPP_
#define CONTROLLER__CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "autoware_control_msgs/msg/control.hpp" // Ez kell a Float32 helyett

class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode();

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brake_command_subscription_;
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr brake_output_publisher_;

  bool is_braking_ = false;

  void brake_command_callback(const std_msgs::msg::Bool & msg);
  void apply_brake(bool brake_active);
};

#endif  // CONTROLLER__CONTROLLER_NODE_HPP_
