#ifndef CONTROLLER__CONTROLLER_NODE_HPP_
#define CONTROLLER__CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  void subscriber_callback(const std_msgs::msg::String & msg);
};

#endif  // CONTROLLER__CONTROLLER_NODE_HPP_
