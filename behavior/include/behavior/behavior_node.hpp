#ifndef BEHAVIOR__BEHAVIOR_NODE_HPP_
#define BEHAVIOR__BEHAVIOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class BehaviorNode : public rclcpp::Node
{
public:
  explicit BehaviorNode();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  void subscriber_callback(const std_msgs::msg::String & msg);
};

#endif  // BEHAVIOR__BEHAVIOR_NODE_HPP_
