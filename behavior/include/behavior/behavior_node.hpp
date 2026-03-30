#ifndef BEHAVIOR__BEHAVIOR_NODE_HPP_
#define BEHAVIOR__BEHAVIOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

class BehaviorNode : public rclcpp::Node
{
public:
  explicit BehaviorNode();

private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr brake_distance_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_command_publisher_;

  double brake_distance_ = 0.0;
  double object_distance_ = 0.0;
  bool should_brake_ = false;

  void brake_distance_callback(const std_msgs::msg::Float32 & msg);
  void distance_callback(const std_msgs::msg::Float32 & msg);
  void evaluate_brake_decision();
};

#endif  // BEHAVIOR__BEHAVIOR_NODE_HPP_
