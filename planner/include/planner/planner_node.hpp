#ifndef PLANNER__PLANNER_NODE_HPP_
#define PLANNER__PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PlannerNode : public rclcpp::Node
{
public:
  explicit PlannerNode();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  void subscriber_callback(const std_msgs::msg::String & msg);
};

#endif  // PLANNER__PLANNER_NODE_HPP_
