#ifndef PLANNER__PLANNER_NODE_HPP_
#define PLANNER__PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "crp_msgs/msg/ego.hpp"
#include "crp_msgs/msg/scenario.hpp"
#include "std_msgs/msg/float32.hpp"

class PlannerNode : public rclcpp::Node
{
public:
  explicit PlannerNode();

private:
  static constexpr double G = 9.81;  // gravitációs gyorsulás (m/s^2)
  static constexpr double MU = 0.7;  // súrlódási együttható (száraz aszfalt)

  rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr speed_subscription_;
  rclcpp::Subscription<crp_msgs::msg::Scenario>::SharedPtr distance_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_distance_publisher_;

  double current_speed_ = 0.0;
  double current_distance_ = 0.0;
  double last_speed_ = 0.0;
  double last_accel_ = 0.0;
  rclcpp::Time last_time_;
  bool first_run_ = true;

  void speed_callback(const crp_msgs::msg::Ego::SharedPtr msg);
  void distance_callback(const crp_msgs::msg::Scenario::SharedPtr msg);
  double calculate_brake_distance(double speed_kmh);
};

#endif  // PLANNER__PLANNER_NODE_HPP_
