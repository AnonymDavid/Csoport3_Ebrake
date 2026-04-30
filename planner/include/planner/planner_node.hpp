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
  static constexpr double G = 9.81;
  static constexpr double MU = 0.7;
  static constexpr double JERK_SAFETY_FACTOR = 0.05; // [s] skálázó

  rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr speed_subscription_;
  rclcpp::Subscription<crp_msgs::msg::Scenario>::SharedPtr distance_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_distance_publisher_;

  double current_speed_  = 0.0;
  double current_distance_ = 0.0;
  double last_speed_     = 0.0;
  double last_accel_     = 0.0;   // előző ciklus gyorsulása (jerk számításhoz)
  double current_accel_  = 0.0;   // ← új tagváltozó
  double current_jerk_   = 0.0;   // ← új tagváltozó
  rclcpp::Time last_time_;
  bool first_run_ = true;

  void speed_callback(const crp_msgs::msg::Ego::SharedPtr msg);
  void distance_callback(const crp_msgs::msg::Scenario::SharedPtr msg);
  // Aláírás bővítve: gyorsulás + jerk is befolyásolja a fékutat
  double calculate_brake_distance(double speed, double accel, double jerk);
};

#endif  // PLANNER__PLANNER_NODE_HPP_