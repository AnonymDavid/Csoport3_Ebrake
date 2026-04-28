#ifndef BEHAVIOR__BEHAVIOR_NODE_HPP_
#define BEHAVIOR__BEHAVIOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
// Új függőség a szimulátor környezeti adataihoz
#include "crp_msgs/msg/scenario.hpp"
#include "tier4_planning_msgs/msg/scenario.hpp"

class BehaviorNode : public rclcpp::Node
{
public:
  explicit BehaviorNode();

private:

  rclcpp::Publisher<tier4_planning_msgs::msg::Scenario>::SharedPtr strategy_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr brake_distance_subscription_;

  // ÚJ: Előfizetés a szimulátor Scenario topicjára a valós távolsághoz
  rclcpp::Subscription<crp_msgs::msg::Scenario>::SharedPtr scenario_subscription_;

  // Publikáló a ControllerNode felé (Fék parancs: true/false)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_command_publisher_;

  // Belső változók az adatok tárolására
  double calculated_brake_distance_ = 0.0;
  double actual_distance_ = 100.0;
  bool should_brake_ = false;

  // Callback függvények
  void brake_distance_callback(const std_msgs::msg::Float32::SharedPtr msg);
  
  // ÚJ: Scenario callback deklarációja
  void scenario_callback(const crp_msgs::msg::Scenario::SharedPtr msg);

  // Logikai döntéshozó függvény
  void check_safety();
};

#endif  // BEHAVIOR__BEHAVIOR_NODE_HPP_