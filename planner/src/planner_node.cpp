#include "planner/planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode() : Node("planner_node")
{
  speed_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "speed_topic", 10, std::bind(&PlannerNode::speed_callback, this, std::placeholders::_1));

  distance_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "distance_topic", 10, std::bind(&PlannerNode::distance_callback, this, std::placeholders::_1));

  brake_distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>("brake_distance_topic", 10);

  RCLCPP_INFO(this->get_logger(), "Planner node is running");
  RCLCPP_INFO(this->get_logger(), "Súrlódási együttható (mu): %.2f", MU);
}

double PlannerNode::calculate_brake_distance(double speed_kmh)
{
  //Convert km/h to m/s
  double speed_ms = speed_kmh / 3.6;

  //s = v^2 / (2 * g * mu)
  double brake_distance = (speed_ms * speed_ms) / (2.0 * G * MU);

  return brake_distance;
}

void PlannerNode::speed_callback(const std_msgs::msg::Float32 & msg)
{
  current_speed_ = msg.data;

  double brake_distance = calculate_brake_distance(current_speed_);

  auto brake_msg = std_msgs::msg::Float32();
  brake_msg.data = brake_distance;
  brake_distance_publisher_->publish(brake_msg);

  RCLCPP_INFO(this->get_logger(), "Sebesség: %.2f km/h | Fékút: %.2f m", current_speed_, brake_distance);
}

void PlannerNode::distance_callback(const std_msgs::msg::Float32 & msg)
{
  current_distance_ = msg.data;
  RCLCPP_INFO(this->get_logger(), "Távolság az objektumig: %.2f m", current_distance_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
