#include "planner/planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner_node")
{
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "input_topic", 10, std::bind(&PlannerNode::subscriber_callback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<std_msgs::msg::String>("output_topic", 10);
  RCLCPP_INFO(this->get_logger(), "Planner node is running");
}

void PlannerNode::subscriber_callback(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg.data.c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
