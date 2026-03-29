#include "controller/controller_node.hpp"

ControllerNode::ControllerNode() : Node("controller_node")
{
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "input_topic", 10, std::bind(&ControllerNode::subscriber_callback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<std_msgs::msg::String>("output_topic", 10);
  RCLCPP_INFO(this->get_logger(), "Controller node is running");
}

void ControllerNode::subscriber_callback(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg.data.c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
