#include "controller/controller_node.hpp"

ControllerNode::ControllerNode() : Node("controller_node")
{
  brake_command_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "brake_command_topic", 10, std::bind(&ControllerNode::brake_command_callback, this, std::placeholders::_1));

  brake_output_publisher_ = this->create_publisher<std_msgs::msg::Float32>("brake_output_topic", 10);

  RCLCPP_INFO(this->get_logger(), "Controller node is running");
}

void ControllerNode::brake_command_callback(const std_msgs::msg::Bool & msg)
{
  apply_brake(msg.data);
}

void ControllerNode::apply_brake(bool brake_active)
{
  is_braking_ = brake_active;

  auto brake_output = std_msgs::msg::Float32();

  if (brake_active)
  {
    brake_output.data = 1.0;  // 100% fékezés
    RCLCPP_WARN(this->get_logger(), "⚠️ FÉKEZ - AZ AUTÓ MEGÁLLT");
  }
  else
  {
    brake_output.data = 0.0;  // 0% fékezés
    RCLCPP_INFO(this->get_logger(), "✓ Normál működés");
  }

  brake_output_publisher_->publish(brake_output);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
