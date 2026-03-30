#include "behavior/behavior_node.hpp"

BehaviorNode::BehaviorNode() : Node("behavior_node")
{
  brake_distance_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "brake_distance_topic", 10, std::bind(&BehaviorNode::brake_distance_callback, this, std::placeholders::_1));

  distance_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "distance_topic", 10, std::bind(&BehaviorNode::distance_callback, this, std::placeholders::_1));

  brake_command_publisher_ = this->create_publisher<std_msgs::msg::Bool>("brake_command_topic", 10);

  RCLCPP_INFO(this->get_logger(), "Behavior node is running");
}

void BehaviorNode::brake_distance_callback(const std_msgs::msg::Float32 & msg)
{
  brake_distance_ = msg.data;
  evaluate_brake_decision();
}

void BehaviorNode::distance_callback(const std_msgs::msg::Float32 & msg)
{
  object_distance_ = msg.data;
  evaluate_brake_decision();
}

void BehaviorNode::evaluate_brake_decision()
{
  bool previous_state = should_brake_;

  // if the object is detected and within the brake distance 
  if (object_distance_ > 0 && object_distance_ <= brake_distance_)
  {
    should_brake_ = true;
  }
  else
  {
    should_brake_ = false;
  }

  // we only publish if the brake command has changed or if there is an object detected
  if (should_brake_ != previous_state || object_distance_ > 0)
  {
    auto brake_msg = std_msgs::msg::Bool();
    brake_msg.data = should_brake_;
    brake_command_publisher_->publish(brake_msg);

    RCLCPP_INFO(this->get_logger(),
                "Távolság: %.2f m | Fékút: %.2f m | Fék: %s",
                object_distance_, brake_distance_, should_brake_ ? "IGEN" : "NEM");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
