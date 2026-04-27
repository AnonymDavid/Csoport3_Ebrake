#include "behavior/behavior_node.hpp"

BehaviorNode::BehaviorNode() : Node("behavior_node")
{
  // 1. Feliratkozás a Planner által számolt fékútra (brake_distance_topic)
  brake_distance_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    "brake_distance_topic", 10, std::bind(&BehaviorNode::brake_distance_callback, this, std::placeholders::_1));

  // 2. Feliratkozás a szimulátor Scenario topicjára a VALÓS távolságért
  scenario_subscription_ = this->create_subscription<crp_msgs::msg::Scenario>(
    "scenario", 10, std::bind(&BehaviorNode::scenario_callback, this, std::placeholders::_1));

  // 3. Publikáló a ControllerNode felé
  brake_command_publisher_ = this->create_publisher<std_msgs::msg::Bool>("brake_command_topic", 10);

  RCLCPP_INFO(this->get_logger(), "Behavior node elindult és figyeli a távolságot.");
}

void BehaviorNode::brake_distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  calculated_brake_distance_ = msg->data;
  check_safety();
}

void BehaviorNode::scenario_callback(const crp_msgs::msg::Scenario::SharedPtr msg)
{
  // Megkeressük a legközelebbi objektumot a szimulátor üzenetéből
  if (!msg->local_moving_objects.objects.empty()) 
  {
    // A Simulator.cpp base_link-ben adja meg, az X a távolságunk
    actual_distance_ = msg->local_moving_objects.objects[0].kinematics.initial_pose_with_covariance.pose.position.x;
  } 
  else 
  {
    actual_distance_ = 100.0; // Ha nincs akadály, legyen messze
  }
  check_safety();
}

void BehaviorNode::check_safety()
{
  auto brake_msg = std_msgs::msg::Bool();

  // DÖNTÉS: Ha a távolság kisebb, mint a fékút + 5 méter biztonsági ráhagyás
  if (actual_distance_ < (calculated_brake_distance_ + 10.0)) 
  {
    brake_msg.data = true;
    if (!should_brake_) {
      RCLCPP_WARN(this->get_logger(), "Veszélyes közelség! Fékút: %.2f m | Távolság: %.2f m", 
                  calculated_brake_distance_, actual_distance_);
    }
    should_brake_ = true;
  } 
  else 
  {
    brake_msg.data = false;
    should_brake_ = false;
  }

  brake_command_publisher_->publish(brake_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}