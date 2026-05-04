#include "behavior/behavior_node.hpp"

BehaviorNode::BehaviorNode() : Node("behavior_node")
{
  strategy_publisher_ = this->create_publisher<tier4_planning_msgs::msg::Scenario>("plan/strategy", 10);
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
  if (!msg->local_moving_objects.objects.empty()) 
  {
    actual_distance_ = msg->local_moving_objects.objects[0].kinematics.initial_pose_with_covariance.pose.position.x;
  } 
  else 
  {
    actual_distance_ = 100.0;
  }
  check_safety();
}

void BehaviorNode::check_safety()
{
  auto brake_msg = std_msgs::msg::Bool();
  tier4_planning_msgs::msg::Scenario strategy_msg;

  if (actual_distance_ < 5.0 || actual_distance_ < calculated_brake_distance_)
  {
    strategy_msg.current_scenario = "longEmergencyImpact";
    brake_msg.data = true;
    should_brake_ = true;
    emergency_triggered_ = true;
  }
  else if (actual_distance_ < (calculated_brake_distance_ + 5.0))
  {
    strategy_msg.current_scenario = "longEmergencyAvoid";
    brake_msg.data = true;
    should_brake_ = true;
    emergency_triggered_ = true;
  }
  else if (actual_distance_ < (calculated_brake_distance_ + 10.0))
  {
    if (emergency_triggered_)
    {
      strategy_msg.current_scenario = "longEmergencyAvoid";
      brake_msg.data = true;
      should_brake_ = true;
    }
    else
    {
      strategy_msg.current_scenario = "warningLevel";
      brake_msg.data = false;
      should_brake_ = false;
    }
  }
  else
  {
    strategy_msg.current_scenario = "noAction";
    brake_msg.data = false;
    should_brake_ = false;
  }

  brake_command_publisher_->publish(brake_msg);
  strategy_publisher_->publish(strategy_msg);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}