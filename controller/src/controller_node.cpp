#include "controller/controller_node.hpp"

ControllerNode::ControllerNode() : Node("controller_node")
{
  // Feliratkozás a BehaviorNode parancsára
  // Fontos: a név egyezzen a BehaviorNode-ban megadottal
  brake_command_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
    "brake_command_topic", 10, std::bind(&ControllerNode::brake_command_callback, this, std::placeholders::_1));

  // Publikálás a szimulátornak
  // KRITIKUS: A perjel az elején (/control/...) biztosítja, hogy pontosan ezt a topicot találja meg
  brake_output_publisher_ = this->create_publisher<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", 10);

  RCLCPP_INFO(this->get_logger(), "Controller node elindult.");
}

void ControllerNode::brake_command_callback(const std_msgs::msg::Bool & msg)
{
  apply_brake(msg.data);
}

void ControllerNode::apply_brake(bool brake_active)
{
  is_braking_ = brake_active;
  auto control_msg = autoware_control_msgs::msg::Control();

  control_msg.stamp = this->now();

  if (brake_active)
  {
    // --- VÉSZFÉKEZÉS ÜZEMMÓD ---
    control_msg.lateral.steering_tire_angle = 0.0;
    
    // A szimulátor belső szabályozójának 0 célsebességet adunk
    control_msg.longitudinal.velocity = 0.0; 
    // Megadjuk a maximális lassulást
    control_msg.longitudinal.acceleration = -5.0; 
    
    RCLCPP_WARN(this->get_logger(), "⚠️ VÉSZFÉKEZÉS AKTIVÁLVA - Megállás parancs elküldve");
  }
  else
  {
    // --- NORMÁL ÜZEMMÓD ---
    // Az autó 15.0 m/s (kb. 54 km/h) sebességgel fog haladni, amíg nincs akadály
    control_msg.longitudinal.velocity = 15.0; 
    control_msg.longitudinal.acceleration = 0.0;
    control_msg.lateral.steering_tire_angle = 0.0;
  }

  brake_output_publisher_->publish(control_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}