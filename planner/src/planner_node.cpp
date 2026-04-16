#include "planner/planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode() : Node("planner_node")
{
  // 1. Feliratkozás az Ego sebességre
  speed_subscription_ = this->create_subscription<crp_msgs::msg::Ego>(
      "ego", 10, std::bind(&PlannerNode::speed_callback, this, std::placeholders::_1));

  // 2. JAVÍTVA: A típus itt is crp_msgs::msg::Scenario kell legyen!
  distance_subscription_ = this->create_subscription<crp_msgs::msg::Scenario>(
      "scenario", 10, std::bind(&PlannerNode::distance_callback, this, std::placeholders::_1));

  // 3. Publikálás a BehaviorNode felé
  brake_distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>("brake_distance_topic", 10);

  RCLCPP_INFO(this->get_logger(), "Planner node is running");
  RCLCPP_INFO(this->get_logger(), "Súrlódási együttható (mu): %.2f", MU);
}

double PlannerNode::calculate_brake_distance(double speed)
{
  // v^2 / (2 * g * mu)
  double brake_distance = (speed * speed) / (2.0 * G * MU);
  return brake_distance;
}

void PlannerNode::speed_callback(const crp_msgs::msg::Ego::SharedPtr msg)
{
  // Adat kinyerése a komplex Ego üzenetből (m/s)
  current_speed_ = msg->twist.twist.linear.x;

  // Fékút számítása
  double brake_distance = calculate_brake_distance(current_speed_);

  // Publikálás Float32 formátumban
  auto brake_msg = std_msgs::msg::Float32();
  brake_msg.data = static_cast<float>(brake_distance);
  brake_distance_publisher_->publish(brake_msg);

  RCLCPP_INFO(this->get_logger(), "Sebesség: %.2f m/s | Szükséges fékút: %.2f m", 
              current_speed_, brake_distance);
}

void PlannerNode::distance_callback(const crp_msgs::msg::Scenario::SharedPtr msg)
{
  // Ellenőrizzük, hogy van-e észlelt objektum
  if (!msg->local_moving_objects.objects.empty()) 
  {
    // A legközelebbi objektum X pozíciója a távolság
    current_distance_ = msg->local_moving_objects.objects[0].kinematics.initial_pose_with_covariance.pose.position.x;
    
    RCLCPP_INFO(this->get_logger(), "Távolság az objektumig: %.2f m", current_distance_);
  } 
  else 
  {
    // Ha nincs objektum, 100 méter (biztonságos távolság)
    current_distance_ = 100.0;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}