#include "planner/planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode() : Node("planner_node")
{
  // 1. Feliratkozás az Ego sebességre
  speed_subscription_ = this->create_subscription<crp_msgs::msg::Ego>(
      "ego", 10, std::bind(&PlannerNode::speed_callback, this, std::placeholders::_1));

  // 2. Távolság figyelése
  distance_subscription_ = this->create_subscription<crp_msgs::msg::Scenario>(
      "scenario", 10, std::bind(&PlannerNode::distance_callback, this, std::placeholders::_1));

  // 3. Publikálás a BehaviorNode felé
  brake_distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>("brake_distance_topic", 10);

  // Idő inicializálása
  last_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Planner node is running with Jerk calculation");
}

double PlannerNode::calculate_brake_distance(double speed)
{
  // v^2 / (2 * g * mu)
  return (speed * speed) / (2.0 * G * MU);
}

void PlannerNode::speed_callback(const crp_msgs::msg::Ego::SharedPtr msg)
{
  current_speed_ = msg->twist.twist.linear.x;
  rclcpp::Time current_time = this->now();
  
  // --- JERK SZÁMÍTÁSA ---
  double dt = (current_time - last_time_).seconds();

  if (!first_run_ && dt > 0.0001) // Megvárjuk a második üzenetet és kerüljük a 0-val osztást
  {
    // Gyorsulás: a = (v_most - v_múlt) / dt
    double current_accel = (current_speed_ - last_speed_) / dt;
    
    // Jerk: j = (a_most - a_múlt) / dt
    double current_jerk = (current_accel - last_accel_) / dt;

    RCLCPP_INFO(this->get_logger(), "Gyorsulás: %.2f m/s² | Jerk: %.2f m/s³", 
                 current_accel, current_jerk);

    last_accel_ = current_accel;
  }
  else
  {
    first_run_ = false;
  }

  // Értékek mentése a következő ciklushoz
  last_speed_ = current_speed_;
  last_time_ = current_time;
  // -----------------------

  // Fékút számítása és publikálása
  double brake_distance = calculate_brake_distance(current_speed_);
  auto brake_msg = std_msgs::msg::Float32();
  brake_msg.data = static_cast<float>(brake_distance);
  brake_distance_publisher_->publish(brake_msg);

  RCLCPP_INFO(this->get_logger(), "Sebesség: %.2f m/s | Szükséges fékút: %.2f m", 
              current_speed_, brake_distance);
}

void PlannerNode::distance_callback(const crp_msgs::msg::Scenario::SharedPtr msg)
{
  if (!msg->local_moving_objects.objects.empty()) 
  {
    current_distance_ = msg->local_moving_objects.objects[0].kinematics.initial_pose_with_covariance.pose.position.x;
    RCLCPP_INFO(this->get_logger(), "Távolság az objektumig: %.2f m", current_distance_);
  } 
  else 
  {
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