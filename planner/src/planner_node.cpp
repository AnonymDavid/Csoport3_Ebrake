#include "planner/planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode() : Node("planner_node")
{
  speed_subscription_ = this->create_subscription<crp_msgs::msg::Ego>(
      "ego", 10, std::bind(&PlannerNode::speed_callback, this, std::placeholders::_1));

  distance_subscription_ = this->create_subscription<crp_msgs::msg::Scenario>(
      "scenario", 10, std::bind(&PlannerNode::distance_callback, this, std::placeholders::_1));

  brake_distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>("brake_distance_topic", 10);

  last_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Planner node is running with Jerk calculation");
}

double PlannerNode::calculate_brake_distance(double speed, double accel, double jerk)
{
  // 1) Alap fékút: v² / (2·g·μ)
  double base = (speed * speed) / (2.0 * G * MU);

  // 2) Gyorsulás-korrekció: ha a jármű gyorsul (a > 0), tovább halad mire
  //    a fék fejti ki hatását; ha már lassul (a < 0), a fékút rövidebb.
  //    Képlet: Δd_a = v·a / (g·μ)
  double accel_correction = (speed * accel) / (G * MU);

  // 3) Jerk-alapú biztonsági tartalék: nagy |jerk| → instabil dinamika
  //    (pl. hirtelen fékezés/gyorsítás) → extra margó
  //    Képlet: Δd_j = |j| · JERK_SAFETY_FACTOR
  double jerk_margin = std::abs(jerk) * JERK_SAFETY_FACTOR;

  double result = std::max(0.0, base + accel_correction + jerk_margin);

  RCLCPP_INFO(this->get_logger(),
    "Fékút összetevők → alap: %.2f m | gyorsulás-korr: %+.2f m | jerk-margó: %.2f m | összesen: %.2f m",
    base, accel_correction, jerk_margin, result);

  return result;
}

void PlannerNode::speed_callback(const crp_msgs::msg::Ego::SharedPtr msg)
{
  current_speed_ = msg->twist.twist.linear.x;
  rclcpp::Time current_time = this->now();

  double dt = (current_time - last_time_).seconds();

  if (!first_run_ && dt > 0.0001)
  {
    current_accel_ = (current_speed_ - last_speed_) / dt;
    current_jerk_  = (current_accel_ - last_accel_) / dt;

    RCLCPP_INFO(this->get_logger(), "Gyorsulás: %.2f m/s² | Jerk: %.2f m/s³",
                current_accel_, current_jerk_);

    last_accel_ = current_accel_;
  }
  else
  {
    first_run_ = false;
  }

  last_speed_ = current_speed_;
  last_time_  = current_time;

  // Fékút számítása — most már felhasználja a gyorsulást és a jerket
  double brake_distance = calculate_brake_distance(current_speed_, current_accel_, current_jerk_);

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
    current_distance_ = msg->local_moving_objects.objects[0]
                          .kinematics.initial_pose_with_covariance.pose.position.x;
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