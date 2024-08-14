#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class ForwardKinematics : public rclcpp::Node
{
public:
  ForwardKinematics()
  : Node("forward_kinematics"), count_(0), x_(0.0), y_(0.0), theta_(0.0), v_l(0.0), v_r(0.0)
  {
    subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ForwardKinematics::sub_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&ForwardKinematics::timer_callback, this));
  }

private:
  void sub_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    v_l = msg->linear.x - (msg->angular.z * L / 2);
    v_r = msg->linear.x + (msg->angular.z * L / 2);
  }

  void timer_callback()
  {
    double dt = 0.1;

    // Calculate velocities
    double v = (v_r + v_l) / 2.0;
    double omega = (v_r - v_l) / L;

    // Update the pose
    theta_ += omega * dt;
    x_ += v * std::cos(theta_) * dt;
    y_ += v * std::sin(theta_) * dt;

    // Create and publish the odometry message
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.z = sin(theta_ / 2.0);
    odom.pose.pose.orientation.w = cos(theta_ / 2.0);

    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = omega;

    RCLCPP_INFO(this->get_logger(), "Pose x: '%f', y: '%f', theta: '%f'", x_, y_, theta_);

    publisher_->publish(odom);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  size_t count_;
  double x_, y_, theta_;
  double v_l, v_r;
  const double L = 1.0;  // Distance between wheels
  const double R = 0.5;  // Radius of the wheels
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForwardKinematics>());
  rclcpp::shutdown();
  return 0;
}
