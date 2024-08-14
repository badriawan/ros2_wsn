#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"


using namespace std::chrono_literals;

class GoalLinear : public rclcpp::Node
{
public:
  GoalLinear()
  : Node("goal_linear")
  {
    this->declare_parameter<double>("goal_x", 0.0);
    this->declare_parameter<double>("goal_y", 0.0);

    this->declare_parameter<double>("Kp", 0.5);

    // Corrected subscriber and publisher types
    _subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoalLinear::odomCallback, this, std::placeholders::_1));
      
    _publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // _x = 0.0; // Initialize _x
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    _x = odom->pose.pose.position.x;
    _y = odom->pose.pose.position.y;

   
    double theta = getYaw(odom->pose.pose.orientation);
    
    RCLCPP_INFO(this->get_logger(), "Pose x: '%f'", _x);

    double goal_x = get_parameter("goal_x").as_double();
    double goal_y = get_parameter("goal_y").as_double();
    double Kp = get_parameter("Kp").as_double();



    geometry_msgs::msg::Twist cmd;

    double ED = sqrt(pow((goal_x - _x),2) + pow((goal_y - _y),2));
    double A = atan2(goal_x - _x,goal_y - _y);
    double EA = A - theta;

    double error_d = Kp * ED; // Corrected the error calculation direction
    double error_a = Kp * EA; // Corrected the error calculation direction


    if (std::abs(error_d) > _threshold) {
        // cmd.linear.x = (error > 0) ? 0.5 : -0.5; // Move towards the set point
        cmd.linear.x = (error_d > 0) ? error_d : error_d; // Move towards the set point
        cmd.angular.z = 0.0;

    } else if(std::abs(error_a) > _threshold){
        cmd.linear.x = 0.0;
        cmd.angular.z = error_a;

    }
    else{
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), " reached goal");

    }
    RCLCPP_INFO(this->get_logger(), "ED: '%f' , EA: '%f' ", ED,EA);

    _publisher->publish(cmd);
  }

  double getYaw(const geometry_msgs::msg::Quaternion &quat){
    tf2::Quaternion q(quat.x,quat.y,quat.z,quat.w);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    return yaw;

  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriber; // Corrected type
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;  // Corrected type
  double _x,_y;
  double _threshold = 0.1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalLinear>());
  rclcpp::shutdown();
  return 0;
}
