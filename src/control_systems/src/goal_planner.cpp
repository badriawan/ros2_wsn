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

class GoalPlanner : public rclcpp::Node
{
public:
  GoalPlanner()
  : Node("goal_planner")
  {
    this->declare_parameter<double>("goal_x", 0.0);
    this->declare_parameter<double>("goal_y", 0.0);
    this->declare_parameter<double>("Kp_dist", 0.5);
    this->declare_parameter<double>("Kp_angle", 0.5);


    // Corrected subscriber and publisher types
    _subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoalPlanner::odomCallback, this, std::placeholders::_1));
      
    _publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // _x = 0.0; // Initialize _x
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    _x = odom->pose.pose.position.x;
    _y = odom->pose.pose.position.y;

   
    double theta = getYaw(odom->pose.pose.orientation);
    
    //RCLCPP_INFO(this->get_logger(), "Pose x: '%f' Pose y: '%f'", _x,_y);

    double goal_x = get_parameter("goal_x").as_double();
    double goal_y = get_parameter("goal_y").as_double();
    double Kp_dist = get_parameter("Kp_dist").as_double();
    // double Ki_dist = get_parameter("Ki_dist").as_double();
    // double Kd_dist = get_parameter("Kd_dist").as_double();


    double Kp_angle = get_parameter("Kp_angle").as_double();




    geometry_msgs::msg::Twist cmd;

    double ED = sqrt(pow((goal_x - _x),2) + pow((goal_y - _y),2));
    double A = atan2(goal_y - _y,goal_x - _x);
    A = atan2(sin(A),cos(A));
    double EA = A - theta;

    //RCLCPP_INFO(this->get_logger(), "Yaw: '%f' , EA: '%f' ", ED,EA);
        
    ED = std::max(0.0,std::min(ED,0.5));
    EA = std::max(-1.0,std::min(EA,1.0));


    double error_d = Kp_dist * ED; // Corrected the error calculation direction
    double error_a = Kp_angle * EA; // Corrected the error calculation direction


    if (std::abs(error_d) > _threshold_d) {
        // cmd.linear.x = (error > 0) ? 0.5 : -0.5; // Move towards the set point
        cmd.linear.x = error_d; // Move towards the set point
        cmd.angular.z = error_a;
        RCLCPP_INFO(this->get_logger(), "velocities '%f' ", cmd.linear.x);

    }
     //else if(std::abs(error_a) > _threshold_a){
    //     cmd.linear.x = 0.0;
    //     cmd.angular.z = error_a;
    //}
    else{
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), " reached goal");

    }
   

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
  double _threshold_d = 0.1;
  double _threshold_a = 0.01;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPlanner>());
  rclcpp::shutdown();
  return 0;
}
