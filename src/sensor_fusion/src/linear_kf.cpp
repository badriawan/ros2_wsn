#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"



using namespace std::chrono_literals;

class SimpleKalmanFilter{
public:
    SimpleKalmanFilter(double process_noise,double measurement_noise,double estimation_error,double initial_value){

    R = measurement_noise;
    Q = process_noise;
    P = estimation_error;
    X = initial_value;
}
    void update(double measurement){

    P = P + Q;
    K = P / (P + R);
    X = X + K + (measurement - X);
    P = (1 - K) * P;
}
    double getState(){ 
    return X; 
}

private:
    double R;
    double Q;
    double P;
    double X;
    double K;
};



class Tb3OdomNode : public rclcpp::Node
{
public:
  Tb3OdomNode()
  : Node("tb3_odom_node"),kf(0.1,0.1,1.0,0.1)
  {
    _subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Tb3OdomNode::sub_callback, this, std::placeholders::_1));

    _publisher = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // _timer = this->create_wall_timer(
    //   100ms, std::bind(&Tb3OdomNode::_timercallback, this));
  }

private:
  void sub_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
   
    double measured_x = odom->pose.pose.position.x;

    kf.update(measured_x);
    double filtered_x = kf.getState();



    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "tb3odom";
    marker.id = 0.0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = filtered_x;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;


    _publisher->publish(marker);
  }


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriber;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _publisher;
  SimpleKalmanFilter kf;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tb3OdomNode>());
  rclcpp::shutdown();
  return 0;
}
