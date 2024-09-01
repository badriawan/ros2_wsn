#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>
#include <random>



using namespace std::chrono_literals;

double generateGaussianNoise(double mean, double stddev) {
    static std::random_device rd;
    static std::mt19937 generator(rd());
    std::normal_distribution<double> distribution(mean, stddev);
    return distribution(generator);
}


class KalmanFilter{
public:
    KalmanFilter(double process_noise,double measurement_noise,double estimation_error,double x_value,double y_value){ //covariance

    R = measurement_noise;
    Q = process_noise;
    P.setIdentity();
    P = estimation_error * Eigen::Matrix2d::Identity();
    X << x_value,y_value; //prediction error ; The goals from the measurement and prediction is to make the less value of P or around 0.
    F.setIdentity();
    H.setIdentity();


}
    void prediction(double controller_x,double controller_y,double dt){ //The controller are velocity.

    double noisy_vx = controller_x + generateGaussianNoise(0,Q);
    double noisy_vy = controller_y + generateGaussianNoise(0,Q);


    //state prediction
    X(0) = X(0) + noisy_vx * dt;
    X(1) = X(1) + noisy_vy * dt;
    
    //cov prediction
    P = F * P * F.transpose() + Q * Eigen::Matrix2d::Identity();

    }
    void update(double measurement_x,double measurement_y){

    double noisy_x = measurement_x + generateGaussianNoise(0,R);
    double noisy_y = measurement_y + generateGaussianNoise(0,R);

    
    Eigen::Vector2d Z;
    Z << noisy_x,noisy_y;

    Eigen::Vector2d Y = Z - H * X; //measurement residual
    Eigen::Matrix2d S = H*P*H.transpose() + R*Eigen::Matrix2d::Identity(); //residual covariance
    Eigen::Matrix2d K = P * H.transpose() * S.inverse();  //kalman gain

    X = X + K * Y;

    P = (Eigen::Matrix2d::Identity() - K*H) * P;
}
    double getStateX(){ 
    return X(0);
}
    double getStateY(){ 
    return X(1);
}

private:
    double R; //Process Noise Covariance
    double Q; //Measurement Noise Covariance
    Eigen::Matrix2d P; //estimation error covariance
    Eigen::Matrix2d F; //transition matrix
    Eigen::Matrix2d H; //measurement matrix
    Eigen::Vector2d X; //state X,Y

};



class Tb3OdomNode : public rclcpp::Node
{
public:
  Tb3OdomNode()
  : Node("tb3_odom_node"),kf(0.1,0.1,1.0,0.0,0.0),controller_x(0.0),controller_y(0.0)
  {
    _odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Tb3OdomNode::odomCallback, this, std::placeholders::_1));
    
    _cmd_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&Tb3OdomNode::cmdCallback, this, std::placeholders::_1));

    _publisher = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    _timer = this->create_wall_timer(
      100ms, std::bind(&Tb3OdomNode::timerCallback, this));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
   
    double measured_x = odom->pose.pose.position.x;
    double measured_y = odom->pose.pose.position.y;


    kf.update(measured_x,measured_y);
   
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd){

    controller_x = cmd->linear.x;
    controller_y = cmd->linear.y;


  }

  void timerCallback(){

    double dt = 0.1;

    kf.prediction(controller_x,controller_y,dt);
    double filtered_x = kf.getStateX();
    double filtered_y = kf.getStateY();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "tb3odom";
    marker.id = 0.0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = filtered_x;
    marker.pose.position.y = filtered_y;
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


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_subscriber;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _timer;
  KalmanFilter kf;
  double controller_x;
  double controller_y;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tb3OdomNode>());
  rclcpp::shutdown();
  return 0;
}
