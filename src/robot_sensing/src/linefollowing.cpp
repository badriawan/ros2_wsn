#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


using namespace std::chrono_literals;

class LineFollowing : public rclcpp::Node
{
public:
  LineFollowing()
  : Node("line_follower"),dt(0.065)
  {
    this->declare_parameter<int>("lower_threshold",200);
    this->declare_parameter<int>("upper_threshold",250);
    this->declare_parameter<double>("Kp", 0.001);
    this->declare_parameter<double>("Ki", 0.001);

    _subscriber = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&LineFollowing::subCallback, this, std::placeholders::_1));

    _publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "test");


    // _timer = this->create_wall_timer(
    //   100ms, std::bind(&LineFollowing::timerCallback, this));
  }

private:
  void subCallback(const sensor_msgs::msg::Image::SharedPtr camera_msg)
  {
    //get ros data
    //ros to opencv data
    // auto now = this->get_clock()->now();
    // RCLCPP_INFO(this->get_logger(), "Current time: %f", now.seconds());
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(camera_msg,"bgr8");

    cv::Mat gray_image,cannyImage;
    cv::cvtColor(cv_ptr->image , gray_image,cv::COLOR_BGR2GRAY);
    int lower_threshold = this->get_parameter("lower_threshold").as_int();
    int upper_threshold = this->get_parameter("upper_threshold").as_int();
    double Kp = get_parameter("Kp").as_double();
    double Ki = get_parameter("Ki").as_double();
    //edge segmentation
    cv::Canny(gray_image,cannyImage,lower_threshold,upper_threshold);

    //col = 1920 row = 1080
    int row = 650, column = 0;
    //boundaries dari pixel yang mau di proses
    cv::Mat roi = cannyImage(cv::Range(row, row + 430), cv::Range(column, column + 1920)); //old version = 640x480

    //save the 255 pixels on row 160 of the new pixel and column i on vector variables edge
    std::vector<int> edge;
    for (int i = 0; i < 1920; ++i) {
      if (roi.at<uchar>(160, i) == 255) {
        edge.push_back(i);
      }
    }
    // check the condition of edge, if edge contains the member list, it will be executed
    if (!edge.empty()){
    int line = edge[1]-edge[0];
    int mid_line = edge[0] + line/2;
    int mid_robot = 1920/2;

    // if (!edge.empty()) {
    // int midArea = edge.back() - edge.front();
    // int midPoint = edge.front() + midArea / 2;
    // int robotMidPoint = 640 / 2;

    cv::circle(roi,cv::Point(mid_line,160),2,cv::Scalar(255,255,255),-1);
    cv::circle(roi,cv::Point(mid_robot,160),5,cv::Scalar(255,255,255),-1);
    // Calculate error and adjust robot's direction
    auto velocityMsg = geometry_msgs::msg::Twist();
    double error = mid_robot - mid_line;
    velocityMsg.linear.x = 0.2;
    // if (error < 0) {
    //   velocityMsg.angular.z = -_angularVel;
    // } else {
    //   velocityMsg.angular.z = _angularVel;
    // }
    
    integral += error * dt;
    velocityMsg.angular.z = Kp*error + Ki*integral;//_angularVel;
    //RCLCPP_INFO(this->get_logger(), "Angular-w: %f", velocityMsg.angular.z);

    _publisher->publish(velocityMsg);


    cv::imshow("Image", roi);
    cv::waitKey(1);
  }
  }


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
  double _angularVel = 0.2;
  double dt;
  double integral;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineFollowing>());
  rclcpp::shutdown();
  return 0;
}
