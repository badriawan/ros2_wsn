#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"

class PointCloudSeg : public rclcpp::Node {
public:
  PointCloudSeg() : Node("point_cloud_segmentation")//,_count(0)
   {
    _publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_seg", 10);

    _subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/kitti/point_cloud", 10, std::bind(&PointCloudSeg::pclCallback, this, std::placeholders::_1));
    // _timer = this->create_wall_timer(
    //   std::chrono::seconds(1), std::bind(&PointCloudSeg::pclCallback, this));
  }
//std::chrono::seconds(1)
private:
  // void pclCallback() {
  //   pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
  //   double radius = 3.0;
  //   int num_points = 50;
  //   double angular_step_size = 2*M_PI/num_points;

  //   for(int i;i<num_points;i++){
  //       int angle = angular_step_size*i;
  //       pcl::PointXYZRGB point;
  //       point.x = radius * std::cos(angle);
  //       point.y = radius * std::sin(angle);
  //       point.z = 1.0;


  //       point.r = 255 * std::cos(angle);
  //       point.g = 255 * std::sin(angle);
  //       point.b = 255 * std::cos(angle + M_PI_2);

  //       cloud_rgb.push_back(point);}

  //       sensor_msgs::msg::PointCloud2 cloud_msg;
  //       pcl::toROSMsg(cloud_rgb,cloud_msg);
  //       cloud_msg.header.stamp =  this->get_clock()->now();
  //       cloud_msg.header.frame_id = "car";
        
  //       _publisher->publish(cloud_msg);

  //   };
  
  void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr p){

   //get ros data
   //define pcl variable
    pcl::PointCloud<pcl::PointXYZ>::Ptr point (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_point (new pcl::PointCloud<pcl::PointXYZ>);

    //covertbto pcl variable
    pcl::fromROSMsg(*p,*point);

    //Apply the filter

    pcl::PassThrough<pcl::PointXYZ> filter_x;
    pcl::PassThrough<pcl::PointXYZ> filter_y;

    radius_filter = 10;
    filter_x.setInputCloud(point);
    filter_x.setFilterFieldName("x");
    filter_x.setFilterLimits(-radius_filter,radius_filter);
    filter_x.filter(*crop_point);




    pcl::toROSMsg(*crop_point,cloud_msg);
    cloud_msg.header.stamp =  this->get_clock()->now();
    cloud_msg.header.frame_id = "car";
        
    _publisher->publish(cloud_msg);



  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  int radius_filter;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscription;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;
  //rclcpp::TimerBase::SharedPtr _timer;
  //size_t _count;



};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSeg>());
  rclcpp::shutdown();
  return 0;
}









// int main() {

//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     cloud.push_back(pcl::PointXYZ(1.0,0.0,0.0)); 
//     cloud.push_back(pcl::PointXYZ(2.0,0.0,0.0)); 
//     cloud.push_back(pcl::PointXYZ(3.0,0.0,0.0)); 
//     cloud.push_back(pcl::PointXYZ(4.0,0.0,0.0)); 

//     //pcl::io::savePCDFileASCII("/home/yusuf/ros2_ws/src/robot_sensing/simple.pcd", cloud);


//     pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
//     double radius = 3.0;
//     int num_points = 50;
//     double angular_step_size = 2*M_PI/num_points;

//     for(int i;i<num_points;i++){
//         int angle = angular_step_size*i;
//         pcl::PointXYZRGB point;
//         point.x = radius * std::cos(angle);
//         point.y = radius * std::sin(angle);
//         point.z = 1.0;


//         point.r = 255 * std::cos(angle);
//         point.g = 255 * std::sin(angle);
//         point.b = 255 * std::cos(angle + M_PI_2);

//         cloud_rgb.push_back(point);

//     }

//     //pcl::io::savePCDFileASCII("/home/yusuf/ros2_ws/src/robot_sensing/circle.pcd", cloud_rgb);




//     return 0;
// }

