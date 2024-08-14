#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>

using std::placeholders::_1;

enum class RobotState {
  MOVE_STRAIGHT,
  TURN_LEFT,
  TURN_RIGHT,
  // OUT_MAZE
};

class MazeLidar : public rclcpp::Node {
public:
  MazeLidar() : Node("maze_lidar") {
    _publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&MazeLidar::lidarCallback, this, _1));
  }

private:
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    obstacleSens(msg);
    robotState();
    robotAction();
  }

  void obstacleSens(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    
    _front_sens = *std::min_element(msg->ranges.begin() + 340, msg->ranges.begin() + 360);
    _left_sens = *std::min_element(msg->ranges.begin() + 80, msg->ranges.begin() + 100);
    _right_sens = *std::min_element(msg->ranges.begin() + 260, msg->ranges.begin() + 280);

    RCLCPP_INFO(this->get_logger(), "front: '%f' | left: '%f' | right: '%f' ",
    _front_sens,
    _left_sens,
    _right_sens);

  }

  void robotState(){
    switch (_action) {
      case RobotState::MOVE_STRAIGHT:
        if (_front_sens < _dist) {
          if (_left_sens < _dist2) {
            _action = RobotState::TURN_RIGHT;
          // } else if (_right_sens < _dist2) {
          //   _action = RobotState::TURN_LEFT;
          // } else {
          //   _action = RobotState::OUT_MAZE;
          }
          else {
            _action = RobotState::TURN_LEFT;
          }
        }
        break;

      case RobotState::TURN_RIGHT:
      case RobotState::TURN_LEFT:
        _action = RobotState::MOVE_STRAIGHT;
        break;

      // case RobotState::OUT_MAZE:
      //   if (_front_sens > 5 && _left_sens > 5 && _right_sens > 5) {
      //     _action = RobotState::OUT_MAZE;
      //   } else {
      //     _action = RobotState::MOVE_STRAIGHT;
      //   }
      //   break;
    }
  }
    
  void robotAction(){
    auto cmd = geometry_msgs::msg::Twist();

    switch (_action) {
      case RobotState::MOVE_STRAIGHT:
        cmd.linear.x = 0.2;
        cmd.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Moving forward");
        break;
      case RobotState::TURN_LEFT:
        cmd.linear.x = 0.0;
        cmd.angular.z = 1.5;
        RCLCPP_INFO(this->get_logger(), "Turning left");
        break;
      case RobotState::TURN_RIGHT:
        cmd.linear.x = 0.0;
        cmd.angular.z = -1.5;
        RCLCPP_INFO(this->get_logger(), "Turning right");
        break;
      // case RobotState::OUT_MAZE:
      //   cmd.linear.x = 0.0;
      //   cmd.angular.z = 0.0;
      //   RCLCPP_INFO(this->get_logger(), "Stopping");
      //   break;
    }

    _publisher->publish(cmd);




  }
    

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _subscription;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
  RobotState _action;
  double _dist = 1.5;
  double _dist2 = 1.5;
  float _front_sens;
  float _left_sens;
  float _right_sens;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MazeLidar>());
  rclcpp::shutdown();
  return 0;
}
