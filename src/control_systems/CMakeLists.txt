cmake_minimum_required(VERSION 3.5)
project(control_systems)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)




# add_executable(talker src/publisher.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)

# add_executable(listener src/subscriber.cpp)
# ament_target_dependencies(listener rclcpp std_msgs)

# add_executable(drive_turtle src/drive_turtle.cpp)
# ament_target_dependencies(drive_turtle rclcpp std_msgs geometry_msgs)

add_executable(goal_linear src/goal_linear.cpp)
ament_target_dependencies(goal_linear rclcpp std_msgs geometry_msgs nav_msgs tf2_ros)

add_executable(goal_planner src/goal_planner.cpp)
ament_target_dependencies(goal_planner rclcpp std_msgs geometry_msgs nav_msgs tf2_ros)



install(TARGETS
  goal_linear
  goal_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()