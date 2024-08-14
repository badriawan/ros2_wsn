cmake_minimum_required(VERSION 3.5)
project(robot_sensing)

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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})






# add_executable(talker src/publisher.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)

# add_executable(listener src/subscriber.cpp)
# ament_target_dependencies(listener rclcpp std_msgs)

# add_executable(drive_turtle src/drive_turtle.cpp)
# ament_target_dependencies(drive_turtle rclcpp std_msgs geometry_msgs)

# add_executable(tf_static src/tf_static.cpp)
# ament_target_dependencies(tf_static rclcpp std_msgs geometry_msgs tf2_ros)

add_executable(odom_kinematic src/odom_kinematic.cpp)
ament_target_dependencies(odom_kinematic rclcpp std_msgs geometry_msgs nav_msgs)

add_executable(maze_lidar src/maze_solving_with_lidar.cpp)
ament_target_dependencies(maze_lidar rclcpp sensor_msgs)

add_executable(line_following src/linefollowing.cpp)
ament_target_dependencies(line_following rclcpp sensor_msgs geometry_msgs cv_bridge)
target_link_libraries(line_following ${OpenCV_LIBS})

add_executable(point_cloud_segmentation src/point_cloud_segmentation.cpp)
ament_target_dependencies(point_cloud_segmentation rclcpp sensor_msgs PCL pcl_conversions )
target_link_libraries(point_cloud_segmentation ${PCL_LIBRARIES})


install(TARGETS
 maze_lidar
 odom_kinematic
 line_following
 point_cloud_segmentation
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  world
  DESTINATION share/${PROJECT_NAME})

ament_package()