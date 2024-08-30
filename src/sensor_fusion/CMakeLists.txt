cmake_minimum_required(VERSION 3.5)
project(sensor_fusion)

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
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)



# find_package(PCL REQUIRED)
# find_package(pcl_conversions REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

add_executable(linear_kf src/linear_kf.cpp)
ament_target_dependencies(linear_kf rclcpp visualization_msgs nav_msgs)

add_executable(advanced_kf src/advanced_kf.cpp)
ament_target_dependencies(advanced_kf rclcpp visualization_msgs nav_msgs)

add_executable(noise_kf src/noise_kf.cpp)
ament_target_dependencies(noise_kf rclcpp visualization_msgs nav_msgs)
target_include_directories(noise_kf PRIVATE ${EIGEN_INCLUDE_DIR} include)
target_link_libraries(noise_kf Eigen3::Eigen)


install(TARGETS
 linear_kf
 advanced_kf
 noise_kf
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()