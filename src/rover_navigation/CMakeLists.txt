cmake_minimum_required(VERSION 3.8)
project(rover_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance rclcpp sensor_msgs geometry_msgs)

install(TARGETS obstacle_avoidance DESTINATION lib/${PROJECT_NAME})

ament_package()
