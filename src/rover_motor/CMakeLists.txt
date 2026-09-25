cmake_minimum_required(VERSION 3.8)
project(rover_motor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(motor_node src/motor_node.cpp)
ament_target_dependencies(motor_node rclcpp rcl_interfaces geometry_msgs)

add_executable(rover_control src/rover_control.cpp)

install(TARGETS motor_node rover_control DESTINATION lib/${PROJECT_NAME})

ament_package()
