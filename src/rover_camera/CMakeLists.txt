cmake_minimum_required(VERSION 3.8)
project(rover_camera)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENNI2 REQUIRED libopenni2)

add_executable(rgb_node src/rgb_node.cpp)
ament_target_dependencies(rgb_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(rgb_node ${OpenCV_LIBS})

add_executable(depth_node src/depth_node.cpp)
ament_target_dependencies(depth_node rclcpp sensor_msgs)
target_include_directories(depth_node PRIVATE ${OPENNI2_INCLUDE_DIRS})
target_link_directories(depth_node PRIVATE ${OPENNI2_LIBRARY_DIRS})
target_link_libraries(depth_node ${OPENNI2_LIBRARIES})

install(TARGETS rgb_node depth_node DESTINATION lib/${PROJECT_NAME})
ament_package()
