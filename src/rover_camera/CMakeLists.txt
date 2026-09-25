cmake_minimum_required(VERSION 3.8)
project(rover_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_path(OPENNI2_INCLUDE_DIR
  NAMES OpenNI.h
  HINTS
    /opt/openni2-orbbec/Include
    /usr/include/openni2
    /usr/include/OpenNI2
)

find_library(OPENNI2_LIBRARY
  NAMES OpenNI2
  HINTS
    /opt/openni2-orbbec
    /opt/openni2-orbbec/Redist
    /usr/lib/aarch64-linux-gnu
    /usr/lib
)

if(NOT OPENNI2_INCLUDE_DIR OR NOT OPENNI2_LIBRARY)
  message(FATAL_ERROR "OpenNI2 not found. Expected Orbbec OpenNI2 under /opt/openni2-orbbec or system libopenni2-dev.")
endif()

add_executable(rgb_node src/rgb_node.cpp)
ament_target_dependencies(rgb_node rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(rgb_node ${OpenCV_LIBS})

add_executable(depth_node src/depth_node.cpp)
ament_target_dependencies(depth_node rclcpp sensor_msgs)
target_include_directories(depth_node PRIVATE ${OPENNI2_INCLUDE_DIR})
target_link_libraries(depth_node ${OPENNI2_LIBRARY})

get_filename_component(OPENNI2_LIBRARY_DIR "${OPENNI2_LIBRARY}" DIRECTORY)
set_target_properties(depth_node PROPERTIES
  BUILD_RPATH "${OPENNI2_LIBRARY_DIR}"
  INSTALL_RPATH "${OPENNI2_LIBRARY_DIR}"
)

install(TARGETS rgb_node depth_node DESTINATION lib/${PROJECT_NAME})
ament_package()
