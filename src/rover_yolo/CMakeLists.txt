cmake_minimum_required(VERSION 3.18)
project(rover_yolo)

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
find_package(CUDAToolkit REQUIRED)

find_path(NVINFER_INCLUDE_DIR NvInfer.h PATHS /usr/include/aarch64-linux-gnu /usr/include)
find_library(NVINFER_LIBRARY nvinfer PATHS /usr/lib/aarch64-linux-gnu /usr/lib)
if(NOT NVINFER_INCLUDE_DIR OR NOT NVINFER_LIBRARY)
  message(FATAL_ERROR "TensorRT NvInfer.h/libnvinfer not found")
endif()

add_executable(yolo_node src/yolo_node.cpp)
ament_target_dependencies(yolo_node rclcpp sensor_msgs std_msgs cv_bridge)
target_include_directories(yolo_node PRIVATE ${NVINFER_INCLUDE_DIR})
target_link_libraries(yolo_node ${OpenCV_LIBS} ${NVINFER_LIBRARY} CUDA::cudart)

install(TARGETS yolo_node DESTINATION lib/${PROJECT_NAME})
ament_package()
