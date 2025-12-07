cmake_minimum_required(VERSION 3.5)
project(pupper_apriltag_cpp)

# Default C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ROS2 + OpenCV
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Build the node
add_executable(camera_viewer_cpp src/camera_viewer_cpp.cpp)

ament_target_dependencies(camera_viewer_cpp
  rclcpp
  sensor_msgs
  apriltag_msgs
  cv_bridge
  OpenCV
)

target_link_libraries(camera_viewer_cpp
  ${OpenCV_LIBS}
)

# Install
install(
  TARGETS camera_viewer_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
