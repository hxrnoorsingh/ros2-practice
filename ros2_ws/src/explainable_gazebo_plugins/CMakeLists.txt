cmake_minimum_required(VERSION 3.8)
project(explainable_gazebo_plugins)

# Standard Ignition Gazebo 6 for ROS Humble
find_package(ignition-gazebo6 REQUIRED)
set(GZ_PKG ignition-gazebo6)

find_package(rclcpp REQUIRED)
find_package(explainable_robot REQUIRED)

add_library(state_visualizer_plugin SHARED
  state_visualizer_plugin.cpp
)

target_link_libraries(state_visualizer_plugin
  ${GZ_PKG}::core
  rclcpp::rclcpp
  explainable_robot::explainable_robot__rosidl_typesupport_cpp
)

# Install the plugin library
install(TARGETS state_visualizer_plugin
  LIBRARY DESTINATION lib
)

ament_package()
