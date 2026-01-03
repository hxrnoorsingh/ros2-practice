cmake_minimum_required(VERSION 3.8)
project(explainable_gazebo_plugins)

# Find Gazebo
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(explainable_robot REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(state_visualizer_plugin SHARED
  state_visualizer_plugin.cpp
)

target_link_libraries(state_visualizer_plugin
  ${GAZEBO_LIBRARIES}
  rclcpp::rclcpp
  explainable_robot::explainable_robot__rosidl_typesupport_cpp
)

# Install the plugin library
install(TARGETS state_visualizer_plugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

ament_package()
