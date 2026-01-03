cmake_minimum_required(VERSION 3.8)
project(explainable_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorStatus.msg"
  "msg/RobotState.msg"
  "msg/Uncertainty.msg"
  DEPENDENCIES std_msgs
)

# Install Python modules
install(DIRECTORY explainable_robot
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  explainable_robot/sensor_health_monitor.py
  explainable_robot/task_state_manager.py
  explainable_robot/uncertainty_estimator.py
  explainable_robot/explainability_publisher.py
  explainable_robot/fault_injector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and urdf files
install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
