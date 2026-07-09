cmake_minimum_required(VERSION 3.8)
project(patrolbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install directories so ROS 2 can find them
install(DIRECTORY
  config
  launch
  maps
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executable ROS 2 nodes
install(PROGRAMS
  scripts/patrolbot_joy_teleop.py
  scripts/patrolbot_safety_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()