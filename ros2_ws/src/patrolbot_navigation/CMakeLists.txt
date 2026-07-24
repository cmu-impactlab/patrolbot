cmake_minimum_required(VERSION 3.8)
project(patrolbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_safety_contract test/test_safety_contract.py)
  ament_add_pytest_test(test_safety_policy test/test_safety_policy.py)
  ament_add_pytest_test(
    test_physical_calibration
    test/test_physical_calibration.py)
  ament_add_pytest_test(
    test_collision_monitor_integration
    test/test_collision_monitor_integration.py
    TIMEOUT 90)
endif()

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
  scripts/validate_physical_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/joystick_policy.py
  scripts/safety_policy.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
