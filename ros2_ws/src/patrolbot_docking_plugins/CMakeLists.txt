cmake_minimum_required(VERSION 3.8)
project(patrolbot_docking_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(patrolbot_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(patrolbot_charging_dock SHARED
  src/patrolbot_charging_dock.cpp)
target_compile_features(patrolbot_charging_dock PUBLIC cxx_std_17)
target_include_directories(patrolbot_charging_dock PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(patrolbot_charging_dock
  geometry_msgs
  opennav_docking_core
  patrolbot_interfaces
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_geometry_msgs)

pluginlib_export_plugin_description_file(
  opennav_docking_core patrolbot_docking_plugins.xml)

install(TARGETS patrolbot_charging_dock
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES patrolbot_docking_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_charge_state test/test_charge_state.cpp)
  target_include_directories(test_charge_state PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(patrolbot_charging_dock)
ament_export_dependencies(opennav_docking_core patrolbot_interfaces)
ament_package()
