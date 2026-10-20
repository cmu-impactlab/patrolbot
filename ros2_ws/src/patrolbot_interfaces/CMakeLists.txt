cmake_minimum_required(VERSION 3.8)
project(patrolbot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BaseState.msg"
  "msg/NavigationPath.msg"
  "msg/DockReadiness.msg"
  "msg/GimbalCommand.msg"
  "msg/GimbalState.msg"
  "srv/GetNavigationPath.srv"
  "srv/ChargeRelease.srv"
  "srv/DockReset.srv"
  "srv/InitializeDockPose.srv"
  "srv/MotorEnable.srv"
  "srv/MotorDisable.srv"
  "srv/SetLidarPower.srv"
  "srv/RobotTuning.srv"
  "action/HardwareUndock.action"
  "action/UndockTurn.action"
  "action/Undock.action"
  "action/LocalizeOnMap.action"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs nav_msgs std_msgs unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
