# 机载控制服务与地面站共享的 ROS 2 接口构建配置。
cmake_minimum_required(VERSION 3.16)
project(guided_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandResult.msg"
  "msg/ControlHeartbeat.msg"
  "msg/ControlStatus.msg"
  "msg/MotionIntent.msg"
  "msg/Waypoint.msg"
  "srv/AcquireControl.srv"
  "srv/ExecuteWaypoints.srv"
  "srv/FlightCommand.srv"
  "srv/SetGpsOrigin.srv"
  DEPENDENCIES
    builtin_interfaces
    geographic_msgs
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
