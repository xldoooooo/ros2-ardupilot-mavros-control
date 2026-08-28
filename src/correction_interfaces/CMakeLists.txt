# AprilTag-Odin 修正服务、extnav 与地面调试面板共享的独立 ROS 2 接口。
cmake_minimum_required(VERSION 3.16)
project(correction_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CorrectionResult.msg"
  "msg/CorrectionStatus.msg"
  "msg/ExtnavCorrectionStatus.msg"
  "srv/SetCorrection.srv"
  "srv/StartCorrection.srv"
  "srv/StopCorrection.srv"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
