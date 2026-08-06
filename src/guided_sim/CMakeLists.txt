# 仿真包只保留模型与可视化；所有持续飞行控制已迁入 onboard_control。
cmake_minimum_required(VERSION 3.16)
project(guided_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# TF 可视化桥不参与飞行控制。
install(
  PROGRAMS
    scripts/pose_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
