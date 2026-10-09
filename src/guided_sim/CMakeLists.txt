# 仿真包提供模型、可视化与合成环境观测；持续飞行控制只由 onboard_control 执行。
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

# TF、预览桥及合成扫描均不参与飞行控制。
install(
  PROGRAMS
    scripts/pose_to_tf.py
    scripts/avoidance_preview.py
    scripts/simulation_obstacles.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
