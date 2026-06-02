cmake_minimum_required(VERSION 3.5)
project(guided_sim)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 found at ${Eigen3_DIR}")

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Get the typesupport target once for all executables
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ============================================
# Executables
# ============================================

# keyboard_vel_controller
add_executable(keyboard_vel_controller
  src/keyboard_vel_controller.cpp
  src/ButterWorthFIlter.cpp
  src/PIDControl.cpp
)
ament_target_dependencies(keyboard_vel_controller
  rclcpp
  geometry_msgs
  mavros_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(keyboard_vel_controller
  ${EIGEN3_LIBRARIES}
  ${cpp_typesupport_target}
)

# Install
install(TARGETS
  keyboard_vel_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  params
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

# Install Python scripts
install(
  PROGRAMS
    scripts/pose_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
