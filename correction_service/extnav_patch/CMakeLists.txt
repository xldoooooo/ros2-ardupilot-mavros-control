# Build the native bridge independently before installing the unchanged ROS entry point.
cmake_minimum_required(VERSION 3.16)
project(extnav_bridge VERSION 2.0.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
option(EXTNAV_WITH_CORRECTION "Build correction API when its interfaces are available" ON)
if(EXTNAV_WITH_CORRECTION)
  find_package(correction_interfaces QUIET)
endif()

# Build a small no-API executable too: missing optional shared libraries are detected before exec.
function(add_extnav_target target api)
  add_executable(${target} src/extnav_to_vision_pose.cpp)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_include_directories(${target} PRIVATE include)
  # Keep IEEE finite checks and predictable double arithmetic; never enable fast-math.
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
  target_compile_definitions(${target} PRIVATE EXTNAV_HAS_CORRECTION_API=${api})
  ament_target_dependencies(${target} rclcpp geometry_msgs nav_msgs)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

if(EXTNAV_WITH_CORRECTION AND correction_interfaces_FOUND)
  add_extnav_target(extnav_to_vision_pose_native 1)
  ament_target_dependencies(extnav_to_vision_pose_native correction_interfaces)
else()
  message(WARNING "correction_interfaces unavailable: building the legacy identity fallback")
endif()
add_extnav_target(extnav_to_vision_pose_identity 0)

install(PROGRAMS scripts/extnav_to_vision_pose DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
