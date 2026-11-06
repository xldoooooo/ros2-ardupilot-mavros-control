# Build the native planner coordinate and goal adapter.
cmake_minimum_required(VERSION 3.16)
project(avoidance_bridge)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
foreach(dep rclcpp sensor_msgs nav_msgs geometry_msgs guided_interfaces correction_interfaces path_planning)
  find_package(${dep} REQUIRED)
endforeach()
add_executable(avoidance_bridge_node src/avoidance_bridge.cpp)
target_include_directories(avoidance_bridge_node PRIVATE include)
ament_target_dependencies(avoidance_bridge_node rclcpp sensor_msgs nav_msgs geometry_msgs guided_interfaces correction_interfaces path_planning)
install(TARGETS avoidance_bridge_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/avoidance_backend_manager.py scripts/avoidance_backend_process.py
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(mode identity extnav)
    ament_add_pytest_test(bridge_${mode} test/test_bridge_ros.py TIMEOUT 90
      ENV "ROS_DOMAIN_ID=230" "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
          "ROS_STATIC_PEERS=" "ROS_DISCOVERY_SERVER=" "AVOIDANCE_COORD_MODE=${mode}")
  endforeach()
  ament_add_pytest_test(backend_selection test/test_backend_selection_ros.py TIMEOUT 90
    ENV "ROS_DOMAIN_ID=230" "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
        "ROS_STATIC_PEERS=" "ROS_DISCOVERY_SERVER=")
  ament_add_pytest_test(launch_parameters test/test_launch_parameters.py TIMEOUT 180
    ENV "ROS_DOMAIN_ID=230" "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
        "ROS_STATIC_PEERS=" "ROS_DISCOVERY_SERVER=")
  # All four contracts use the same synthetic endpoints and must run serially.
  set_tests_properties(bridge_identity bridge_extnav backend_selection launch_parameters
    PROPERTIES RUN_SERIAL TRUE)
endif()
ament_package()
