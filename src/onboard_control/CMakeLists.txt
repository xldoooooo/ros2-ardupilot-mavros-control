# 无图形机载控制服务及其纯 C++ PD+DOB 控制库构建配置。
cmake_minimum_required(VERSION 3.16)
project(onboard_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(guided_interfaces REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(dob_controller
  src/dob_controller.cpp
  src/reference_generator.cpp
)
target_include_directories(dob_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(dob_controller Eigen3::Eigen)

add_executable(onboard_control_node
  src/main.cpp
  src/onboard_control_node.cpp
)
target_include_directories(onboard_control_node PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(onboard_control_node
  geographic_msgs
  geometry_msgs
  guided_interfaces
  mavros_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(onboard_control_node dob_controller Eigen3::Eigen)

install(TARGETS dob_controller onboard_control_node
  EXPORT export_onboard_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch deploy DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dob_controller test/test_dob_controller.cpp)
  if(TARGET test_dob_controller)
    target_include_directories(test_dob_controller PRIVATE include ${EIGEN3_INCLUDE_DIRS})
    target_link_libraries(test_dob_controller dob_controller Eigen3::Eigen)
  endif()
  ament_add_gtest(test_reference_generator test/test_reference_generator.cpp)
  if(TARGET test_reference_generator)
    target_include_directories(test_reference_generator PRIVATE include ${EIGEN3_INCLUDE_DIRS})
    target_link_libraries(test_reference_generator dob_controller Eigen3::Eigen)
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_onboard_control HAS_LIBRARY_TARGET)
ament_package()
