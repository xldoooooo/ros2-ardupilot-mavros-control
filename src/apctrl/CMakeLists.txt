cmake_minimum_required(VERSION 3.8)
project(apctrl)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# 包含目录
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(apctrl_node 
  src/apctrl_node.cpp
  src/APCtrlParam.cpp
  src/test.cpp
  src/controller.cpp
  src/input.cpp
  src/APCtrlFSM.cpp
)

# 链接依赖库
ament_target_dependencies(apctrl_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  quadrotor_msgs
  Eigen3
)

# 安装可执行文件
install(TARGETS
  apctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_package() 


