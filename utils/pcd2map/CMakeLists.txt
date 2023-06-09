cmake_minimum_required(VERSION 3.5)
project(pcd2pgm)

cmake_policy(SET CMP0074 NEW)

# Default to C99
set(CMAKE_C_STANDARD 99)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
)

set(dependencies
  nav_msgs
  nav2_msgs
  pcl_ros
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  PCL
)

add_executable(pcd2map src/pcd2pgm.cpp)
ament_target_dependencies(pcd2map
  ${dependencies}
)
install(TARGETS pcd2map DESTINATION lib/${PROJECT_NAME})

# 编译依赖文件
install(DIRECTORY map DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
