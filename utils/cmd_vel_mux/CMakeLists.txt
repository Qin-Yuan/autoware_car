cmake_minimum_required(VERSION 3.5)
project(cmd_vel_mux)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Nodelet library
add_library(${PROJECT_NAME} SHARED src/cmd_vel_mux.cpp)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  rcl_interfaces
  rclcpp
  rclcpp_components
  std_msgs
)

add_executable(cmd_vel_mux_node src/cmd_vel_mux_node.cpp)
ament_target_dependencies(cmd_vel_mux_node
  rclcpp
)
target_link_libraries(cmd_vel_mux_node cmd_vel_mux)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "cmd_vel_mux::CmdVelMux")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS cmd_vel_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
