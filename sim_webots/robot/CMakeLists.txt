cmake_minimum_required(VERSION 3.5)
project(robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 设置源文件根目录，并设置别名为SRC_DIR
set(SRC_DIR ./)
set(WEBOTS_HOME /usr/local/webots)
# set(WEBOTS_HOME ../../utils/webots_ros2/webots_ros2_driver/webots)
set(BOOST_HOME /usr/include/boost)
set(EIGEN3_HOME /usr/include/eigen3)

# 宏定义及头文件路径设置
add_definitions(-D_USE_MATH_DEFINES)

add_definitions(-DSIMU_ROBOT)
# 声明三方库位置路径
link_directories(${WEBOTS_HOME}/lib/controller)
set(LIBRARIES Controller)
include_directories(
        ${SRC_DIR}/SimuWebot/
        ${WEBOTS_HOME}/include/controller/c/
        ${WEBOTS_HOME}/include/controller/cpp/)
# 添加webots源码编译：extern控制
aux_source_directory(${WEBOTS_HOME}/src/controller/cpp WEBOTS_SRC)
set(RBT_SRC ${SRC_DIR}/SimuWebot/Droid.cpp)

include_directories(
        ${WEBOTS_HOME}/
        ${BOOST_HOME}/
        ${EIGEN3_HOME}/
        ${SRC_DIR}/Locomotion/
        ${SRC_DIR}/ComSource/
        ${SRC_DIR}/XRos2/
        ${SRC_DIR}/Task/)

set(COM_SRC
        ${SRC_DIR}/Locomotion/CWheels.cpp
        ${SRC_DIR}/Locomotion/DiffeDriver.cpp
        ${SRC_DIR}/XRos2/Ros2Info.cpp
        ${SRC_DIR}/Task/CState.cpp
        ${SRC_DIR}/Task/main.cpp)

# 项目包含哪些 文件
add_executable(${PROJECT_NAME} ${WEBOTS_SRC} ${COM_SRC} ${RBT_SRC})

# 链接库

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        geometry_msgs
        std_msgs
        std_srvs
        sensor_msgs
        nav_msgs
        visualization_msgs
)
# 链接webots库
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
# 库文件拷贝 TODO: Ubuntu下面搜索系统目录，暂时用不着
# if (${CMAKE_SYSTEM_NAME} STREQUAL "Linux") #XXX平台可以是“Linux”等
#     message(STATUS "拷贝webots库文件: ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}")
#     add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
#         COMMAND ${CMAKE_COMMAND} -E copy $ENV{WEBOTS_HOME}/lib/controller/${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}  ${PROJECT_BINARY_DIR}/
#         COMMAND ${CMAKE_COMMAND} -E copy $ENV{WEBOTS_HOME}/lib/controller/${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX} ${PROJECT_BINARY_DIR}/
#         )
# elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
#     message(STATUS "拷贝webots库文件: Controller${CMAKE_SHARED_LIBRARY_SUFFIX}")
#     add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
#         COMMAND ${CMAKE_COMMAND} -E copy $ENV{WEBOTS_HOME}/lib/controller/Controller${CMAKE_SHARED_LIBRARY_SUFFIX}  ${PROJECT_BINARY_DIR}/
#         COMMAND ${CMAKE_COMMAND} -E copy $ENV{WEBOTS_HOME}/lib/controller/CppController${CMAKE_SHARED_LIBRARY_SUFFIX} ${PROJECT_BINARY_DIR}/
#         )
# endif()

# TODO: 需要将libController.so拷贝到/usr/lib/ 或者设置LD_LIBRARY_PATH
# sudo cp ${WEBOTS_HOME}/lib/controller/libController.so /usr/lib/
# 将指定路径的库文件安装到指定目录 TODO:ROS2下搜索当前目录
# install(DIRECTORY lib/ DESTINATION lib/${PROJECT_NAME})
install(TARGETS  ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(FILES  
        lib/libController.so 
        DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY 
#         ${PROJECT_SOURCE_DIR}/../launch 
#         DESTINATION share/${PROJECT_NAME})
ament_package()

# colcon build --packages-select robot
