cmake_minimum_required(VERSION 3.14)
project(autoware_pointcloud_preprocessor)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Suppress CGAL warning
set(CGAL_DATA_DIR ".")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)

include_directories(
  include
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(pointcloud_preprocessor_filter_base SHARED
  src/filter.cpp
)

target_include_directories(pointcloud_preprocessor_filter_base PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(pointcloud_preprocessor_filter_base
  message_filters
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2_ros
  # tier4_autoware_utils
  pcl_ros
)

ament_auto_add_library(pointcloud_preprocessor_filter SHARED
  # src/autoware_downsample_filter/voxel_grid_downsample_filter_nodelet.cpp
  src/autoware_downsample_filter/random_downsample_filter_nodelet.cpp
  # src/autoware_downsample_filter/approximate_downsample_filter_nodelet.cpp
)

target_link_libraries(pointcloud_preprocessor_filter
  pointcloud_preprocessor_filter_base
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ========== Down Sampler Filter ==========
# -- Voxel Grid Downsample Filter --
# rclcpp_components_register_node(pointcloud_preprocessor_filter
#   PLUGIN "autoware_pointcloud_preprocessor::VoxelGridDownsampleFilterComponent"
#   EXECUTABLE voxel_grid_downsample_filter_node)

# -- Random Downsample Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware_pointcloud_preprocessor::RandomDownsampleFilterComponent"
  EXECUTABLE random_downsample_filter_node)

# rclcpp_components_register_node(pointcloud_preprocessor_filter
#   PLUGIN "autoware_pointcloud_preprocessor::ApproximateDownsampleFilterComponent"
#   EXECUTABLE approximate_downsample_filter_node)


install(
  TARGETS pointcloud_preprocessor_filter_base EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE
  launch
)
