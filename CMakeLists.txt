cmake_minimum_required(VERSION 3.8)
project(voxelnav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Voxelizer library
add_library(voxelnav_voxelizer SHARED
  src/voxelizer.cpp
)
ament_target_dependencies(voxelnav_voxelizer
  rclcpp sensor_msgs Eigen3
)

# Segmenter library (ONNX inference)
add_library(voxelnav_segmenter SHARED
  src/segmenter.cpp
)
ament_target_dependencies(voxelnav_segmenter
  rclcpp sensor_msgs OpenCV
)

# Main ROS2 node
add_executable(voxelnav_node
  src/voxelnav_node.cpp
)
target_link_libraries(voxelnav_node
  voxelnav_voxelizer
  voxelnav_segmenter
)
ament_target_dependencies(voxelnav_node
  rclcpp rclcpp_components sensor_msgs geometry_msgs
)

# Nav2 costmap plugin
add_library(voxelnav_costmap_plugin SHARED
  src/costmap_plugin.cpp
)
ament_target_dependencies(voxelnav_costmap_plugin
  rclcpp nav2_costmap_2d pluginlib
)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  add_executable(test_voxelizer test/test_voxelizer.cpp)
  target_link_libraries(test_voxelizer voxelnav_voxelizer)
endif()

install(TARGETS
  voxelnav_node
  voxelnav_voxelizer
  voxelnav_segmenter
  voxelnav_costmap_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()