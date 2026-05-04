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
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(ONNXRUNTIME_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime/pkg/onnxruntime-linux-x64-1.25.1)
set(ONNXRUNTIME_LIB_DIR ${ONNXRUNTIME_ROOT}/lib)
include_directories(${ONNXRUNTIME_ROOT}/include)
link_directories(${ONNXRUNTIME_LIB_DIR})

set(CMAKE_BUILD_RPATH ${ONNXRUNTIME_LIB_DIR})
set(CMAKE_INSTALL_RPATH "$ORIGIN/..")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Voxelizer library
add_library(voxelnav_voxelizer SHARED
  src/voxelizer.cpp
)
ament_target_dependencies(voxelnav_voxelizer
  rclcpp sensor_msgs Eigen3
)

target_link_libraries(voxelnav_voxelizer
  ${PCL_LIBRARIES}
)

# Segmenter library (ONNX inference)
add_library(voxelnav_segmenter SHARED
  src/segmenter.cpp
)
target_link_libraries(voxelnav_segmenter
  onnxruntime
  ${OpenCV_LIBRARIES}
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
  ${PCL_LIBRARIES}
  onnxruntime
)
ament_target_dependencies(voxelnav_node
  rclcpp rclcpp_components sensor_msgs geometry_msgs message_filters cv_bridge pcl_conversions
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
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_voxelizer test/test_voxelizer.cpp)
  if(TARGET test_voxelizer)
    target_link_libraries(test_voxelizer voxelnav_voxelizer)
    ament_target_dependencies(test_voxelizer Eigen3)
  endif()
endif()

install(TARGETS
  voxelnav_node
  voxelnav_voxelizer
  voxelnav_segmenter
  voxelnav_costmap_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.1
  ${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.1.25.1
  DESTINATION lib
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(DIRECTORY
    launch
    config
    models
    DESTINATION share/${PROJECT_NAME}
  )
else()
  install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()