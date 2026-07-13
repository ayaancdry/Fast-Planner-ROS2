cmake_minimum_required(VERSION 3.5)
project(local_sensing_node)

set(CMAKE_BUILD_TYPE Release)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# The ROS1 version of this package also had a CUDA-accelerated depth-render
# path (src/depth_render.cu, src/pcl_render_node.cpp) gated behind
# ENABLE_CUDA, which defaulted to OFF and was therefore never actually built.
# Only the CPU point-cloud path (pointcloud_render_node.cpp) is ported here;
# the CUDA path still needs its own ROS2 + dynamic_reconfigure-removal port.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcl_render_node src/pointcloud_render_node.cpp)
ament_target_dependencies(pcl_render_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
)
target_link_libraries(pcl_render_node ${PCL_LIBRARIES})

install(TARGETS pcl_render_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

ament_package()
