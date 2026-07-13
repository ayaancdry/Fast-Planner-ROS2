cmake_minimum_required(VERSION 3.5)
project(odom_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pose_utils REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  include
  ${ARMADILLO_INCLUDE_DIRS}
)

add_executable(odom_visualization src/odom_visualization.cpp)
ament_target_dependencies(odom_visualization
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  pose_utils
  quadrotor_msgs
)
target_link_libraries(odom_visualization ${ARMADILLO_LIBRARIES})

install(TARGETS odom_visualization
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
