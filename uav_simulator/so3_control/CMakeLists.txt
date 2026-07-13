cmake_minimum_required(VERSION 3.5)
project(so3_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(SO3Control src/SO3Control.cpp)
target_include_directories(SO3Control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(so3_control_component SHARED src/so3_control_component.cpp)
target_include_directories(so3_control_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(so3_control_component
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  std_msgs
  quadrotor_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(so3_control_component SO3Control)

rclcpp_components_register_node(so3_control_component
  PLUGIN "so3_control::SO3ControlComponent"
  EXECUTABLE so3_control_node
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS SO3Control so3_control_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(SO3Control so3_control_component)
ament_package()
