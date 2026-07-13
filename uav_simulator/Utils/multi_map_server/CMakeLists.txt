cmake_minimum_required(VERSION 3.5)
project(multi_map_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Armadillo REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultiOccupancyGrid.msg"
  "msg/MultiSparseMap3D.msg"
  "msg/SparseMap3D.msg"
  "msg/VerticalOccupancyGridList.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include
  ${ARMADILLO_INCLUDE_DIRS}
)

add_executable(multi_map_visualization src/multi_map_visualization.cc)
target_include_directories(multi_map_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(multi_map_visualization
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pose_utils
  tf2
)
target_link_libraries(multi_map_visualization
  "${cpp_typesupport_target}"
  ${ARMADILLO_LIBRARIES}
)

install(TARGETS multi_map_visualization
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
