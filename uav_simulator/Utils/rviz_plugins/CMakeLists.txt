cmake_minimum_required(VERSION 3.5)
project(rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(multi_map_server REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/pose_tool.cpp
  src/goal_tool.cpp
  src/probmap_display.cpp
  src/aerialmap_display.cpp
  src/multi_probmap_display.cpp
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  pluginlib
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  multi_map_server
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
