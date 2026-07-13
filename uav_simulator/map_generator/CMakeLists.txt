cmake_minimum_required(VERSION 3.5)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(random_forest src/random_forest_sensing.cpp)
ament_target_dependencies(random_forest
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  pcl_conversions
)
target_link_libraries(random_forest ${PCL_LIBRARIES})

install(TARGETS random_forest
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
