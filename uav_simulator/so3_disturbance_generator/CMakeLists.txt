cmake_minimum_required(VERSION 3.5)
project(so3_disturbance_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  ${ARMADILLO_INCLUDE_DIRS}
)

add_executable(so3_disturbance_generator src/so3_disturbance_generator.cpp)
ament_target_dependencies(so3_disturbance_generator
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  pose_utils
)
target_link_libraries(so3_disturbance_generator
  ${ARMADILLO_LIBRARIES}
)

install(TARGETS so3_disturbance_generator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
