cmake_minimum_required(VERSION 3.5)
project(uav_utils)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/odom_to_euler.py
  scripts/send_odom.py
  scripts/tf_assist.py
  scripts/topic_statistics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test src/uav_utils_test.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    ament_target_dependencies(${PROJECT_NAME}-test nav_msgs geometry_msgs)
  endif()
endif()

ament_package()
