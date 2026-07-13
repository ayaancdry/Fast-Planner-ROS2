cmake_minimum_required(VERSION 3.5)
project(so3_quadrotor_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)

# include/ode is a vendored copy of boost::numeric::odeint (plain C++/boost,
# no ROS dependency) kept as-is from the ROS1 version.
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ode
  ${EIGEN3_INCLUDE_DIR}
  ${ARMADILLO_INCLUDE_DIRS}
)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)
target_include_directories(quadrotor_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
ament_target_dependencies(quadrotor_simulator_so3
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  quadrotor_msgs
  uav_utils
)
target_link_libraries(quadrotor_simulator_so3 quadrotor_dynamics)

install(TARGETS quadrotor_simulator_so3
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
