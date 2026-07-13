cmake_minimum_required(VERSION 3.5)
project(quadrotor_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PositionCommand.msg"
  "msg/Bspline.msg"
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/LQRTrajectory.msg"
  "msg/Odometry.msg"
  "msg/OutputData.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs
 )

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(quadrotor_msgs_serial SHARED
  src/encode_msgs.cpp
  src/decode_msgs.cpp
)
target_include_directories(quadrotor_msgs_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(quadrotor_msgs_serial "${cpp_typesupport_target}")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS quadrotor_msgs_serial
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(quadrotor_msgs_serial)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
