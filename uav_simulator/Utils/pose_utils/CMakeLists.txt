cmake_minimum_required(VERSION 3.5)
project(pose_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  include
  ${ARMADILLO_INCLUDE_DIRS}
)

add_library(pose_utils SHARED
  src/pose_utils.cpp
)
target_include_directories(pose_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pose_utils ${ARMADILLO_LIBRARIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS pose_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(pose_utils)
ament_package()
