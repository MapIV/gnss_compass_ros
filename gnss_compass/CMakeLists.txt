cmake_minimum_required(VERSION 3.5)
project(gnss_compass)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(llh_converter REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(gnss_compass
        src/gnss_compass_node.cpp
        src/gnss_compass_core.cpp
        src/util.cpp
)

ament_target_dependencies(gnss_compass
  rclcpp
  nmea_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  llh_converter
  tf2
  tf2_ros
  tf2_geometry_msgs
  diagnostic_msgs
)

install(TARGETS gnss_compass
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()