cmake_minimum_required(VERSION 3.0.2)
project(gnss_compass)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nmea_msgs
  roscpp
  geometry_msgs
  nav_msgs
  llh_converter
  tf2
  tf2_ros
  tf2_geometry_msgs
  diagnostic_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gnss_compass
        src/gnss_compass_node.cpp
        src/gnss_compass_core.cpp
)

add_dependencies(gnss_compass ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(gnss_compass ${catkin_LIBRARIES})

install(TARGETS
            gnss_compass
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY
            rviz
            config
            launch
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

