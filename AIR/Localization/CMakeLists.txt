cmake_minimum_required(VERSION 3.0.2)
project(localization)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  # jsoncpp
)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
## Find GeographicLib
find_package(GeographicLib REQUIRED)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES localization
  CATKIN_DEPENDS roscpp std_msgs nav_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/localization.cc)

## Specify libraries to link the executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

add_executable(test_gps_node src/test_gps.cc)
target_link_libraries(test_gps_node
  ${catkin_LIBRARIES}
)