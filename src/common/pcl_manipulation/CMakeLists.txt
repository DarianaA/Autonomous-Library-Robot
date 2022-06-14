cmake_minimum_required(VERSION 3.0.2)
project(pcl_manipulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  sensor_msgs
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SegmentPlane.srv
  RemoveOutliers.srv
  Downsample.srv
  Crop.srv
  Euclidian.srv
  CartesianFilter.srv
  MinMax.srv
  Centroid.srv
  CombineClouds.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(pcl_manipulation_server src/pcl_manipulation_server.cpp src/segmentation.cpp src/downsample.cpp src/euclidian.cpp src/cartesian_filter.cpp src/min_max.cpp src/centroid.cpp src/combine_clouds.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(pcl_manipulation_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
   pcl_manipulation_server
   ${catkin_LIBRARIES}
)
