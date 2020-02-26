cmake_minimum_required(VERSION 2.8.3)
project(hector_elevation_mapping)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hector_elevation_msgs
  hector_map_tools
  nodelet
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ElevationMapping
  CATKIN_DEPENDS geometry_msgs hector_elevation_msgs hector_map_tools nodelet pcl_ros roscpp sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(ElevationMapping src/hector_elevation_mapping.cpp)

## Declare a cpp executable
add_executable(ElevationMappingNode src/hector_elevation_mapping_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(ElevationMapping hector_elevation_msgs_generate_messages_cpp)
add_dependencies(ElevationMappingNode hector_elevation_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ElevationMapping
  ${catkin_LIBRARIES}
)
target_link_libraries(ElevationMappingNode
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ElevationMappingNode RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ElevationMapping LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
