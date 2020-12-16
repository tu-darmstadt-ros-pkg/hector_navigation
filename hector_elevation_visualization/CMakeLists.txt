cmake_minimum_required(VERSION 3.0.2)
project(hector_elevation_visualization)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  hector_elevation_msgs
  hector_map_tools
  roscpp
  tf
  visualization_msgs
)
generate_dynamic_reconfigure_options(cfg/ElevationVisualization.cfg)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hector_elevation_visualization
  CATKIN_DEPENDS dynamic_reconfigure hector_elevation_msgs hector_map_tools roscpp tf visualization_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(hector_elevation_visualization_node src/hector_elevation_visualization_node.cpp src/hector_elevation_visualization.cpp)
add_dependencies(hector_elevation_visualization_node hector_elevation_msgs_generate_messages_cpp ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(hector_elevation_visualization_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS hector_elevation_visualization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

