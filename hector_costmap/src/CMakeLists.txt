#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#set(ROS_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)

rosbuild_add_executable(hector_costmap hector_costmap_node.cpp hector_costmap.h hector_costmap.cpp)
target_link_libraries(hector_costmap ${OpenCV_LIBS})
