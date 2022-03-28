cmake_minimum_required(VERSION 2.8.3)
project(transform_pointcloud)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        tf
        nodelet
        tf2
        tf2_geometry_msgs
        tf2_sensor_msgs)

catkin_package(
        LIBRARIES 
        transform_pointcloud_nodelet
        roscpp
        sensor_msgs
        tf
        nodelet
        tf2
        tf2_geometry_msgs
        tf2_sensor_msgs
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(transformPointcloud src/transform_pointcloud_nodelet.cpp)
target_link_libraries(transformPointcloud ${catkin_LIBRARIES})
add_dependencies(transformPointcloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Mark library for installation.
install(TARGETS
        transformPointcloud 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} FILES_MATCHING PATTERN "*.launch")
install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
