cmake_minimum_required(VERSION 1.0.0)
project(gtsam_node)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)


#set(GTSAM_DIR "/home/iman/gtsam")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type -O3 -march=native")


find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs ackermann_msgs std_srvs message_generation tf)

## Declare ROS messages and services
add_message_files(FILES ArticulatedAngles.msg CanData.msg)
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs ackermann_msgs message_runtime
  DEPENDS system_lib
)

## Build talker and listener

#include_directories(BEFORE "${PROJECT_SOURCE_DIR}")
#find_package(GTSAM REQUIRED) # Uses installed package

set(GTSAM_DIR "/home/iman/gtsam/build")
find_package(GTSAM)
include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${GTSAM_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})


include_directories(
  ${catkin_INCLUDE_DIRS}
  src
  include
  include/gtsam_node
)




add_executable(gtsam_node src/gtsam_node.cpp)
target_link_libraries(gtsam_node gtsam ${catkin_LIBRARIES} )
add_dependencies(gtsam_node gtsam_node_generate_messages_cpp)

