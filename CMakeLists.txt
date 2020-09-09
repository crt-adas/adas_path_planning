cmake_minimum_required(VERSION 1.0.0)
project(mhe_estimator)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type -O3 -march=native")


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs ackermann_msgs std_srvs message_generation tf)

## Declare ROS messages and services
add_message_files(FILES ArticulatedAngles.msg CanData.msg)
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs std_srvs)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs ackermann_msgs std_srvs message_runtime
  DEPENDS system_lib
)


## Build talker and listener
find_package(CASADI REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
  include
  include/mhe_estimator
  ${CASADI_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mhe_estimator_node src/mhe_estimator_node.cpp)
target_link_libraries(mhe_estimator_node ${catkin_LIBRARIES} ${CASADI_LIBRARIES})
add_dependencies(mhe_estimator_node mhe_estimator_generate_messages_cpp)

