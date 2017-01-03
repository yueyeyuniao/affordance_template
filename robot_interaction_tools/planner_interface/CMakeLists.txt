cmake_minimum_required(VERSION 2.8.3)
project(planner_interface)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  message_generation
  genmsg
  control_msgs
  geometry_msgs
  moveit_ros_planning_interface
  pluginlib
  roscpp
  rospy
  sensor_msgs
  moveit_msgs
  trajectory_msgs
  visualization_msgs
  kdl_conversions
  robot_interaction_tools_msgs
  srdfdom
  rit_utils
)

find_package(urdfdom REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetObstacles.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  visualization_msgs
  moveit_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS message_runtime control_msgs rit_utils srdfdom geometry_msgs moveit_msgs moveit_ros_planning_interface pluginlib roscpp rospy kdl_conversions sensor_msgs trajectory_msgs visualization_msgs robot_interaction_tools_msgs
   DEPENDS system_lib urdfdom
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${srdfdom_INCLUDE_DIRS}
  ${rit_utils_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

add_library(planner_interface
  src/planner_interface.cpp
  src/tolerance_util.cpp
)

## Declare a cpp executable
add_executable(tolerance_util src/tolerance_util.cpp)

target_link_libraries(${PROJECT_NAME} ${urdfdom_LIBRARIES} ${srdfdom_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(tolerance_util ${catkin_LIBRARIES})

add_dependencies(planner_interface robot_interaction_tools_msgs ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
