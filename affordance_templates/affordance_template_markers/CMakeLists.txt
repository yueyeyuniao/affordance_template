cmake_minimum_required(VERSION 2.8.3)
project(affordance_template_markers)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
    affordance_template_msgs
    affordance_template_library
    interactive_markers
    planner_interface
    end_effector_helper
    geometry_msgs
    nav_msgs
    std_msgs
    roscpp
    actionlib
    end_effector_helper
)
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
    INCLUDE_DIRS include
	CATKIN_DEPENDS affordance_template_msgs affordance_template_library end_effector_helper geometry_msgs interactive_markers nav_msgs roscpp std_msgs planner_interface actionlib end_effector_helper 
    LIBRARIES robot_interface affordance_template
    DEPENDS system_lib yaml-cpp
)

include_directories(
    include
    yaml-cpp
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(robot_interface src/robot_interface.cpp)
target_link_libraries(robot_interface ${catkin_LIBRARIES} yaml-cpp)

add_library(affordance_template src/affordance_template.cpp)
target_link_libraries(affordance_template ${catkin_LIBRARIES} yaml-cpp ${Boost_FILESYSTEM_LIBRARY})

install(TARGETS affordance_template robot_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)