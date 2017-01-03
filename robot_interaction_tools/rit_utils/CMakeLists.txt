cmake_minimum_required(VERSION 2.8.3)
project(rit_utils)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  srdfdom
  sensor_msgs
  kdl_parser
)

find_package(urdfdom REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} predictive_robot_display
  CATKIN_DEPENDS srdfdom roscpp sensor_msgs kdl_parser
  DEPENDS system_lib urdfdom
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${srdfdom_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

add_library(rit_utils
  src/rdf_model.cpp
  include/rit_utils/generic_utils.h
)
target_link_libraries(rit_utils ${srdfdom_LIBRARIES} ${urdfdom_LIBRARIES} ${kdl_parser_LIBRARIES})

#add_executable(display_test src/test_robot_display.cpp)
#target_link_libraries(display_test ${catkin_LIBRARIES})

add_library(predictive_robot_display src/predictive_robot_display.cpp)
target_link_libraries(predictive_robot_display ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY})

install(TARGETS ${PROJECT_NAME} predictive_robot_display
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
