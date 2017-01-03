cmake_minimum_required(VERSION 2.8.3)
project(rviz_affordance_template_panel)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

set(QT_FORMS ${PROJECT_SOURCE_DIR}/ui/rviz_affordance_template_panel.ui)
set(QT_RESS ${PROJECT_SOURCE_DIR}/resources/images.qrc)

qt4_wrap_cpp(MOC_FILES
    include/${PROJECT_NAME}/msg_headers.h
    include/${PROJECT_NAME}/server_status_monitor.h
    include/${PROJECT_NAME}/rviz_client.h
    include/${PROJECT_NAME}/affordance.h
    include/${PROJECT_NAME}/robot_config.h
    include/${PROJECT_NAME}/waypoint_display.h
    include/${PROJECT_NAME}/controls.h
    include/${PROJECT_NAME}/util.h
    include/${PROJECT_NAME}/rviz_affordance_template_panel.h
)
qt4_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt4_add_resources(QT_RESOURCES ${QT_RESS})

set(SRC_FILES
    src/server_status_monitor.cpp
    src/rviz_client.cpp  
    src/affordance.cpp
    src/controls.cpp
    src/waypoint_display.cpp
    src/util.cpp
    src/rviz_affordance_template_panel.cpp
    ${MOC_FILES}
)

# everything else
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  roscpp
  rviz
  affordance_template_server
  visualization_msgs
  affordance_template_msgs
)

find_package(orocos_kdl REQUIRED)
link_directories(${orocos_kdl_LIBRARY_DIRS})
find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rviz_affordance_template_panel ${KDL_LIBRARY}
  CATKIN_DEPENDS geometry_msgs interactive_markers roscpp roslib rviz affordance_template_server visualization_msgs affordance_template_msgs
  DEPENDS system_lib orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} ${SRC_FILES} ${QT_FORMS_HPP} ${QT_RESOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})