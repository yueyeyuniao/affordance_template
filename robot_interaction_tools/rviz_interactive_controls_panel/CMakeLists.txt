cmake_minimum_required(VERSION 2.8.3)
project(rviz_interactive_controls_panel)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rviz 
  actionlib 
  planner_interface 
  robot_interaction_tools_msgs
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

include_directories(
  include
  ${OGRE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  CATKIN_DEPENDS roscpp rviz actionlib planner_interface robot_interaction_tools_msgs
)

set(QT_FORMS 
  ${PROJECT_SOURCE_DIR}/ui/rviz_panel.ui
  ${PROJECT_SOURCE_DIR}/ui/group_controls_widget.ui
  ${PROJECT_SOURCE_DIR}/ui/multi_group_controls_widget.ui
  #${PROJECT_SOURCE_DIR}/ui/navigation_controls_widget.ui
)

qt4_wrap_cpp(MOC_SOURCES
    include/rviz_interactive_controls_panel/rviz_panel.h
    include/rviz_interactive_controls_panel/group_controls_widget.h
    include/rviz_interactive_controls_panel/interface_utils.h
    include/rviz_interactive_controls_panel/multi_group_controls_widget.h
    #include/rviz_interactive_controls_panel/navigation_controls_widget.h
    include/rviz_interactive_controls_panel/rosparam_dialog.h
    include/rviz_interactive_controls_panel/rviz_panel.h
)
qt4_wrap_ui(QT_FORMS_HPP ${QT_FORMS})

add_library(${PROJECT_NAME} 
  ${MOC_SOURCES}
  ${QT_FORMS_HPP}
  src/rviz_panel.cpp
  src/group_controls_widget.cpp
  src/interface_utils.cpp
  src/multi_group_controls_widget.cpp
  #src/navigation_controls_widget.cpp
  src/rosparam_dialog.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${planner_interface_LIBRARIES}
  ${QT_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${OGRE_LIBRARIES}
)