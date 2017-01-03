cmake_minimum_required(VERSION 2.8.3)
project(affordance_template_server)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    affordance_template_msgs
    affordance_template_markers
    affordance_template_library
    interactive_markers
    actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp affordance_template_msgs affordance_template_markers tf affordance_template_library interactive_markers 
    LIBRARIES ${PROJECT_NAME} 
    DEPENDS system_lib Boost affordance_template_library
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/interface.cpp src/server.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/interface.cpp src/server.cpp src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

#add_executable(plan_test src/test_plan.cpp)
#target_link_libraries(plan_test ${catkin_LIBRARIES} affordance_template)
#add_executable(exe_test src/test_exe.cpp)
#target_link_libraries(exe_test ${catkin_LIBRARIES} affordance_template)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)