cmake_minimum_required(VERSION 2.8.3)
project(gazebo_set_joint_positions)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    roscpp
    sensor_msgs
    roslint
)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp gazebo sensor_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
) 

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/set_joint_positions.h
    src/set_joint_positions.cpp
)

target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(ROSLINT_CPP_OPTS  "--filter=-whitespace/line_length,-readability/todo,-runtime/references,-build/c++11" )
roslint_cpp()
