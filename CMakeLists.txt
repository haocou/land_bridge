cmake_minimum_required(VERSION 2.8.3)

project(land_bridge)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
    tf
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_message_files(
  FILES
  ControlCommand.msg
  DroneState.msg
  PositionReference.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime 
)

add_executable(land_bridge ${PROJECT_SOURCE_DIR}/src/droneState.cpp)
add_dependencies(land_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(land_bridge ${catkin_LIBRARIES})