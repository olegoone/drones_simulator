cmake_minimum_required(VERSION 2.8.3)
project(drones_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  rospy
  roscpp
  tf
  airsim_ros_pkgs
)


add_message_files(
  FILES
  DroneWaypoint.msg
  DronePath.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(drones_controller_node
  src/drones_controller_node.cpp
  src/DronesController.cpp
)

target_link_libraries(drones_controller_node
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS drones_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
