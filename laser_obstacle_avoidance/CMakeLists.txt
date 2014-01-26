cmake_minimum_required(VERSION 2.8.3)
project(laser_obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS signals system)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(laser_obstacle_avoidance_node src/laser_obstacle_avoidance_node.cpp)

target_link_libraries(laser_obstacle_avoidance_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS laser_obstacle_avoidance_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
