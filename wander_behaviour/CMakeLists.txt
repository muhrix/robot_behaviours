cmake_minimum_required(VERSION 2.8.3)
project(wander_behaviour)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS signals system)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(wander_node src/wander_behaviour.cpp)

target_link_libraries(wander_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS wander_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
