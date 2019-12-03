cmake_minimum_required(VERSION 2.8.3)
project(initial_pose_publisher)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  roslint
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES initial_pose_publisher
#  CATKIN_DEPENDS geometry_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/initial_pose_publisher_node.cpp
  src/initial_pose_publisher.cpp
)

roslint_cpp(
  src/initial_pose_publisher_node.cpp
  src/initial_pose_publisher.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
