cmake_minimum_required(VERSION 2.8.3)
project(mobility_base_examples)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)
