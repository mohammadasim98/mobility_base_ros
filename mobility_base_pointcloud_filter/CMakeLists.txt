cmake_minimum_required(VERSION 2.8.3)
project(mobility_base_pointcloud_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  sensor_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/PointCloudFilter.cpp
  src/filter_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(pointcloud_filter_node src/filter_node.cpp src/PointCloudFilter.cpp)
target_link_libraries(pointcloud_filter_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} pointcloud_filter_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

