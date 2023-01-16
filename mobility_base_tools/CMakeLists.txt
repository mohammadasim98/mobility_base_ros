cmake_minimum_required(VERSION 2.8.3)
project(mobility_base_tools)

find_package(catkin REQUIRED COMPONENTS 
  roslaunch
  rospy
)

catkin_python_setup()

catkin_package()

install(DIRECTORY launch rviz resource images
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)

install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
