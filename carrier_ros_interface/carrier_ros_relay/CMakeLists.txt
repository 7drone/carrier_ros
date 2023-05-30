cmake_minimum_required(VERSION 3.0.2)
project(carrier_ros_relay)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/node_shutdown.py
  src/excute_outdoor_launch.py
  src/excute_indoor_launch.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)