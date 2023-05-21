################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(carrier_ros_navigation)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3 ${CMAKE_CXX_FLAGS}")
################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
    rospy
    actionlib
    actionlib_msgs
    std_msgs
    # message_generation
    roscpp
    geometry_msgs
    tf2
    tf2_ros
    carrier_ros_srv
)

################################################################################
# Setup for python modules and scripts
################################################################################

catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

# add_service_files(
#   FILES
#   Speak.srv
# )


# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )
################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES carrier_ros_teleop
 CATKIN_DEPENDS rospy std_msgs 
#  DEPENDS system_lib
)



################################################################################
# Build
################################################################################
add_executable(carrier_ros_nav_interface src/carrier_ros_nav_interface.cpp)
target_link_libraries(carrier_ros_nav_interface ${catkin_LIBRARIES})
add_dependencies(carrier_ros_nav_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

include_directories(
  # include 
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/src
)
################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  # nodes/init_pose.py
  # nodes/carrier_ros_costmap_clear.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch maps param rviz bag
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
