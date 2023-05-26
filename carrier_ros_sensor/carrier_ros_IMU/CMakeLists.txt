cmake_minimum_required(VERSION 3.0.2)
project(carrier_ros_imu)

###################################
# Packages
###################################
find_package(catkin REQUIRED
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  tf
  geometry_msgs
  sensor_msgs_ext
  message_generation
)

## add Service....
add_service_files(
  FILES
  all_data_reset.srv
  euler_angle_init.srv
  euler_angle_reset.srv
  pose_velocity_reset.srv
  reboot_sensor.srv
)

generate_messages(   
  DEPENDENCIES  
  std_msgs
  std_srvs
  geometry_msgs
) 

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES iahrs_driver
  CATKIN_DEPENDS roscpp std_msgs std_srvs sensor_msgs tf
)

###########
## Build ##
###########
include_directories(
 ${catkin_INCLUDE_DIRS}
)

#add_compile_options(-std=c++11)
add_executable(carrier_ros_imu src/iahrs_driver.cpp)
add_dependencies(carrier_ros_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(carrier_ros_imu ${catkin_LIBRARIES})

add_executable(carrier_ros_imu_heading src/imu_heading.cpp)
add_dependencies(carrier_ros_imu_heading ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(carrier_ros_imu_heading ${catkin_LIBRARIES})
