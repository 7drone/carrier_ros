cmake_minimum_required(VERSION 3.0.2)
project(carrier_ros_msg)
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(DIRECTORY msg
  FILES
  BatteryTwo.msg
  BatteryOne.msg

)


generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)