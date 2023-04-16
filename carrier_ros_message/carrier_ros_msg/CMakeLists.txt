cmake_minimum_required(VERSION 3.0.2)
project(carrier_ros_msg)
find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(DIRECTORY msg
FILES
# RobotStart.srv
  test.msg
)


generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)