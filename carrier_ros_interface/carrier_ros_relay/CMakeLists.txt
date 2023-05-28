cmake_minimum_required(VERSION 3.0.2)
project(carrier_ros_relay)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  carrier_ros_msg
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(battery_node src/battery_node.cpp)
target_link_libraries(battery_node ${catkin_LIBRARIES})
add_dependencies(battery_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})