cmake_minimum_required(VERSION 2.8.3)
project(bici_ros_sensor_reader)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  TactileData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensor_parser src/sensor_parser.cpp src/sensor_parser.h)
add_dependencies(sensor_parser bici_ros_sensor_reader_generate_messages_cpp)

target_link_libraries(sensor_parser ${catkin_LIBRARIES})

