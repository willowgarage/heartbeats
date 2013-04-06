cmake_minimum_required(VERSION 2.8.3)
project(heartbeats)

find_package(catkin REQUIRED COMPONENTS rospy message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Heartbeat.msg
  HeartbeatUpdate.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

#catkin_python_setup()

catkin_package(DEPENDS std_msgs)
