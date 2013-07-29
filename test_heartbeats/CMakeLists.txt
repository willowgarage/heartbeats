cmake_minimum_required(VERSION 2.8.3)
project(test_heartbeats)

find_package(catkin REQUIRED COMPONENTS heartbeats rospy rostest rosunit)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_heartbeats
#  CATKIN_DEPENDS heartbeats rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_rostest(test/rostest/test_heartbeats.test)
