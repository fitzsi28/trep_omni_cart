cmake_minimum_required(VERSION 2.8.3)
project(trep_omni_cart)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs visualization_msgs genmsg)

add_message_files(FILES trepsys.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   # INCLUDE_DIRS include
   CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs genmsg visualization_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
