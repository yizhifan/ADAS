cmake_minimum_required(VERSION 2.8.3)
project(framework_status_msgs)
find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation std_msgs system_manager_msgs geometry_msgs)
add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES std_msgs system_manager_msgs geometry_msgs)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package()