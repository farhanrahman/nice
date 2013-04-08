cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

project(rrttest)

include_directories($ENV{ROS_WORKSPACE}/nice/nice_global_planner/include/)
include_directories(/opt/ros/fuerte/include/)
include_directories($ENV{ROS_WORKSPACE}/nice/utils/include/)

add_executable(rrttest main.cpp $ENV{ROS_WORKSPACE}/nice/nice_global_planner/src/RRTPlanner.cpp)