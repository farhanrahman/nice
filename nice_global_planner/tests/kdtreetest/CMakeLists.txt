cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(kdtree_search)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories($ENV{ROS_WORKSPACE}/nice/nice_global_planner/include/)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (kdtree_search kdtree_search.cpp)
#add_executable(kdtree_search main.cpp)
target_link_libraries (kdtree_search ${PCL_LIBRARIES})
