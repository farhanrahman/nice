cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_boost_directories()

set(OTL_SOURCES
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_exception.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_helpers.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_helpers.cpp
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_kernel.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_kernel_factory.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_kernel_gaussian.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_kernel_gaussian.cpp
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_kernel_recursive_gaussian.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_kernel_recursive_gaussian.cpp
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_kernel_recursive_equality_gaussian.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_kernel_recursive_equality_gaussian.cpp
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_learning_algs.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_rls.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_rls.cpp
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_aug_state.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_reservoir.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_reservoir.cpp
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_window.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_window.cpp
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_sogp.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_sogp.cpp
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_oesgp.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_oesgp.cpp
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_storkgp.h
    $ENV{ROS_WORKSPACE}/nice/otl/src/libOTL/otl_storkgp.cpp
)

add_library(OTL ${OTL_SOURCES})

include_directories($ENV{ROS_WORKSPACE}/nice/otl/src/libOTL)

include_directories($ENV{ROS_WORKSPACE}/nice/nice_predictor/include)

include_directories($ENV{ROS_WORKSPACE}/nice/costmap_update/include)

rosbuild_add_executable(${PROJECT_NAME} src/main.cpp src/NicePredictor.cpp)

target_link_libraries(${PROJECT_NAME} OTL)

rosbuild_link_boost(${PROJECT_NAME} thread)