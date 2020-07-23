cmake_minimum_required(VERSION 2.8.3)
project(gtsam_tutorial)


# if you see a segfault: https://groups.google.com/forum/#!topic/gtsam-users/sDNbMiCtRYI
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})

#find_package(Boost REQUIRED)
#include_directories(${Boost_INCLUDE_DIR})


add_executable( 1_SimpleRotation cpp/1_SimpleRotation.cpp )
target_link_libraries( 1_SimpleRotation  gtsam )



add_executable( 2_CameraResectioning cpp/2_CameraResectioning.cpp )
target_link_libraries( 2_CameraResectioning  gtsam )


add_executable( 3_StereoVOExample cpp/3_StereoVOExample.cpp )
target_link_libraries( 3_StereoVOExample  gtsam )


add_executable( 4_LocalizationExample cpp/4_LocalizationExample.cpp )
target_link_libraries( 4_LocalizationExample  gtsam )



add_executable( 5_OdometryExample cpp/5_OdometryExample.cpp )
target_link_libraries( 5_OdometryExample  gtsam )


add_executable( ImuFactorsExample cpp/ImuFactorsExample.cpp )
target_link_libraries( ImuFactorsExample  gtsam )


add_executable( 1_fitline custom_factors/1_fitline.cpp )
target_link_libraries( 1_fitline  gtsam )
