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



#### Custom factors
add_executable( 1_fit_h_eq_y_minus_ax custom_factors/1_fit_h_eq_y_minus_ax.cpp )
target_link_libraries( 1_fit_h_eq_y_minus_ax  gtsam )


add_executable( 2_fit_y_eq_ax_plus_b custom_factors/2_fit_y_eq_ax_plus_b.cpp )
target_link_libraries( 2_fit_y_eq_ax_plus_b  gtsam )


add_executable( 3_fit_pose custom_factors/3_fit_pose.cpp )
target_link_libraries( 3_fit_pose  gtsam )
