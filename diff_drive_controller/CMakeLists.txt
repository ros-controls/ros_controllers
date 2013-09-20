cmake_minimum_required(VERSION 2.8.3)
project(diff_drive_controller)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_add_library(${PROJECT_NAME} src/diff_drive_controller.cpp
                                       include/diff_drive_controller/odometry.h)


  rosbuild_add_executable(diffbot test/diffbot.cpp)
  rosbuild_add_executable(diff_drive_test test/diff_drive_test.cpp)
  rosbuild_add_gtest_build_flags(diff_drive_test)
  rosbuild_add_rostest(test/diff_drive_controller.test)
