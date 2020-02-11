cmake_minimum_required(VERSION 2.8.3)
project(test_diff_drive_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  controller_manager
  dynamic_reconfigure
  geometry_msgs
  hardware_interface
  nav_msgs
  realtime_tools
  roscpp
  rosgraph_msgs
  std_msgs
  std_srvs
  tf
)

# Declare a catkin package
catkin_package()


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Specify header include paths
  include_directories(include ${catkin_INCLUDE_DIRS})

  add_executable(diffbot test/diffbot.cpp)
  target_link_libraries(diffbot ${catkin_LIBRARIES})

  add_executable(skidsteerbot test/skidsteerbot.cpp)
  target_link_libraries(skidsteerbot ${catkin_LIBRARIES})

  add_dependencies(tests diffbot skidsteerbot)

  add_rostest_gtest(diff_drive_test
    test/diff_drive_controller.test
    test/diff_drive_test.cpp)
  target_link_libraries(diff_drive_test ${catkin_LIBRARIES})
  add_rostest_gtest(diff_drive_nan_test
    test/diff_drive_controller_nan.test
    test/diff_drive_nan_test.cpp)
  target_link_libraries(diff_drive_nan_test ${catkin_LIBRARIES})
  add_rostest_gtest(diff_drive_limits_test
    test/diff_drive_controller_limits.test
    test/diff_drive_limits_test.cpp)
  target_link_libraries(diff_drive_limits_test ${catkin_LIBRARIES})
  add_rostest_gtest(diff_drive_timeout_test
    test/diff_drive_timeout.test
    test/diff_drive_timeout_test.cpp)
  target_link_libraries(diff_drive_timeout_test ${catkin_LIBRARIES})
  add_rostest(test/diff_drive_multipliers.test)
  add_rostest(test/diff_drive_left_right_multipliers.test)
  add_rostest_gtest(diff_drive_fail_test
    test/diff_drive_wrong.test
    test/diff_drive_fail_test.cpp)
  target_link_libraries(diff_drive_fail_test ${catkin_LIBRARIES})
  add_rostest_gtest(diff_drive_odom_tf_test
    test/diff_drive_odom_tf.test
    test/diff_drive_odom_tf_test.cpp)
  target_link_libraries(diff_drive_odom_tf_test ${catkin_LIBRARIES})
  add_rostest_gtest(diff_drive_default_cmd_vel_out_test
    test/diff_drive_default_cmd_vel_out.test
    test/diff_drive_default_cmd_vel_out_test.cpp)
  target_link_libraries(diff_drive_default_cmd_vel_out_test ${catkin_LIBRARIES})
  add_rostest_gtest(diff_drive_pub_cmd_vel_out_test
    test/diff_drive_pub_cmd_vel_out.test
    test/diff_drive_pub_cmd_vel_out_test.cpp)
  target_link_libraries(diff_drive_pub_cmd_vel_out_test ${catkin_LIBRARIES})
  add_rostest_gtest(diff_drive_default_wheel_joint_controller_state_test
    test/diff_drive_default_wheel_joint_controller_state.test
    test/diff_drive_default_wheel_joint_controller_state_test.cpp)
  target_link_libraries(diff_drive_default_wheel_joint_controller_state_test ${catkin_LIBRARIES})
  add_rostest_gtest(diff_drive_publish_wheel_joint_controller_state_test
    test/diff_drive_publish_wheel_joint_controller_state.test
    test/diff_drive_publish_wheel_joint_controller_state_test.cpp)
  target_link_libraries(diff_drive_publish_wheel_joint_controller_state_test ${catkin_LIBRARIES})
  add_rostest_gtest(diff_drive_default_odom_frame_test
    test/diff_drive_default_odom_frame.test
    test/diff_drive_default_odom_frame_test.cpp)
  target_link_libraries(diff_drive_default_odom_frame_test ${catkin_LIBRARIES})
  add_rostest_gtest(diff_drive_odom_frame_test
    test/diff_drive_odom_frame.test
    test/diff_drive_odom_frame_test.cpp)
  target_link_libraries(diff_drive_odom_frame_test ${catkin_LIBRARIES})
  add_rostest_gtest(diff_drive_multiple_cmd_vel_publishers_test
    test/diff_drive_multiple_cmd_vel_publishers.test
    test/diff_drive_multiple_cmd_vel_publishers_test.cpp)
  target_link_libraries(diff_drive_multiple_cmd_vel_publishers_test ${catkin_LIBRARIES})
  add_rostest(test/diff_drive_open_loop.test)
  add_rostest(test/skid_steer_controller.test)
  add_rostest(test/skid_steer_no_wheels.test)
  add_rostest(test/diff_drive_radius_sphere.test)
  add_rostest(test/diff_drive_radius_param.test)
  add_rostest(test/diff_drive_radius_param_fail.test)
  add_rostest(test/diff_drive_separation_param.test)
  add_rostest(test/diff_drive_bad_urdf.test)
  add_rostest_gtest(diff_drive_dyn_reconf_test
    test/diff_drive_dyn_reconf.test
    test/diff_drive_dyn_reconf_test.cpp)
  target_link_libraries(diff_drive_dyn_reconf_test ${catkin_LIBRARIES})
endif()
