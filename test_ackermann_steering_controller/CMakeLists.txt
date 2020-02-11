cmake_minimum_required(VERSION 2.8.3)
project(test_ackermann_steering_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  geometry_msgs
  hardware_interface
  nav_msgs
  realtime_tools
  roscpp
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
  include_directories(${catkin_INCLUDE_DIRS})

  add_executable(ackermann_steering_bot test/common/src/ackermann_steering_bot.cpp)
  add_dependencies(tests ackermann_steering_bot)
  target_link_libraries(ackermann_steering_bot ${catkin_LIBRARIES})

  add_rostest_gtest(ackermann_steering_controller_test
    test/ackermann_steering_controller_test/ackermann_steering_controller.test
    test/ackermann_steering_controller_test/ackermann_steering_controller_test.cpp)
  target_link_libraries(ackermann_steering_controller_test ${catkin_LIBRARIES})

  add_rostest_gtest(ackermann_steering_controller_nan_test
    test/ackermann_steering_controller_nan_test/ackermann_steering_controller_nan.test
    test/ackermann_steering_controller_nan_test/ackermann_steering_controller_nan_test.cpp)
  target_link_libraries(ackermann_steering_controller_nan_test ${catkin_LIBRARIES})

  add_rostest_gtest(ackermann_steering_controller_limits_test
    test/ackermann_steering_controller_limits_test/ackermann_steering_controller_limits.test
    test/ackermann_steering_controller_limits_test/ackermann_steering_controller_limits_test.cpp)
  target_link_libraries(ackermann_steering_controller_limits_test ${catkin_LIBRARIES})

  add_rostest_gtest(ackermann_steering_controller_timeout_test
    test/ackermann_steering_controller_timeout_test/ackermann_steering_controller_timeout.test
    test/ackermann_steering_controller_timeout_test/ackermann_steering_controller_timeout_test.cpp)
  target_link_libraries(ackermann_steering_controller_timeout_test ${catkin_LIBRARIES})

  add_rostest_gtest(ackermann_steering_controller_fail_test
    test/ackermann_steering_controller_fail_test/ackermann_steering_controller_wrong.test
    test/ackermann_steering_controller_fail_test/ackermann_steering_controller_fail_test.cpp)
  target_link_libraries(ackermann_steering_controller_fail_test ${catkin_LIBRARIES})

  add_rostest_gtest(ackermann_steering_controller_odom_tf_test
    test/ackermann_steering_controller_odom_tf_test/ackermann_steering_controller_odom_tf.test
    test/ackermann_steering_controller_odom_tf_test/ackermann_steering_controller_odom_tf_test.cpp)
  target_link_libraries(ackermann_steering_controller_odom_tf_test ${catkin_LIBRARIES})

  add_rostest_gtest(ackermann_steering_controller_default_odom_frame_test
    test/ackermann_steering_controller_default_odom_frame_test/ackermann_steering_controller_default_odom_frame.test
    test/ackermann_steering_controller_default_odom_frame_test/ackermann_steering_controller_default_odom_frame_test.cpp)
  target_link_libraries(ackermann_steering_controller_default_odom_frame_test ${catkin_LIBRARIES})

  add_rostest_gtest(ackermann_steering_controller_odom_frame_test
    test/ackermann_steering_controller_odom_frame_test/ackermann_steering_controller_odom_frame.test
    test/ackermann_steering_controller_odom_frame_test/ackermann_steering_controller_odom_frame_test.cpp)
  target_link_libraries(ackermann_steering_controller_odom_frame_test ${catkin_LIBRARIES})

  add_rostest(test/ackermann_steering_controller_open_loop_test/ackermann_steering_controller_open_loop.test)
  add_rostest(test/ackermann_steering_controller_no_wheel_test/ackermann_steering_controller_no_wheel.test)
  add_rostest(test/ackermann_steering_controller_no_steer_test/ackermann_steering_controller_no_steer.test)
  add_rostest(test/ackermann_steering_controller_radius_param_test/ackermann_steering_controller_radius_param.test)
  add_rostest(test/ackermann_steering_controller_radius_param_fail_test/ackermann_steering_controller_radius_param_fail.test)
  add_rostest(test/ackermann_steering_controller_separation_param_test/ackermann_steering_controller_separation_param.test)
endif()
