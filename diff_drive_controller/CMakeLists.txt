cmake_minimum_required(VERSION 2.8.3)
project(diff_drive_controller)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    control_msgs
    dynamic_reconfigure
    nav_msgs
    realtime_tools
    tf
    urdf
)

generate_dynamic_reconfigure_options(cfg/DiffDriveController.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/diff_drive_controller.cpp src/odometry.cpp src/speed_limiter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(FILES diff_drive_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS controller_manager rosgraph_msgs rostest std_srvs tf)
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
