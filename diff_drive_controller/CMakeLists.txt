cmake_minimum_required(VERSION 2.8.3)
project(diff_drive_controller)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    nav_msgs
    realtime_tools
    tf
    urdf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/diff_drive_controller.cpp src/odometry.cpp src/speed_limiter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES diff_drive_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS controller_manager rostest std_srvs tf)
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
  add_rostest_gtest(diff_drive_default_odom_frame_test
    test/diff_drive_default_odom_frame.test
    test/diff_drive_default_odom_frame_test.cpp)
  target_link_libraries(diff_drive_default_odom_frame_test ${catkin_LIBRARIES})
  add_rostest_gtest(diff_drive_odom_frame_test
    test/diff_drive_odom_frame.test
    test/diff_drive_odom_frame_test.cpp)
  target_link_libraries(diff_drive_odom_frame_test ${catkin_LIBRARIES})
  add_rostest(test/diff_drive_open_loop.test)
  add_rostest(test/skid_steer_controller.test)
  add_rostest(test/skid_steer_no_wheels.test)
  add_rostest(test/diff_drive_radius_sphere.test)
  add_rostest(test/diff_drive_radius_param.test)
  add_rostest(test/diff_drive_radius_param_fail.test)
  add_rostest(test/diff_drive_separation_param.test)
  add_rostest(test/diff_drive_bad_urdf.test)
endif()
