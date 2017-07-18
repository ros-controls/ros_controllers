cmake_minimum_required(VERSION 2.8.3)
project(steer_drive_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  urdf
  realtime_tools
  tf
  nav_msgs
  roscpp
  angles
  control_toolbox
  gazebo_ros_control
  hardware_interface
  joint_limits_interface
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  angles
  control_toolbox
  gazebo_ros_control
  hardware_interface
  joint_limits_interface
)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} src/steer_drive_controller.cpp src/odometry.cpp src/speed_limiter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES steer_drive_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS
    rostest
    std_srvs
    controller_manager
    tf
  )
  include_directories(include ${catkin_INCLUDE_DIRS})

  add_executable(${PROJECT_NAME}_steerbot test/common/src/steerbot.cpp)
  target_link_libraries(${PROJECT_NAME}_steerbot ${catkin_LIBRARIES})

  add_dependencies(tests ${PROJECT_NAME}_steerbot)

  add_rostest_gtest(${PROJECT_NAME}_test
    test/steer_drive_controller_test/steer_drive_controller.test
    test/steer_drive_controller_test/steer_drive_controller_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})
  add_rostest_gtest(${PROJECT_NAME}_nan_test
    test/steer_drive_controller_nan_test/steer_drive_controller_nan.test
    test/steer_drive_controller_nan_test/steer_drive_controller_nan_test.cpp)
  target_link_libraries(${PROJECT_NAME}_nan_test ${catkin_LIBRARIES})
  add_rostest_gtest(${PROJECT_NAME}_limits_test
    test/steer_drive_controller_limits_test/steer_drive_controller_limits.test
    test/steer_drive_controller_limits_test/steer_drive_controller_limits_test.cpp)
  target_link_libraries(${PROJECT_NAME}_limits_test ${catkin_LIBRARIES})
  add_rostest_gtest(${PROJECT_NAME}_timeout_test
    test/steer_drive_controller_timeout_test/steer_drive_controller_timeout.test
    test/steer_drive_controller_timeout_test/steer_drive_controller_timeout_test.cpp)
  target_link_libraries(${PROJECT_NAME}_timeout_test ${catkin_LIBRARIES})
  add_rostest_gtest(steer_drive_controller_fail_test
    test/steer_drive_controller_fail_test/steer_drive_controller_wrong.test
    test/steer_drive_controller_fail_test/steer_drive_controller_fail_test.cpp)
  target_link_libraries(steer_drive_controller_fail_test ${catkin_LIBRARIES})
  add_rostest_gtest(${PROJECT_NAME}_odom_tf_test
    test/steer_drive_controller_odom_tf_test/steer_drive_controller_odom_tf.test
    test/steer_drive_controller_odom_tf_test/steer_drive_controller_odom_tf_test.cpp)
  target_link_libraries(${PROJECT_NAME}_odom_tf_test ${catkin_LIBRARIES})
  add_rostest_gtest(${PROJECT_NAME}_default_odom_frame_test
    test/steer_drive_controller_default_odom_frame_test/steer_drive_controller_default_odom_frame.test
    test/steer_drive_controller_default_odom_frame_test/steer_drive_controller_default_odom_frame_test.cpp)
  target_link_libraries(${PROJECT_NAME}_default_odom_frame_test ${catkin_LIBRARIES})
  add_rostest_gtest(steer_drive_controller_odom_frame_test
    test/steer_drive_controller_odom_frame_test/steer_drive_controller_odom_frame.test
    test/steer_drive_controller_odom_frame_test/steer_drive_controller_odom_frame_test.cpp)
  target_link_libraries(steer_drive_controller_odom_frame_test ${catkin_LIBRARIES})
  add_rostest(test/steer_drive_controller_open_loop_test/steer_drive_controller_open_loop.test)
  add_rostest(test/steer_drive_controller_no_wheel_test/steer_drive_controller_no_wheel.test)
  add_rostest(test/steer_drive_controller_no_steer_test/steer_drive_controller_no_steer.test)
  add_rostest(test/steer_drive_controller_radius_param_test/steer_drive_controller_radius_param.test)
  add_rostest(test/steer_drive_controller_radius_param_fail_test/steer_drive_controller_radius_param_fail.test)
  add_rostest(test/steer_drive_controller_separation_param_test/steer_drive_controller_separation_param.test)
endif()
