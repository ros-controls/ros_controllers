cmake_minimum_required(VERSION 3.0.2)
project(diff_drive_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    controller_interface
    control_msgs
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    nav_msgs
    pluginlib
    realtime_tools
    tf
    urdf
)

find_package(Boost REQUIRED COMPONENTS headers)

# Declare a catkin package
generate_dynamic_reconfigure_options(cfg/DiffDriveController.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    nav_msgs
    realtime_tools
    tf
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)


###########
## Build ##
###########

# Specify header include paths
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/diff_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(controller_manager REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(rostest REQUIRED)
  
  include_directories(
    ${controller_manager_INCLUDE_DIRS}
    ${rosgraph_msgs_INCLUDE_DIRS}
    ${std_srvs_INCLUDE_DIRS}
  )

  set(test_LIBRARIES
    ${controller_manager_LIBRARIES}
    ${rosgraph_msgs_LIBRARIES}
    ${std_srvs_LIBRARIES}
  )

  add_executable(diffbot test/diffbot.cpp)
  target_link_libraries(diffbot ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_executable(skidsteerbot test/skidsteerbot.cpp)
  target_link_libraries(skidsteerbot ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_dependencies(tests diffbot skidsteerbot)

  add_rostest_gtest(diff_drive_test
    test/diff_drive_controller.test
    test/diff_drive_test.cpp
  )
  target_link_libraries(diff_drive_test ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_rostest_gtest(diff_drive_nan_test
    test/diff_drive_controller_nan.test
    test/diff_drive_nan_test.cpp
  )
  target_link_libraries(diff_drive_nan_test ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_rostest_gtest(diff_drive_limits_test
    test/diff_drive_controller_limits.test
    test/diff_drive_limits_test.cpp
  )
  target_link_libraries(diff_drive_limits_test ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_rostest_gtest(diff_drive_timeout_test
    test/diff_drive_timeout.test
    test/diff_drive_timeout_test.cpp
  )
  target_link_libraries(diff_drive_timeout_test ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_rostest(test/diff_drive_multipliers.test)

  add_rostest(test/diff_drive_left_right_multipliers.test)

  add_rostest_gtest(diff_drive_fail_test
    test/diff_drive_wrong.test
    test/diff_drive_fail_test.cpp
  )
  target_link_libraries(diff_drive_fail_test ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_rostest_gtest(diff_drive_odom_tf_test
    test/diff_drive_odom_tf.test
    test/diff_drive_odom_tf_test.cpp
  )
  target_link_libraries(diff_drive_odom_tf_test ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_rostest_gtest(diff_drive_default_cmd_vel_out_test
    test/diff_drive_default_cmd_vel_out.test
    test/diff_drive_default_cmd_vel_out_test.cpp
  )
  target_link_libraries(diff_drive_default_cmd_vel_out_test ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_rostest_gtest(diff_drive_pub_cmd_vel_out_test
    test/diff_drive_pub_cmd_vel_out.test
    test/diff_drive_pub_cmd_vel_out_test.cpp
  )
  target_link_libraries(diff_drive_pub_cmd_vel_out_test ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_rostest_gtest(diff_drive_default_wheel_joint_controller_state_test
    test/diff_drive_default_wheel_joint_controller_state.test
    test/diff_drive_default_wheel_joint_controller_state_test.cpp
  )
  target_link_libraries(diff_drive_default_wheel_joint_controller_state_test ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_rostest_gtest(diff_drive_publish_wheel_joint_controller_state_test
    test/diff_drive_publish_wheel_joint_controller_state.test
    test/diff_drive_publish_wheel_joint_controller_state_test.cpp
  )
  target_link_libraries(diff_drive_publish_wheel_joint_controller_state_test ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_rostest_gtest(diff_drive_default_odom_frame_test
    test/diff_drive_default_odom_frame.test
    test/diff_drive_default_odom_frame_test.cpp
  )
  target_link_libraries(diff_drive_default_odom_frame_test ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_rostest_gtest(diff_drive_odom_frame_test
    test/diff_drive_odom_frame.test
    test/diff_drive_odom_frame_test.cpp
  )
  target_link_libraries(diff_drive_odom_frame_test ${catkin_LIBRARIES} ${test_LIBRARIES})

  add_rostest_gtest(diff_drive_multiple_cmd_vel_publishers_test
    test/diff_drive_multiple_cmd_vel_publishers.test
    test/diff_drive_multiple_cmd_vel_publishers_test.cpp
  )
  target_link_libraries(diff_drive_multiple_cmd_vel_publishers_test ${catkin_LIBRARIES} ${test_LIBRARIES})

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
    test/diff_drive_dyn_reconf_test.cpp
  )
  target_link_libraries(diff_drive_dyn_reconf_test ${catkin_LIBRARIES} ${test_LIBRARIES})

endif()


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(FILES diff_drive_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
