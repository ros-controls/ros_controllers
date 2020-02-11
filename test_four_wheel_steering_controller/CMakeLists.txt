cmake_minimum_required(VERSION 2.8.3)
project(test_four_wheel_steering_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  four_wheel_steering_msgs
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
  include_directories(${catkin_INCLUDE_DIRS})

  add_executable(four_wheel_steering test/src/four_wheel_steering.cpp)
  target_link_libraries(four_wheel_steering ${catkin_LIBRARIES})

  add_dependencies(tests four_wheel_steering)

  add_rostest_gtest(four_wheel_steering_controller_twist_cmd_test
                    test/four_wheel_steering_controller_twist_cmd.test
                    test/src/four_wheel_steering_twist_cmd_test.cpp)
  target_link_libraries(four_wheel_steering_controller_twist_cmd_test ${catkin_LIBRARIES})

  add_rostest_gtest(four_wheel_steering_controller_4ws_cmd_test
                    test/four_wheel_steering_controller_4ws_cmd.test
                    test/src/four_wheel_steering_4ws_cmd_test.cpp)
  target_link_libraries(four_wheel_steering_controller_4ws_cmd_test ${catkin_LIBRARIES})

  add_rostest_gtest(four_wheel_steering_wrong_config_test
                    test/four_wheel_steering_wrong_config.test
                    test/src/four_wheel_steering_wrong_config.cpp)
  target_link_libraries(four_wheel_steering_wrong_config_test ${catkin_LIBRARIES})
endif()
