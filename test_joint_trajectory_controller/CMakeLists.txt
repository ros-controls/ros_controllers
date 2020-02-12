cmake_minimum_required(VERSION 2.8.3)
project(test_joint_trajectory_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  controller_manager
  controller_manager_msgs
  hardware_interface
  joint_trajectory_controller
  realtime_tools
  roscpp
  std_msgs
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

  catkin_add_gtest(quintic_spline_segment_test test/quintic_spline_segment_test.cpp)
  target_link_libraries(quintic_spline_segment_test ${catkin_LIBRARIES})

  catkin_add_gtest(trajectory_interface_test test/trajectory_interface_test.cpp)
  target_link_libraries(trajectory_interface_test ${catkin_LIBRARIES})

  catkin_add_gtest(joint_trajectory_segment_test test/joint_trajectory_segment_test.cpp)
  target_link_libraries(joint_trajectory_segment_test ${catkin_LIBRARIES})

  catkin_add_gtest(joint_trajectory_msg_utils_test test/joint_trajectory_msg_utils_test.cpp)
  target_link_libraries(joint_trajectory_msg_utils_test ${catkin_LIBRARIES})

  catkin_add_gtest(init_joint_trajectory_test test/init_joint_trajectory_test.cpp)
  target_link_libraries(init_joint_trajectory_test ${catkin_LIBRARIES})

  add_rostest_gtest(tolerances_test
                  test/tolerances.test
                  test/tolerances_test.cpp)
  target_link_libraries(tolerances_test ${catkin_LIBRARIES})

  add_executable(rrbot test/rrbot.cpp)
  target_link_libraries(rrbot ${catkin_LIBRARIES})

  add_executable(rrbot_wrapping test/rrbot_wrapping.cpp)
  target_link_libraries(rrbot_wrapping ${catkin_LIBRARIES})

  add_dependencies(tests rrbot)
  add_dependencies(tests rrbot_wrapping)

  add_rostest_gtest(joint_trajectory_controller_test
                    test/joint_trajectory_controller.test
                    test/joint_trajectory_controller_test.cpp)
  target_link_libraries(joint_trajectory_controller_test ${catkin_LIBRARIES})

  add_rostest_gtest(joint_trajectory_controller_stopramp_test
                    test/joint_trajectory_controller_stopramp.test
                    test/joint_trajectory_controller_test.cpp)
  target_link_libraries(joint_trajectory_controller_stopramp_test ${catkin_LIBRARIES})

  add_rostest_gtest(joint_trajectory_controller_vel_test
                    test/joint_trajectory_controller_vel.test
                    test/joint_trajectory_controller_test.cpp)
  target_link_libraries(joint_trajectory_controller_vel_test ${catkin_LIBRARIES})
  target_compile_definitions(joint_trajectory_controller_vel_test PRIVATE TEST_VELOCITY_FF=1)

  add_rostest_gtest(joint_trajectory_controller_wrapping_test
                    test/joint_trajectory_controller_wrapping.test
                    test/joint_trajectory_controller_wrapping_test.cpp)
  target_link_libraries(joint_trajectory_controller_wrapping_test ${catkin_LIBRARIES})
endif()
