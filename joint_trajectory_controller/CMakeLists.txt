cmake_minimum_required(VERSION 3.0.2)
project(joint_trajectory_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin
  REQUIRED COMPONENTS
    actionlib
    angles
    cmake_modules
    roscpp
    urdf
    control_toolbox
    controller_interface
    hardware_interface
    realtime_tools
    control_msgs
    trajectory_msgs
    pluginlib
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
  actionlib
  angles
  roscpp
  urdf
  control_toolbox
  controller_interface
  hardware_interface
  realtime_tools
  control_msgs
  trajectory_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

add_library(${PROJECT_NAME} src/joint_trajectory_controller.cpp
                            include/joint_trajectory_controller/hardware_interface_adapter.h
                            include/joint_trajectory_controller/init_joint_trajectory.h
                            include/joint_trajectory_controller/joint_trajectory_controller.h
                            include/joint_trajectory_controller/joint_trajectory_controller_impl.h
                            include/joint_trajectory_controller/joint_trajectory_msg_utils.h
                            include/joint_trajectory_controller/joint_trajectory_segment.h
                            include/joint_trajectory_controller/tolerances.h
                            include/trajectory_interface/trajectory_interface.h
                            include/trajectory_interface/quintic_spline_segment.h
                            include/trajectory_interface/pos_vel_acc_state.h)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin
    REQUIRED COMPONENTS
      actionlib
      controller_manager
      xacro
  )

  find_package(rostest REQUIRED)
  include_directories(include ${catkin_INCLUDE_DIRS})

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

  catkin_add_gtest(trajectory_builder_test test/trajectory_builder_test.cpp)
  target_link_libraries(trajectory_builder_test ${catkin_LIBRARIES})

  catkin_add_gtest(hold_trajectory_builder_test
                   test/hold_trajectory_builder_test.cpp
                   test/test_common.cpp)
  target_link_libraries(hold_trajectory_builder_test ${catkin_LIBRARIES})

  catkin_add_gtest(stop_trajectory_builder_test
                   test/stop_trajectory_builder_test.cpp
                   test/test_common.cpp)
  target_link_libraries(stop_trajectory_builder_test ${catkin_LIBRARIES})

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
  add_dependencies(tests ${PROJECT_NAME})

  add_rostest_gtest(joint_trajectory_controller_test
                    test/joint_trajectory_controller.test
                    test/joint_trajectory_controller_test.cpp
                    test/test_common.cpp)
  target_link_libraries(joint_trajectory_controller_test ${catkin_LIBRARIES})

  add_rostest_gtest(joint_trajectory_controller_stopramp_test
                    test/joint_trajectory_controller_stopramp.test
                    test/joint_trajectory_controller_test.cpp
                    test/test_common.cpp)
  target_link_libraries(joint_trajectory_controller_stopramp_test ${catkin_LIBRARIES})

  add_rostest_gtest(joint_trajectory_controller_vel_test
                    test/joint_trajectory_controller_vel.test
                    test/joint_trajectory_controller_test.cpp
                    test/test_common.cpp)
  target_link_libraries(joint_trajectory_controller_vel_test ${catkin_LIBRARIES})
  target_compile_definitions(joint_trajectory_controller_vel_test PRIVATE TEST_VELOCITY_FF=1)

  add_rostest_gtest(joint_trajectory_controller_wrapping_test
                    test/joint_trajectory_controller_wrapping.test
                    test/joint_trajectory_controller_wrapping_test.cpp)
  target_link_libraries(joint_trajectory_controller_wrapping_test ${catkin_LIBRARIES})

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(NAME ${PROJECT_NAME}_coverage)
  endif()

endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/trajectory_interface/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/trajectory_interface/)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(FILES ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# TODO: Install test resource files as well?
