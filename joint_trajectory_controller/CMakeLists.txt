cmake_minimum_required(VERSION 2.8.3)
project(joint_trajectory_controller)

if(USE_ROSBUILD)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_add_library(${PROJECT_NAME}
    src/joint_trajectory_controller.cpp
    include/joint_trajectory_controller/joint_trajectory_controller.h
    include/joint_trajectory_controller/joint_trajectory_controller_impl.h
    include/joint_trajectory_controller/joint_trajectory_msg_utils.h
    include/joint_trajectory_controller/joint_trajectory_segment.h
    include/trajectory_interface/trajectory_interface.h
    include/trajectory_interface/quintic_spline_segment.h
    include/trajectory_interface/pos_vel_acc_state.h)

  rosbuild_add_gtest(quintic_spline_segment_test     test/quintic_spline_segment_test.cpp)
  rosbuild_add_gtest(trajectory_interface_test       test/trajectory_interface_test.cpp)
  rosbuild_add_gtest(joint_trajectory_segment_test   test/joint_trajectory_segment_test.cpp)
  rosbuild_add_gtest(joint_trajectory_msg_utils_test test/joint_trajectory_msg_utils_test.cpp)
  rosbuild_add_gtest(init_joint_trajectory_test      test/init_joint_trajectory_test.cpp)

  rosbuild_add_executable(tolerances_test test/tolerances_test.cpp)
  rosbuild_add_gtest_build_flags(tolerances_test)
  rosbuild_add_rostest(test/tolerances.test)

  rosbuild_add_executable(rrbot test/rrbot.cpp)

  rosbuild_add_executable(joint_trajectory_controller_test test/joint_trajectory_controller_test.cpp)
  rosbuild_add_gtest_build_flags(joint_trajectory_controller_test)
  rosbuild_add_rostest(test/joint_trajectory_controller.test)

  # TODO: why is it explicitly needed???, without it the linker fails.
  target_link_libraries(quintic_spline_segment_test     pthread)
  target_link_libraries(trajectory_interface_test       pthread)
  target_link_libraries(joint_trajectory_segment_test   pthread)
  target_link_libraries(joint_trajectory_msg_utils_test pthread)
  target_link_libraries(init_joint_trajectory_test      pthread)
  target_link_libraries(tolerances_test                 pthread)
  target_link_libraries(joint_trajectory_controller     pthread)
else()
  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS actionlib
                                          angles
                                          roscpp
                                          urdf
                                          control_toolbox
                                          controller_interface
                                          hardware_interface
                                          realtime_tools
                                          control_msgs
                                          trajectory_msgs
                                          rostest
                                          controller_manager
                                          xacro)

  include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

  add_library(${PROJECT_NAME} src/joint_trajectory_controller.cpp
                              include/joint_trajectory_controller/joint_trajectory_controller.h
                              include/joint_trajectory_controller/joint_trajectory_controller_impl.h
                              include/joint_trajectory_controller/joint_trajectory_msg_utils.h
                              include/joint_trajectory_controller/joint_trajectory_segment.h
                              include/trajectory_interface/trajectory_interface.h
                              include/trajectory_interface/quintic_spline_segment.h
                              include/trajectory_interface/pos_vel_acc_state.h)

  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

  # Declare catkin package
  catkin_package(
    CATKIN_DEPENDS actionlib
                   angles
                   roscpp
                   urdf
                   control_toolbox
                   controller_interface
                   hardware_interface
                   realtime_tools
                   control_msgs
                   trajectory_msgs
                   rostest
                   controller_manager
                   xacro

    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )

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

  catkin_add_gtest(tolerances_test test/tolerances_test.cpp)
  target_link_libraries(tolerances_test ${catkin_LIBRARIES})
  add_rostest(test/tolerances.test)

  add_executable(rrbot test/rrbot.cpp)
  add_dependencies(tests rrbot)
  catkin_add_gtest(joint_trajectory_controller_test test/joint_trajectory_controller_test.cpp)
  target_link_libraries(joint_trajectory_controller_test ${catkin_LIBRARIES})
  add_rostest(test/joint_trajectory_controller.test)

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  # TODO: Install test resource files as well?
endif()
