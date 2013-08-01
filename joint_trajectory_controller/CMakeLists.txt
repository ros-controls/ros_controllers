cmake_minimum_required(VERSION 2.8.3)
project(joint_trajectory_controller)

if(USE_ROSBUILD)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_add_library(${PROJECT_NAME}
    src/joint_trajectory_controller.cpp include/joint_trajectory_controller/joint_trajectory_controller.h
    include/joint_trajectory_controller/trajectory_interface_ros.h
    include/joint_trajectory_controller/joint_trajectory_segment.h
    include/trajectory_interface/trajectory_interface.h
    include/trajectory_interface/quintic_spline_segment.h)

  rosbuild_add_gtest(quintic_spline_segment_test   test/quintic_spline_segment_test.cpp)
  rosbuild_add_gtest(multi_dof_segment_test        test/multi_dof_segment_test.cpp)
  rosbuild_add_gtest(trajectory_interface_test     test/trajectory_interface_test.cpp)
  rosbuild_add_gtest(joint_trajectory_segment_test test/joint_trajectory_segment_test.cpp)
  rosbuild_add_gtest(trajectory_interface_ros_test test/trajectory_interface_ros_test.cpp)
  rosbuild_add_gtest(init_joint_trajectory_test    test/init_joint_trajectory_test.cpp)

  # TODO: why is it explicitly needed???, without it the linker fails.
  target_link_libraries(quintic_spline_segment_test   pthread)
  target_link_libraries(multi_dof_segment_test        pthread)
  target_link_libraries(trajectory_interface_test     pthread)
  target_link_libraries(joint_trajectory_segment_test pthread)
  target_link_libraries(trajectory_interface_ros_test pthread)
  target_link_libraries(init_joint_trajectory_test    pthread)
else()
  # TODO

  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS controller_interface controllers_msgs forward_command_controller control_toolbox realtime_tools urdf)

  include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

  add_library(${PROJECT_NAME}
    src/joint_effort_controller.cpp include/effort_controllers/joint_effort_controller.h
    src/joint_velocity_controller.cpp include/effort_controllers/joint_velocity_controller.h
    src/joint_position_controller.cpp include/effort_controllers/joint_position_controller.h)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

  # Declare catkin package
  catkin_package(
    CATKIN_DEPENDS controller_interface controllers_msgs control_toolbox realtime_tools urdf forward_command_controller
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()
