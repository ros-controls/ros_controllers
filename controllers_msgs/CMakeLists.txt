# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(controllers_msgs)

if(USE_ROSBUILD)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  # Generate Actions
  rosbuild_find_ros_package(actionlib_msgs)
  include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
  genaction()

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_genmsg()
  rosbuild_gensrv()
else()
  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS actionlib_msgs trajectory_msgs geometry_msgs message_generation)

  add_message_files(
    FILES  JointControllerState.msg
    )
  add_service_files(
    FILES QueryCalibrationState.srv  QueryTrajectoryState.srv
    )
  add_action_files(
    FILES JointTrajectory.action  SingleJointPosition.action
    )

  ## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES actionlib_msgs geometry_msgs trajectory_msgs std_msgs
    )

  # Declare catkin package
  catkin_package(
    CATKIN_DEPENDS actionlib_msgs trajectory_msgs geometry_msgs message_runtime
    )
endif()
