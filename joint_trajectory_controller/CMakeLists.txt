cmake_minimum_required(VERSION 2.8.3)
project(joint_trajectory_controller)

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
    controller_manager
    xacro
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
  controller_manager
  xacro
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

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
  find_package(rostest REQUIRED)

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

  add_dependencies(tests rrbot)
  add_dependencies(tests ${PROJECT_NAME})

  add_rostest_gtest(joint_trajectory_controller_test
                    test/joint_trajectory_controller.test
                    test/joint_trajectory_controller_test.cpp)
  target_link_libraries(joint_trajectory_controller_test ${catkin_LIBRARIES})
endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/trajectory_interface/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/trajectory_interface/)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# TODO: Install test resource files as well?
