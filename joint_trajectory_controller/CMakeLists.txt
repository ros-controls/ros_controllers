cmake_minimum_required(VERSION 2.8.3)
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
