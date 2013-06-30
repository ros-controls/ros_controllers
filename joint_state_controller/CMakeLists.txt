cmake_minimum_required(VERSION 2.8.3)
project(joint_state_controller)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_add_library(${PROJECT_NAME} src/joint_state_controller.cpp)

else()
  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS realtime_tools roscpp hardware_interface pluginlib controller_interface sensor_msgs)

  include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
  add_library(${PROJECT_NAME} src/joint_state_controller.cpp)

  # Declare catkin package
  catkin_package(
    CATKIN_DEPENDS realtime_tools roscpp hardware_interface pluginlib controller_interface sensor_msgs
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

  install(FILES joint_state_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

endif()
