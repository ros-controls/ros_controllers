cmake_minimum_required(VERSION 3.0.2)
project(joint_state_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS realtime_tools roscpp hardware_interface pluginlib controller_interface sensor_msgs)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS realtime_tools roscpp hardware_interface pluginlib controller_interface sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/joint_state_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(joint_state_controller_test
                    test/joint_state_controller.test
                    test/joint_state_controller_test.cpp)
  target_link_libraries(joint_state_controller_test ${PROJECT_NAME})
endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(FILES joint_state_plugin.xml
              joint_state_controller.yaml
              joint_state_controller.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
