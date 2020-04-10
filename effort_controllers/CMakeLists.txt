cmake_minimum_required(VERSION 3.0.2)
project(effort_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  control_msgs
  control_toolbox
  forward_command_controller
  realtime_tools
  urdf
  pluginlib
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    control_toolbox
    forward_command_controller
    realtime_tools
    urdf
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME}
  src/joint_effort_controller.cpp
  src/joint_velocity_controller.cpp
  src/joint_position_controller.cpp
  src/joint_group_effort_controller.cpp
  src/joint_group_position_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(FILES effort_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(controller_manager_msgs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  include_directories(
    ${controller_manager_INCLUDE_DIRS}
    ${hardware_interface_INCLUDE_DIRS}
    ${rosgraph_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
  )

  add_executable(simple_bot_hw test/simple_bot_hw.cpp)
  target_link_libraries(simple_bot_hw
    ${catkin_LIBRARIES}
    ${controller_manager_LIBRARIES}
    ${hardware_interface_LIBRARIES}
  )
  add_dependencies(tests simple_bot_hw)

  add_rostest_gtest(effort_position_controller_test
    test/effort_position_controller.test
    test/simple_bot_goto_position.cpp
  )
  target_link_libraries(
    effort_position_controller_test
    ${catkin_LIBRARIES}
  )
endif()
