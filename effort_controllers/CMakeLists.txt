cmake_minimum_required(VERSION 3.0.2)
project(effort_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  angles
  control_msgs
  control_toolbox
  controller_interface
  forward_command_controller
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  std_msgs
  urdf
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    control_msgs
    control_toolbox
    controller_interface
    forward_command_controller
    hardware_interface
    realtime_tools
    std_msgs
    urdf
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/joint_effort_controller.cpp
  src/joint_group_effort_controller.cpp
  src/joint_group_position_controller.cpp
  src/joint_position_controller.cpp
  src/joint_velocity_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)

  include_directories(
    ${controller_manager_INCLUDE_DIRS}
    ${rosgraph_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
  )

  add_executable(simple_bot_hw test/simple_bot_hw.cpp)
  target_link_libraries(simple_bot_hw
    ${catkin_LIBRARIES}
    ${controller_manager_LIBRARIES}
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


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(FILES effort_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
