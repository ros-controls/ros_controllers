cmake_minimum_required(VERSION 3.0.2)
project(velocity_controllers)

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
  realtime_tools
  urdf
  pluginlib
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    angles
    control_msgs
    control_toolbox
    controller_interface
    forward_command_controller
    realtime_tools
    urdf
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/joint_velocity_controller.cpp
  src/joint_position_controller.cpp
  src/joint_group_velocity_controller.cpp
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

install(FILES velocity_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
