cmake_minimum_required(VERSION 3.0.2)
project(forward_command_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  realtime_tools
  roscpp
  std_msgs
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    realtime_tools
    roscpp
    std_msgs
)


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
