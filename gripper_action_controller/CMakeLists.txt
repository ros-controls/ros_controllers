cmake_minimum_required(VERSION 2.8.3)
project(gripper_action_controller)

find_package(catkin 
  REQUIRED COMPONENTS
      actionlib
      angles
      cmake_modules
      controller_interface
      controller_manager
      control_msgs
      control_toolbox
      hardware_interface
      realtime_tools
      roscpp
      trajectory_msgs
      urdf
      xacro
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES gripper_action_controller
CATKIN_DEPENDS 
  actionlib 
  cmake_modules 
  controller_interface 
  control_msgs 
  hardware_interface 
  realtime_tools
  trajectory_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(gripper_action_controller
   src/gripper_action_controller.cpp
   include/gripper_action_controller/gripper_action_controller.h
)

target_link_libraries(gripper_action_controller
 ${catkin_LIBRARIES}
)

install(TARGETS gripper_action_controller
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES ros_control_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
