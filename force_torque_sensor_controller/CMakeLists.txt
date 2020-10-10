cmake_minimum_required(VERSION 3.0.2)
project(force_torque_sensor_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    controller_interface
    geometry_msgs
    hardware_interface
    realtime_tools
    roscpp
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/force_torque_sensor_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


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

# Intall plugins
install(FILES force_torque_sensor_plugin.xml
              config/force_torque_sensor_controller.yaml
              launch/force_torque_sensor_controller.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
