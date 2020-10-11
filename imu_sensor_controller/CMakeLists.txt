cmake_minimum_required(VERSION 3.0.2)
project(imu_sensor_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    realtime_tools
    roscpp
    sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/imu_sensor_controller.cpp
  include/imu_sensor_controller/imu_sensor_controller.h
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

# Install plugins
install(FILES imu_sensor_plugin.xml
              config/imu_sensor_controller.yaml
              launch/imu_sensor_controller.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
