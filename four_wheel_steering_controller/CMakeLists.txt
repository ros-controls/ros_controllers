cmake_minimum_required(VERSION 3.0.2)
project(four_wheel_steering_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  controller_interface
  four_wheel_steering_msgs
  hardware_interface
  nav_msgs
  pluginlib
  realtime_tools
  roscpp
  tf
  urdf_geometry_parser
)

find_package(Boost REQUIRED)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    four_wheel_steering_msgs
    hardware_interface
    nav_msgs
    realtime_tools
    roscpp
    tf
  DEPENDS Boost
)


###########
## Build ##
###########

# Specify header include paths
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/four_wheel_steering_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(controller_manager REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(rostest REQUIRED)
  
  include_directories(
    ${controller_manager_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${std_srvs_INCLUDE_DIRS}
    ${rosgraph_msgs_INCLUDE_DIRS}
  )

  add_executable(four_wheel_steering test/src/four_wheel_steering.cpp)
  target_link_libraries(four_wheel_steering ${catkin_LIBRARIES} ${controller_manager_LIBRARIES})

  add_dependencies(tests four_wheel_steering)

  add_rostest_gtest(four_wheel_steering_controller_twist_cmd_test
                    test/four_wheel_steering_controller_twist_cmd.test
                    test/src/four_wheel_steering_twist_cmd_test.cpp
  )
  target_link_libraries(four_wheel_steering_controller_twist_cmd_test ${catkin_LIBRARIES})

  add_rostest_gtest(four_wheel_steering_controller_4ws_cmd_test
                    test/four_wheel_steering_controller_4ws_cmd.test
                    test/src/four_wheel_steering_4ws_cmd_test.cpp
  )
  target_link_libraries(four_wheel_steering_controller_4ws_cmd_test ${catkin_LIBRARIES})

  add_rostest_gtest(four_wheel_steering_wrong_config_test
                    test/four_wheel_steering_wrong_config.test
                    test/src/four_wheel_steering_wrong_config.cpp
  )
  target_link_libraries(four_wheel_steering_wrong_config_test ${catkin_LIBRARIES})

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
install(FILES four_wheel_steering_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
