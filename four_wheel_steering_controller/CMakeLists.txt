cmake_minimum_required(VERSION 2.8.3)
project(four_wheel_steering_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  four_wheel_steering_msgs
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  realtime_tools
  roscpp
  tf
  urdf_geometry_parser
)

find_package(Boost REQUIRED COMPONENTS system thread)

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

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/four_wheel_steering_controller.cpp src/odometry.cpp src/speed_limiter.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES four_wheel_steering_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS
    controller_interface
    controller_manager
    four_wheel_steering_msgs
    gazebo_ros
    geometry_msgs
    hardware_interface
    nav_msgs
    pluginlib
    realtime_tools
    roscpp
    rosgraph_msgs
    rostest
    rosunit
    std_msgs
    std_srvs
    tf
    urdf_geometry_parser
    xacro
  )

  add_executable(four_wheel_steering test/src/four_wheel_steering.cpp)
  target_link_libraries(four_wheel_steering ${catkin_LIBRARIES})

  add_dependencies(tests four_wheel_steering)

  add_rostest_gtest(four_wheel_steering_controller_twist_cmd_test
                    test/four_wheel_steering_controller_twist_cmd.test
                    test/src/four_wheel_steering_twist_cmd_test.cpp)
  target_link_libraries(four_wheel_steering_controller_twist_cmd_test ${catkin_LIBRARIES})

  add_rostest_gtest(four_wheel_steering_controller_4ws_cmd_test
                    test/four_wheel_steering_controller_4ws_cmd.test
                    test/src/four_wheel_steering_4ws_cmd_test.cpp)
  target_link_libraries(four_wheel_steering_controller_4ws_cmd_test ${catkin_LIBRARIES})

  add_rostest_gtest(four_wheel_steering_wrong_config_test
                    test/four_wheel_steering_wrong_config.test
                    test/src/four_wheel_steering_wrong_config.cpp)
  target_link_libraries(four_wheel_steering_wrong_config_test ${catkin_LIBRARIES})

endif()
