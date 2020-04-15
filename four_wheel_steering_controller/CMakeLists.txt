cmake_minimum_required(VERSION 3.0.2)
project(four_wheel_steering_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(${PROJECT_NAME}_CATKIN_DEPS
    controller_interface
    nav_msgs
    four_wheel_steering_msgs
    realtime_tools
    tf
    urdf_geometry_parser
    pluginlib)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/four_wheel_steering_controller.cpp src/odometry.cpp src/speed_limiter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(FILES four_wheel_steering_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rosgraph_msgs rostest std_srvs controller_manager tf)
  include_directories(${catkin_INCLUDE_DIRS})

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
