cmake_minimum_required(VERSION 2.8.3)
project(swerve_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Package dependencies
set(${PROJECT_NAME}_CATKIN_DEPS
    controller_interface
    joint_state_controller
    nav_msgs
    realtime_tools
    robot_state_publisher
    roslint
    realtime_tools
    tf
    urdf_geometry_parser)
list(SORT ${PROJECT_NAME}_CATKIN_DEPS)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

# Create the package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

# Source and headers for build
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/odometry.cpp src/speed_limiter.cpp src/swerve_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Paths for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES swerve_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Tests for CI
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(controller_manager REQUIRED)

  include_directories(test/include)
  include_directories(${catkin_INCLUDE_DIRS})

  add_rostest_gtest(swerve_controller_test
                    test/swervebot.test
                    test/src/swervebot.cpp)

  target_link_libraries(swerve_controller_test ${catkin_LIBRARIES})
endif()

# Syntax linter for CI
roslint_cpp()