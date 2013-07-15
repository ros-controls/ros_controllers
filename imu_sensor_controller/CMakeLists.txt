cmake_minimum_required(VERSION 2.8.3)
project(imu_sensor_controller)

if(USE_ROSBUILD)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_add_library(${PROJECT_NAME} src/imu_sensor_controller.cpp include/imu_sensor_controller/imu_sensor_controller.h)

else()
  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS controller_interface hardware_interface sensor_msgs realtime_tools)

  include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

  add_library(${PROJECT_NAME}
    src/imu_sensor_controller.cpp include/imu_sensor_controller/imu_sensor_controller.h)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

  # Declare catkin package
  catkin_package(
    CATKIN_DEPENDS controller_interface hardware_interface sensor_msgs realtime_tools
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()
