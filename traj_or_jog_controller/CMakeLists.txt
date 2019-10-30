cmake_minimum_required(VERSION 2.8.3)
project(traj_or_jog_controller)

find_package(catkin
  REQUIRED COMPONENTS
    joint_trajectory_controller
    realtime_tools
    roscpp
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    joint_trajectory_controller
    realtime_tools
    roscpp
    std_msgs
  INCLUDE_DIRS include
  LIBRARIES joint_trajectory_controller traj_or_jog_controller
)

include_directories(include ${catkin_INCLUDE_DIRS} ${joint_trajectory_controller_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_library(${PROJECT_NAME} src/traj_or_jog_controller.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES traj_or_jog_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  