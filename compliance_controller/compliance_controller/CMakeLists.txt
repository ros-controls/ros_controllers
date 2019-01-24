cmake_minimum_required(VERSION 2.8.3)
project(compliance_controller)

add_compile_options(-std=c++11)

find_package(catkin
  REQUIRED COMPONENTS
    compliance_control_msgs
    message_generation
    realtime_tools
    roscpp
    std_srvs
    traj_or_jog_controller
)

find_package(traj_or_jog_controller REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    compliance_control_msgs
    realtime_tools
    roscpp
    traj_or_jog_controller
  INCLUDE_DIRS include
  LIBRARIES compliance_controller traj_or_jog_controller
)

include_directories(include ${catkin_INCLUDE_DIRS} ${traj_or_jog_controller_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_library(compliance_controller src/compliance_controller.cpp)
target_link_libraries(compliance_controller ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES compliance_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})