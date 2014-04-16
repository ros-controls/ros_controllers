cmake_minimum_required(VERSION 2.8.3)
project(position_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS controller_interface forward_command_controller)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
  src/joint_position_controller.cpp include/position_controllers/joint_position_controller.h)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Declare catkin project
catkin_package(
    CATKIN_DEPENDS controller_interface forward_command_controller
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

install(FILES position_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
