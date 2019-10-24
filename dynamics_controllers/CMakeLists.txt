cmake_minimum_required(VERSION 2.8.3)
project(dynamics_controllers)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# For now, latest build from source is needed
find_package(orocos_kdl REQUIRED)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  kdl_parser
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
    orocos_kdl
  CATKIN_DEPENDS
    controller_interface
    kdl_parser
)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/dynamics_controller_base.cpp
  src/kdl_chain_controller.cpp
  src/kdl_tree_controller.cpp
  src/kdl/treeidsolver_recursive_newton_euler.cpp # NOTE: this will be removed
)

# the options are used to force the linker to report undefined symbols as errors
target_link_libraries(${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} -Wl,--no-undefined)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  FILES dynamics_controllers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)