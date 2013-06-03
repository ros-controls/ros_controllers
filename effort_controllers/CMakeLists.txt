# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(effort_controllers)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS controller_interface controllers_msgs forward_command_controller control_toolbox realtime_tools urdf)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/joint_effort_controller.cpp include/effort_controllers/joint_effort_controller.h
  src/joint_velocity_controller.cpp include/effort_controllers/joint_velocity_controller.h
  src/joint_position_controller.cpp include/effort_controllers/joint_position_controller.h)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS controller_interface controllers_msgs control_toolbox realtime_tools urdf forward_command_controller
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)
