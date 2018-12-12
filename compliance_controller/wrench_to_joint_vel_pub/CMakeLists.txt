cmake_minimum_required(VERSION 2.8.3)
project(wrench_to_joint_vel_pub)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  compliance_control_msgs
  geometry_msgs
  moveit_ros_planning_interface
  roscpp
  rosparam_shortcuts
  rospy
  sensor_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES compliant_control
  CATKIN_DEPENDS
    compliance_control_msgs
    geometry_msgs
    moveit_ros_planning_interface
    roscpp
    rosparam_shortcuts
    rospy
    sensor_msgs
    std_srvs
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(compliant_control
  src/${PROJECT_NAME}/compliant_control.cpp
  src/${PROJECT_NAME}/low_pass_filter.cpp
)

add_executable(wrench_to_joint_vel_pub src/${PROJECT_NAME}/wrench_to_joint_vel_pub.cpp)
add_dependencies(wrench_to_joint_vel_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wrench_to_joint_vel_pub ${catkin_LIBRARIES} compliant_control ${Eigen_LIBRARIES})

add_executable(compliance_publisher_demo src/${PROJECT_NAME}/compliance_publisher_demo.cpp)
add_dependencies(compliance_publisher_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(compliance_publisher_demo ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS compliant_control wrench_to_joint_vel_pub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
