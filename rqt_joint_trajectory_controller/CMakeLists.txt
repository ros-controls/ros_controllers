cmake_minimum_required(VERSION 2.8.3)
project(rqt_joint_trajectory_controller)

find_package(catkin REQUIRED COMPONENTS)
catkin_python_setup()
catkin_package()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/rqt_joint_trajectory_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
