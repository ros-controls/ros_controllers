cmake_minimum_required(VERSION 3.0.2)
project(rqt_joint_trajectory_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

# Declare a catkin package
catkin_package()


#############
## Install ##
#############

# Install plugins
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Intall extra
install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install scripts
catkin_install_python(PROGRAMS
  scripts/rqt_joint_trajectory_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
