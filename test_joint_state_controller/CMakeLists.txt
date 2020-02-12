cmake_minimum_required(VERSION 2.8.3)
project(test_joint_state_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  joint_state_controller
  sensor_msgs
  roscpp
)

# Declare a catkin package
catkin_package()


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Specify header include paths
  include_directories(${catkin_INCLUDE_DIRS})

  add_rostest_gtest(joint_state_controller_test
                    test/joint_state_controller.test
                    test/joint_state_controller_test.cpp)
  target_link_libraries(joint_state_controller_test ${catkin_LIBRARIES})
endif()
