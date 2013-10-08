cmake_minimum_required(VERSION 2.8.3)
project(diff_drive_controller)

find_package(catkin REQUIRED COMPONENTS
  rostest
  controller_manager
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES diff_drive_controller
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/diff_drive_controller.cpp)

add_executable(diffbot test/diffbot.cpp)
target_link_libraries(diffbot ${catkin_LIBRARIES})

catkin_add_gtest(diff_drive_test test/diff_drive_test.cpp)
add_rostest(test/diff_drive_controller.test)
