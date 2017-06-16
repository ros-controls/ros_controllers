// NOTE: The contents of this file have been taken largely from the ros_control wiki tutorials

// ROS
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>

#include "four_wheel_steering.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "four_wheel_steering");
  ros::NodeHandle nh;

  FourWheelSteering robot;
  ROS_WARN_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok())
  {
    robot.read();
    cm.update(robot.getTime(), robot.getPeriod());
    robot.write();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
