// NOTE: The contents of this file have been taken largely from the ros_control wiki tutorials

#include "four_wheel_steering.h"
#include <chrono>
#include <thread>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "four_wheel_steering");
  ros::NodeHandle nh;

  // This should be set in launch files as well
  nh.setParam("/use_sim_time", true);

  FourWheelSteering robot;
  ROS_WARN_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Publisher clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();
  std::chrono::system_clock::time_point end   = std::chrono::system_clock::now();

  ros::Time internal_time(0);
  const ros::Duration dt = robot.getPeriod();
  double elapsed_secs = 0;

  while(ros::ok())
  {
    begin = std::chrono::system_clock::now();

    robot.read();
    cm.update(internal_time, dt);
    robot.write();

    end = std::chrono::system_clock::now();

    elapsed_secs = std::chrono::duration_cast<std::chrono::duration<double> >((end - begin)).count();

    if (dt.toSec() - elapsed_secs < 0.0)
    {
      ROS_WARN_STREAM_THROTTLE(
            0.1, "Control cycle is taking to much time, elapsed: " << elapsed_secs);
    }
    else
    {
      ROS_DEBUG_STREAM_THROTTLE(1.0, "Control cycle is, elapsed: " << elapsed_secs);
      std::this_thread::sleep_for(std::chrono::duration<double>(dt.toSec() - elapsed_secs));
    }

    rosgraph_msgs::Clock clock;
    clock.clock = ros::Time(internal_time);
    clock_publisher.publish(clock);
    internal_time += dt;
  }
  spinner.stop();

  return 0;
}
