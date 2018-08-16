#include "test_common.h"
#include <tf/transform_listener.h>

// TEST CASES
TEST_F(FourWheelSteeringControllerTest, testForward)
{
  // wait for ROS
  waitForController();

  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  ros::Duration(0.1).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a velocity command of 0.1 m/s
  cmd_vel.linear.x = 0.1;
  publish(cmd_vel);
  // wait for Xs
  double travel_time = 5.0;
  ros::Duration(travel_time).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  const ros::Duration actual_elapsed_time = new_odom.header.stamp - old_odom.header.stamp;

  const double expected_distance = cmd_vel.linear.x * actual_elapsed_time.toSec();

  // check if the robot traveled 1 meter in XY plane, changes in z should be ~~0
  const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
  const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
  const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
  EXPECT_NEAR(sqrt(dx*dx + dy*dy), expected_distance, POSITION_TOLERANCE);
  EXPECT_LT(fabs(dz), EPS);

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_LT(fabs(roll_new - roll_old), EPS);
  EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
  EXPECT_LT(fabs(yaw_new - yaw_old), EPS);
  EXPECT_NEAR(fabs(new_odom.twist.twist.linear.x), cmd_vel.linear.x, EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

  EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z), EPS);
}

TEST_F(FourWheelSteeringControllerTest, testTurn)
{
  // wait for ROS
  waitForController();

  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  ros::Duration(0.1).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a velocity command
  cmd_vel.linear.x = M_PI/2.0;
  cmd_vel.angular.z = M_PI/10.0;
  publish(cmd_vel);
  // wait for 10s to make a half turn
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot traveled 20 meter in XY plane, changes in z should be ~~0
  const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
  const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
  const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
  EXPECT_NEAR(sqrt(dx*dx + dy*dy), 2*cmd_vel.linear.x/cmd_vel.angular.z, POSITION_TOLERANCE);
  EXPECT_LT(fabs(dz), EPS);

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_LT(fabs(roll_new - roll_old), EPS);
  EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
  EXPECT_NEAR(fabs(yaw_new - yaw_old), M_PI, ORIENTATION_TOLERANCE);

  EXPECT_NEAR(fabs(new_odom.twist.twist.linear.x), cmd_vel.linear.x, EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

  EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
  EXPECT_NEAR(fabs(new_odom.twist.twist.angular.z), cmd_vel.angular.z, EPS);
}

TEST_F(FourWheelSteeringControllerTest, testOdomFrame)
{
  // wait for ROS
  waitForController();

  // set up tf listener
  tf::TransformListener listener;
  ros::Duration(2.0).sleep();
  // check the odom frame exist
  EXPECT_TRUE(listener.frameExists("odom"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "four_wheel_steering_twist_cmd_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
