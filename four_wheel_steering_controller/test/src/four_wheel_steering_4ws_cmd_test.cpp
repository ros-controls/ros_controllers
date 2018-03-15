#include "test_common.h"
#include <tf/transform_listener.h>

// TEST CASES
TEST_F(FourWheelSteeringControllerTest, testForward)
{
  // wait for ROS
  waitForController();

  // zero everything before test
  four_wheel_steering_msgs::FourWheelSteering cmd_vel;
  cmd_vel.speed = 0.0;
  cmd_vel.front_steering_angle = 0.0;
  cmd_vel.rear_steering_angle = 0.0;
  publish_4ws(cmd_vel);
  ros::Duration(0.1).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a velocity command of 0.1 m/s
  cmd_vel.speed = 0.1;
  publish_4ws(cmd_vel);
  // wait for Xs
  double travel_time = 5.0;
  ros::Duration(travel_time).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  const double actual_travel_time = (new_odom.header.stamp - old_odom.header.stamp).toSec();
  const double expected_distance = cmd_vel.speed * actual_travel_time;

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
  EXPECT_NEAR(fabs(new_odom.twist.twist.linear.x), cmd_vel.speed, EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

  EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z), EPS);
}

TEST_F(FourWheelSteeringControllerTest, testCrab)
{
  // wait for ROS
  waitForController();

  // zero everything before test
  four_wheel_steering_msgs::FourWheelSteering cmd_vel;
  cmd_vel.speed = 0.0;
  cmd_vel.front_steering_angle = 0.0;
  cmd_vel.rear_steering_angle = 0.0;
  publish_4ws(cmd_vel);
  ros::Duration(0.1).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a velocityand steering command
  cmd_vel.speed = 0.2;
  cmd_vel.front_steering_angle = M_PI/8.0;
  cmd_vel.rear_steering_angle = cmd_vel.front_steering_angle;
  publish_4ws(cmd_vel);
  // wait for Xs
  double travel_time = 5.0;
  ros::Duration(travel_time).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  double actual_travel_time = (new_odom.header.stamp - old_odom.header.stamp).toSec();

  // check if the robot traveled 5 meter in XY plane, changes in z should be ~~0
  const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
  const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
  const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
  EXPECT_NEAR(sqrt(dx*dx + dy*dy), cmd_vel.speed*actual_travel_time, POSITION_TOLERANCE);
  EXPECT_NEAR(dx, cmd_vel.speed*actual_travel_time*cos(cmd_vel.front_steering_angle), POSITION_TOLERANCE);
  EXPECT_NEAR(dy, cmd_vel.speed*actual_travel_time*sin(cmd_vel.front_steering_angle), POSITION_TOLERANCE);
  EXPECT_LT(fabs(dz), EPS);

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_LT(fabs(roll_new - roll_old), EPS);
  EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
  EXPECT_LT(fabs(yaw_new - yaw_old), EPS);
  EXPECT_NEAR(fabs(new_odom.twist.twist.linear.x), cmd_vel.speed*cos(cmd_vel.front_steering_angle), EPS);
  EXPECT_NEAR(fabs(new_odom.twist.twist.linear.y), cmd_vel.speed*sin(cmd_vel.front_steering_angle), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

  EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z), EPS);
}

TEST_F(FourWheelSteeringControllerTest, testSymmetricTurn)
{
  // wait for ROS
  waitForController();

  // zero everything before test
  four_wheel_steering_msgs::FourWheelSteering cmd_vel;
  cmd_vel.speed = 0.0;
  cmd_vel.front_steering_angle = 0.0;
  cmd_vel.rear_steering_angle = 0.0;
  publish_4ws(cmd_vel);
  ros::Duration(0.1).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a velocity command
  cmd_vel.speed = M_PI/2.0;
  // send steering for angular speed of M_PI/10.0
  double cmd_angular = M_PI/10.0;
  /// diff_steering = pi/10 * wheel_base / (pi/2) = tan(front_steering) - tan(rear_steering)
  cmd_vel.front_steering_angle = 0.18776;
  cmd_vel.rear_steering_angle = -0.18776;
  publish_4ws(cmd_vel);
  // wait for 10s to make a half turn
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot traveled 20 meter in XY plane, changes in z should be ~~0
  const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
  const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
  const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
  EXPECT_NEAR(sqrt(dx*dx + dy*dy), fabs(2*cmd_vel.speed/(cmd_angular)), POSITION_TOLERANCE);
  EXPECT_LT(fabs(dz), EPS);

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_LT(fabs(roll_new - roll_old), EPS);
  EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
  EXPECT_NEAR(fabs(yaw_new - yaw_old), M_PI, ORIENTATION_TOLERANCE);

  EXPECT_NEAR(fabs(new_odom.twist.twist.linear.x), cmd_vel.speed, EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

  EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
  EXPECT_NEAR(new_odom.twist.twist.angular.z, cmd_angular, EPS);
}

TEST_F(FourWheelSteeringControllerTest, testNonSymmetricTurn)
{
  // wait for ROS
  waitForController();

  // zero everything before test
  four_wheel_steering_msgs::FourWheelSteering cmd_vel;
  cmd_vel.speed = 0.0;
  cmd_vel.front_steering_angle = 0.0;
  cmd_vel.rear_steering_angle = 0.0;
  publish_4ws(cmd_vel);
  ros::Duration(0.1).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a velocity command
  cmd_vel.speed = M_PI/2.0;
  // send steering for angular speed
  double cmd_angular = -M_PI/10.0;
  /// wheel_base = 1.9
  /// diff_steering = pi/10 * wheel_base / (pi/2) = tan(front_steering) - tan(rear_steering)
  cmd_vel.front_steering_angle = -0.21655;
  cmd_vel.rear_steering_angle = 0.15866;
  publish_4ws(cmd_vel);
  // wait for 10s to make a half turn
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot traveled 20 meter in XY plane, changes in z should be ~~0
  const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
  const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
  const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
  EXPECT_NEAR(sqrt(dx*dx + dy*dy), fabs(2*cmd_vel.speed/(cmd_angular)), POSITION_TOLERANCE);
  EXPECT_LT(fabs(dz), EPS);

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_LT(fabs(roll_new - roll_old), EPS);
  EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
  EXPECT_NEAR(fabs(yaw_new - yaw_old), M_PI, ORIENTATION_TOLERANCE);

  EXPECT_NEAR(sqrt(pow(new_odom.twist.twist.linear.x,2)+pow(new_odom.twist.twist.linear.y,2)),
              fabs(cmd_vel.speed), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

  EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
  EXPECT_NEAR(new_odom.twist.twist.angular.z, cmd_angular, EPS);
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
  ros::init(argc, argv, "four_wheel_steering_4ws_cmd_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
