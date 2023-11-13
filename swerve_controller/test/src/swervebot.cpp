/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Exobotic
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Exobotic nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "swerve_controller/swervebot.h"
#include <tf/transform_listener.h>


/*
 * Test that swerve_controller is publishing the "odom" frame
 * and related transforms
 */
TEST_F(SwerveControllerTest, testOdomFrame)
{
    // Notify the user
    ROS_INFO("Test if odom frames shows up!");

    // Wait unitl ROS shows up
    waitForController();

    // Set up a TF listener
    tf::TransformListener listener;
    ros::Duration(2.0).sleep();

    // Check that the odom frame exists
    EXPECT_TRUE(listener.frameExists("odom"));
}

/*
 * Navigate in a square and check that we returned in the same place
 */
TEST_F(SwerveControllerTest, testSquare)
{
    // Wait for ROS
    waitForController();

    // Publish a Twist message with zero values
    this->publish(0, 0, 0, 0.2);

    // Get initial odometry
    nav_msgs::Odometry old_odom = getLastOdom();

    // Make a square
    this->publish(0.5, 0, 0, 2);
    this->publish(0, 0.5, 0, 2);
    this->publish(-0.5, 0, 0, 2);
    this->publish(0, -0.5, 0, 2);

    // Get elapsed time and expected distance
    nav_msgs::Odometry new_odom = getLastOdom();

    // Publish a Twist message with zero values
    this->publish(0, 0, 0, 0.2);

    // Compute position difference and angle difference
    const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
    const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
    const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
    double roll_old, pitch_old, yaw_old;
    double roll_new, pitch_new, yaw_new;
    tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old,
                                                                                pitch_old, yaw_old);
    tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new,
                                                                                pitch_new, yaw_new);

    // Check position and velocity errors
    EXPECT_LT(fabs(dx), POSITION_TOLERANCE);
    EXPECT_LT(fabs(dy), POSITION_TOLERANCE);
    EXPECT_LT(fabs(dz), EPS);
    EXPECT_LT(fabs(roll_new - roll_old), EPS);
    EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
    EXPECT_LT(fabs(yaw_new - yaw_old), EPS);

    // Reset gazebo for next test
    this->publish(0, 0, 0, 0.2);
    ros::Duration(2.0).sleep();
    std_srvs::Empty resetWorldSrv;
    ros::service::call("/gazebo/reset_world", resetWorldSrv);
    ros::Duration(1.0).sleep();
}


/*
 * Set the robot X speed for a given time and check that the robot has moved correctly
 */
TEST_F(SwerveControllerTest, testForwardX)
{
    // Wait for ROS
    waitForController();

    // Publish a Twist message with zero values
    this->publish(0, 0, 0, 0.2);

    // Get initial odometry
    nav_msgs::Odometry old_odom = getLastOdom();

    // Set Y speed of 0.2 for 5 seconds
    double lin_x = 0.2;
    this->publish(lin_x, 0, 0, 5);

    // Get elapsed time and expected distance
    nav_msgs::Odometry new_odom = getLastOdom();
    const ros::Duration actual_elapsed_time = new_odom.header.stamp - old_odom.header.stamp;
    const double expected_distance = lin_x * actual_elapsed_time.toSec();
    std::cerr <<  actual_elapsed_time.toSec() << std::endl;

    // Compute position difference and angle difference
    const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
    const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
    const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
    double roll_old, pitch_old, yaw_old;
    double roll_new, pitch_new, yaw_new;
    tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old,
                                                                                pitch_old, yaw_old);
    tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new,
                                                                                pitch_new, yaw_new);
    // Check position and velocity errors
    EXPECT_NEAR(sqrt(dx*dx + dy*dy), expected_distance, POSITION_TOLERANCE);
    EXPECT_LT(fabs(dz), EPS);

    EXPECT_LT(fabs(roll_new - roll_old), EPS);
    EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
    EXPECT_LT(fabs(yaw_new - yaw_old), EPS);

    EXPECT_NEAR(fabs(new_odom.twist.twist.linear.x), lin_x, VELOCITY_TOLERANCE);
    EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

    EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.angular.z), EPS);

    // Reset gazebo for next test
    this->publish(0, 0, 0, 0.2);
    ros::Duration(2.0).sleep();
    std_srvs::Empty resetWorldSrv;
    ros::service::call("/gazebo/reset_world", resetWorldSrv);
    ros::Duration(1.0).sleep();
}


/*
 * Set the robot Y speed for a given time and check that the robot has moved correctly
 */
TEST_F(SwerveControllerTest, testForwardY)
{
    // Wait for ROS
    waitForController();

    // Publish a Twist message with zero values
    this->publish(0, 0, 0, 0.2);

    // Get initial odometry
    nav_msgs::Odometry old_odom = getLastOdom();

    // Set Y speed of 0.2 for 5 seconds
    double lin_y = 0.2;
    this->publish(0, lin_y, 0, 5);

    // Get elapsed time and expected distance
    nav_msgs::Odometry new_odom = getLastOdom();
    const ros::Duration actual_elapsed_time = new_odom.header.stamp - old_odom.header.stamp;
    const double expected_distance = lin_y * actual_elapsed_time.toSec();

    // Compute position difference and angle difference
    const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
    const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
    const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
    double roll_old, pitch_old, yaw_old;
    double roll_new, pitch_new, yaw_new;
    tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old,
                                                                                pitch_old, yaw_old);
    tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new,
                                                                                pitch_new, yaw_new);

    // Check position and velocity errors
    EXPECT_NEAR(sqrt(dx*dx + dy*dy), expected_distance, POSITION_TOLERANCE);
    EXPECT_LT(fabs(dz), EPS);

    EXPECT_LT(fabs(roll_new - roll_old), EPS);
    EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
    EXPECT_LT(fabs(yaw_new - yaw_old), EPS);

    EXPECT_LT(fabs(new_odom.twist.twist.linear.x), EPS);
    EXPECT_NEAR(fabs(new_odom.twist.twist.linear.y), lin_y, VELOCITY_TOLERANCE);
    EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

    EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.angular.z), EPS);

    // Reset gazebo for next test
    this->publish(0, 0, 0, 0.2);
    ros::Duration(2.0).sleep();
    std_srvs::Empty resetWorldSrv;
    ros::service::call("/gazebo/reset_world", resetWorldSrv);
    ros::Duration(1.0).sleep();
}

/*
 * Check that the robot is making half a circle clockwise correctly
 */
TEST_F(SwerveControllerTest, testCircleCw)
{
    // Wait for ROS
    waitForController();

    // Publish a Twist message with zero values
    this->publish(0, 0, 0, 0.2);

    // Get initial odometry
    nav_msgs::Odometry old_odom = getLastOdom();

    // Turning (linx = pi*r/d and angz= pi/d):
    double lin_x = M_PI * 2/ 10;
    double ang_z = -M_PI / 10;
    this->publish(lin_x, 0, ang_z, 10);

    // Get expected distance
    nav_msgs::Odometry new_odom = getLastOdom();

    // Compute position difference and angle difference
    const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
    const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
    const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
    double roll_old, pitch_old, yaw_old;
    double roll_new, pitch_new, yaw_new;
    tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old,
                                                                                pitch_old, yaw_old);
    tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new,
                                                                                pitch_new, yaw_new);
    // Check position and velocity errors
    EXPECT_NEAR(sqrt(dx*dx + dy*dy), -2*lin_x/ang_z, 2*POSITION_TOLERANCE);
    EXPECT_LT(fabs(dz), EPS);

    EXPECT_LT(fabs(roll_new - roll_old), EPS);
    EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
    EXPECT_NEAR(fabs(yaw_new - yaw_old), M_PI, ORIENTATION_TOLERANCE);

    EXPECT_NEAR(fabs(new_odom.twist.twist.linear.x), lin_x, VELOCITY_TOLERANCE);
    EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

    EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
    EXPECT_NEAR(new_odom.twist.twist.angular.z, ang_z, VELOCITY_TOLERANCE);

    // Reset gazebo for next test
    this->publish(0, 0, 0, 0.2);
    ros::Duration(2.0).sleep();
    std_srvs::Empty resetWorldSrv;
    ros::service::call("/gazebo/reset_world", resetWorldSrv);
    ros::Duration(1.0).sleep();
}


/*
 * Check that the robot is making half a circle counter-clockwise correctly
 */
TEST_F(SwerveControllerTest, testCircleCcw)
{
    // Wait for ROS
    waitForController();

    // Publish a Twist message with zero values
    this->publish(0, 0, 0, 0.2);

    // Get initial odometry
    nav_msgs::Odometry old_odom = getLastOdom();

    // Turning (linx = pi*r/d and angz= pi/d):
    double lin_x = M_PI * 2 / 10;
    double ang_z = M_PI / 10;
    this->publish(lin_x, 0, ang_z, 10);

    // Get expected distance
    nav_msgs::Odometry new_odom = getLastOdom();

    // Compute position difference and angle difference
    const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
    const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
    const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
    double roll_old, pitch_old, yaw_old;
    double roll_new, pitch_new, yaw_new;
    tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old,
                                                                                pitch_old, yaw_old);
    tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new,
                                                                                pitch_new, yaw_new);

    // Check position and velocity errors
    EXPECT_NEAR(sqrt(dx*dx + dy*dy), 2*lin_x/ang_z, 2*POSITION_TOLERANCE);
    EXPECT_LT(fabs(dz), EPS);

    EXPECT_LT(fabs(roll_new - roll_old), EPS);
    EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
    EXPECT_NEAR(fabs(yaw_new - yaw_old), M_PI, ORIENTATION_TOLERANCE);

    EXPECT_NEAR(fabs(new_odom.twist.twist.linear.x), lin_x, VELOCITY_TOLERANCE);
    EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

    EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
    EXPECT_NEAR(new_odom.twist.twist.angular.z, ang_z, VELOCITY_TOLERANCE);

    // Reset gazebo for next test
    this->publish(0, 0, 0, 0.2);
    ros::Duration(2.0).sleep();
    std_srvs::Empty resetWorldSrv;
    ros::service::call("/gazebo/reset_world", resetWorldSrv);
    ros::Duration(1.0).sleep();
}

/*
 * Check that the robot is turning correctly clockwise around its center
 */
TEST_F(SwerveControllerTest, testTurnCw)
{
    // Wait for ROS
    waitForController();

    // Publish a Twist message with zero values
    this->publish(0, 0, 0, 0.2);

    // Get initial odometry
    nav_msgs::Odometry old_odom = getLastOdom();

    // Turning a half circle
    double ang_z = M_PI / 10;
    this->publish(0, 0, ang_z, 10);

    // Get expected distance
    nav_msgs::Odometry new_odom = getLastOdom();

    // Compute position difference and angle difference
    const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
    const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
    const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
    double roll_old, pitch_old, yaw_old;
    double roll_new, pitch_new, yaw_new;
    tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old,
                                                                                pitch_old, yaw_old);
    tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new,
                                                                                pitch_new, yaw_new);

    // Check if position error does not exceed EPS
    EXPECT_LT(fabs(dx), EPS);
    EXPECT_LT(fabs(dy), EPS);
    EXPECT_LT(fabs(dz), EPS);

    EXPECT_LT(fabs(roll_new - roll_old), EPS);
    EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
    EXPECT_NEAR(fabs(yaw_new - yaw_old), M_PI, ORIENTATION_TOLERANCE);

    EXPECT_LT(fabs(new_odom.twist.twist.linear.x), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

    EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
    EXPECT_NEAR(new_odom.twist.twist.angular.z, ang_z, VELOCITY_TOLERANCE);

    // Reset gazebo for next test
    this->publish(0, 0, 0, 0.2);
    ros::Duration(2.0).sleep();
    std_srvs::Empty resetWorldSrv;
    ros::service::call("/gazebo/reset_world", resetWorldSrv);
    ros::Duration(1.0).sleep();
}


/*
 * Check that the robot is turning correctly counter-clockwise around its center
 */
TEST_F(SwerveControllerTest, testTurnCcw)
{
    // Wait for ROS
    waitForController();

    // Publish a Twist message with zero values
    this->publish(0, 0, 0, 0.2);

    // Get initial odometry
    nav_msgs::Odometry old_odom = getLastOdom();

    // Turning a full and a half circle
    double ang_z = -M_PI / 10;
    this->publish(0, 0, ang_z, 10);

    // Get expected distance
    nav_msgs::Odometry new_odom = getLastOdom();

    // Compute position difference and angle difference
    const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
    const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
    const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
    double roll_old, pitch_old, yaw_old;
    double roll_new, pitch_new, yaw_new;
    tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old,
                                                                                pitch_old, yaw_old);
    tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new,
                                                                                pitch_new, yaw_new);

    // Check if position error does not exceed EPS
    EXPECT_LT(fabs(dx), EPS);
    EXPECT_LT(fabs(dy), EPS);
    EXPECT_LT(fabs(dz), EPS);

    EXPECT_LT(fabs(roll_new - roll_old), EPS);
    EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
    EXPECT_NEAR(fabs(yaw_new - yaw_old), M_PI, ORIENTATION_TOLERANCE);

    EXPECT_LT(fabs(new_odom.twist.twist.linear.x), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

    EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
    EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
    EXPECT_NEAR(new_odom.twist.twist.angular.z, ang_z, VELOCITY_TOLERANCE);

    // Reset gazebo for next test
    this->publish(0, 0, 0, 0.2);
    ros::Duration(2.0).sleep();
    std_srvs::Empty resetWorldSrv;
    ros::service::call("/gazebo/reset_world", resetWorldSrv);
    ros::Duration(1.0).sleep();
}

int main(int argc, char** argv)
{
    // Initialize the test
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "swervebot_test");

    // Start spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Run the tests
    int ret = RUN_ALL_TESTS();

    // Stop all
    spinner.stop();
    ros::shutdown();

    return ret;
}
