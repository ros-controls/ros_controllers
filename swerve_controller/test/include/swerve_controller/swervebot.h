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

#ifndef SWERVE_CONTROLLER_SWERVEBOT_H
#define SWERVE_CONTROLLER_SWERVEBOT_H

#include <cmath>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <std_srvs/Empty.h>

// Floating-point value comparison threshold
const double EPS = 0.025;
const double POSITION_TOLERANCE = 0.1;    // 10cm/5s = 2 cm-s precision
const double ORIENTATION_TOLERANCE = 0.2; // 0.2rad = 11 degrees precision
const double VELOCITY_TOLERANCE = 0.05;   // not consistent (linear and angular speed)

/**
 * This class allows to test the swerve controller with google test
 */
class SwerveControllerTest : public ::testing::Test
{
public:
    /**
         * \brief Constructor. Create services and topic publishers and subscribers
         */
    SwerveControllerTest()
        : received_first_odom(false), cmd_twist_pub(nh.advertise<geometry_msgs::Twist>("cmd_vel", 100)), odom_sub(nh.subscribe("odom", 100, &SwerveControllerTest::odomCallback, this)), start_srv(nh.serviceClient<std_srvs::Empty>("start")), stop_srv(nh.serviceClient<std_srvs::Empty>("stop")), loop_rate(ros::Rate(40))
    {
    }

    /**
         * \brief Desctructor. Shuts off the subscriber
         */
    ~SwerveControllerTest()
    {
        odom_sub.shutdown();
    }

    /**
         * \brief return le last odometry value
         */
    nav_msgs::Odometry getLastOdom()
    {
        return last_odom;
    }

    /**
         * \brief Publishes a Twist command in a loop for a given period of time
         * \param lin_x linear speed along the X-axis
         * \param lin_y linear speed along the Y-axis
         * \param ang_z angular speed along the Z-axis
         * \param timeout how long to publish the command
        */
    void publish(double lin_x, double lin_y, double ang_z, double timeout)
    {
        geometry_msgs::Twist cmd_vel;
        ros::Time start_time = ros::Time::now();
        ros::Duration to(timeout);
        while (ros::Time::now() - start_time < to)
        {
            cmd_vel.linear.x = lin_x;
            cmd_vel.linear.y = lin_y;
            cmd_vel.angular.z = ang_z;
            cmd_twist_pub.publish(cmd_vel);
            loop_rate.sleep();
        }
    }

    /**
         * \brief Check if controller is still alive
        */
    const bool isControllerAlive()
    {
        return (odom_sub.getNumPublishers() > 0) &&
               ((cmd_twist_pub.getNumSubscribers() > 0) || (cmd_4ws_pub.getNumSubscribers() > 0));
    }

    /**
         * \brief Check if the controller has published the first odometry message
        */
    const bool hasReceivedFirstOdom()
    {
        return received_first_odom;
    }

    /**
         * \brief Start the controller
        */
    void start()
    {
        std_srvs::Empty srv;
        start_srv.call(srv);
    }

    /**
         * \brief Stop the controller
        */
    void stop()
    {
        std_srvs::Empty srv;
        stop_srv.call(srv);
    }

    /**
         * \brief Wait until controller is started
        */
    const void waitForController()
    {
        while (!isControllerAlive() && ros::ok())
        {
            ROS_DEBUG_STREAM_THROTTLE(0.5, "Waiting for controller.");
            ros::Duration(0.1).sleep();
        }
        if (!ros::ok())
            FAIL() << "Something went wrong while executing test.";
    }

    /**
         * \brief Wait until the controller has sent the furst odometry message
        */
    const void waitForOdomMsgs()
    {
        while (!hasReceivedFirstOdom() && ros::ok())
        {
            ROS_DEBUG_STREAM_THROTTLE(0.5, "Waiting for odom messages to be published.");
            ros::Duration(0.01).sleep();
        }
        if (!ros::ok())
            FAIL() << "Something went wrong while executing test.";
    }

private:
    bool received_first_odom;
    ros::NodeHandle nh;
    ros::Publisher cmd_twist_pub, cmd_4ws_pub;
    ros::Subscriber odom_sub;
    nav_msgs::Odometry last_odom;

    ros::ServiceClient start_srv;
    ros::ServiceClient stop_srv;
    ros::Rate loop_rate;

    /**
         * \brief Callback function when odometry message is received
        */
    void odomCallback(const nav_msgs::Odometry &odom)
    {
        ROS_INFO_STREAM("Callback reveived: pos.x: " << odom.pose.pose.position.x
                                                     << ", orient.z: " << odom.pose.pose.orientation.z
                                                     << ", lin_est: " << odom.twist.twist.linear.x
                                                     << ", ang_est: " << odom.twist.twist.angular.z);
        last_odom = odom;
        received_first_odom = true;
    }
};

/**
 * \brief Tansforms a geometry quaternion message to a simple quaternion
* \param quat the geometry_msg quaternion to be transformed
*/
inline tf::Quaternion tfQuatFromGeomQuat(const geometry_msgs::Quaternion &quat)
{
    return tf::Quaternion(quat.x, quat.y, quat.z, quat.w);
}

#endif // SWERVE_CONTROLLER_SWERVEBOT_H
