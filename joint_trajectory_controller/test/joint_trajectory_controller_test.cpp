///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian

#include <cmath>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>

// Floating-point value comparison threshold
const double EPS = 0.01;

class JointTrajectoryControllerTest : public ::testing::Test
{
public:
  JointTrajectoryControllerTest()
    : nh("rrbot_controller"),
      controller_state()/*,
      action_client()*/
  {
    n_joints = (2);
    joint_names.resize(n_joints);
    joint_names[0] = "joint1";
    joint_names[1] = "joint2";

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(n_joints, 0.0);
    point.velocities.resize(n_joints, 0.0);
    point.accelerations.resize(n_joints, 0.0);

    // Go home trajectory
    traj_home.joint_names = joint_names;
    traj_home.points.resize(1, point);
    traj_home.points[0].time_from_start = ros::Duration(1.0);

    // Three-point trajectory
    points.resize(3, point);
    points[0].positions[0] =  M_PI / 4.0;
    points[0].positions[1] =  0.0;
    points[0].time_from_start = ros::Duration(1.0);

    points[1].positions[0] =  0.0;
    points[1].positions[1] = -M_PI / 4.0;
    points[1].time_from_start = ros::Duration(2.0);

    points[2].positions[0] = -M_PI / 4.0;
    points[2].positions[1] =  M_PI / 4.0;
    points[2].time_from_start = ros::Duration(4.0);

    traj.joint_names = joint_names;
    traj.points = points;

    // Action goals
    traj_home_goal.trajectory = traj_home;
    traj_goal.trajectory      = traj;

    // Trajectory publisher
    traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("command", 1);

    // State subscriber
    state_sub = nh.subscribe<control_msgs::JointTrajectoryControllerState>("state",
                                                                           1,
                                                                           &JointTrajectoryControllerTest::stateCB,
                                                                           this);

    // Query state service client
    query_state_service = nh.serviceClient<control_msgs::QueryTrajectoryState>("query_state");

    // Action client
//    action_client.reset(new ActionClient(nh.getNamespace() + "/follow_joint_trajectory"));
  }

protected:
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;
  typedef boost::shared_ptr<ActionClient> ActionClientPtr;
  typedef control_msgs::FollowJointTrajectoryGoal ActionGoal;
  typedef control_msgs::JointTrajectoryControllerStateConstPtr StateConstPtr;

  boost::mutex mutex;
  ros::NodeHandle nh;

  unsigned int n_joints;
  std::vector<std::string> joint_names;
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  trajectory_msgs::JointTrajectory traj_home;
  trajectory_msgs::JointTrajectory traj;
  ActionGoal                       traj_home_goal;
  ActionGoal                       traj_goal;

  ros::Publisher     traj_pub;
  ros::Subscriber    state_sub;
  ros::ServiceClient query_state_service;
//  ActionClientPtr    action_client;


  StateConstPtr controller_state;

  void stateCB(const StateConstPtr& state)
  {
    boost::mutex::scoped_lock lock(mutex);
    controller_state = state;
  }

  StateConstPtr getState()
  {
    boost::mutex::scoped_lock lock(mutex);
    return controller_state;
  }

  bool initState(const ros::Duration& timeout = ros::Duration(5.0))
  {
    bool init_ok = false;
    ros::Time start_time = ros::Time::now();
    while (!init_ok && (ros::Time::now() - start_time) < timeout)
    {
      {
        boost::mutex::scoped_lock lock(mutex);
        init_ok = controller_state && !controller_state->joint_names.empty();
        ROS_ERROR_STREAM(init_ok);
      }
      ros::Duration(0.1).sleep();
    }
    return init_ok;
  }

//  bool initActionClient(const ros::Duration& timeout = ros::Duration(5.0))
//  {
//    return action_client->waitForServer(timeout);
//  }

};

TEST_F(JointTrajectoryControllerTest, stateTopicConsistency)
{
  // Get current controller state
  ASSERT_TRUE(initState());
  StateConstPtr state = getState();

  // Checks that are valid for all state messages
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_EQ(joint_names[i], state->joint_names[i]);
  }

  EXPECT_EQ(n_joints, state->desired.positions.size());
  EXPECT_EQ(n_joints, state->desired.velocities.size());
  EXPECT_EQ(n_joints, state->desired.accelerations.size());

  EXPECT_EQ(n_joints, state->actual.positions.size());
  EXPECT_EQ(n_joints, state->actual.velocities.size());
  EXPECT_TRUE(state->actual.accelerations.empty());

  EXPECT_EQ(n_joints, state->error.positions.size());
  EXPECT_EQ(n_joints, state->error.velocities.size());
  EXPECT_TRUE(state->error.accelerations.empty());
}

TEST_F(JointTrajectoryControllerTest, queryStateServiceConsistency)
{
  query_state_service.waitForExistence(ros::Duration(2.0));
  ASSERT_TRUE(query_state_service.isValid());

  control_msgs::QueryTrajectoryState srv;
  srv.request.time = ros::Time::now();
  ASSERT_TRUE(query_state_service.call(srv));

  // Checks that are valid for all queries
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_EQ(joint_names[i], srv.response.name[i]);
  }

  EXPECT_EQ(n_joints, srv.response.position.size());
  EXPECT_EQ(n_joints, srv.response.velocity.size());
  EXPECT_EQ(n_joints, srv.response.acceleration.size());
}

TEST_F(JointTrajectoryControllerTest, topicInterface)
{
  ASSERT_TRUE(initState());

  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);
  ros::Duration wait_duration = traj.points.back().time_from_start + ros::Duration(0.5);
  wait_duration.sleep(); // Wait until done

  // Validate state topic values
  StateConstPtr state = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(traj.points.back().positions[i],     state->desired.positions[i],     EPS);
    EXPECT_NEAR(traj.points.back().velocities[i],    state->desired.velocities[i],    EPS);
    EXPECT_NEAR(traj.points.back().accelerations[i], state->desired.accelerations[i], EPS);

    EXPECT_NEAR(traj.points.back().positions[i],  state->actual.positions[i],  EPS);
    EXPECT_NEAR(traj.points.back().velocities[i], state->actual.velocities[i], EPS);

    EXPECT_NEAR(0.0, state->error.positions[i],  EPS);
    EXPECT_NEAR(0.0, state->error.velocities[i], EPS);
  }

  // Validate query state service
  control_msgs::QueryTrajectoryState srv;
  srv.request.time = ros::Time::now();
  ASSERT_TRUE(query_state_service.call(srv));
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(state->desired.positions[i],     srv.response.position[i],     EPS);
    EXPECT_NEAR(state->desired.velocities[i],    srv.response.velocity[i],     EPS);
    EXPECT_NEAR(state->desired.accelerations[i], srv.response.acceleration[i], EPS);
  }
}

TEST_F(JointTrajectoryControllerTest, interruptTrajectory)
{
  ASSERT_TRUE(initState());

  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory that preempts the previous one
  traj_home.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj_home);
  wait_duration = traj_home.points.back().time_from_start + ros::Duration(0.5);
  wait_duration.sleep(); // Wait until done

  // Check that we're back home
  StateConstPtr state = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(traj_home.points.back().positions[i],     state->desired.positions[i],     EPS);
    EXPECT_NEAR(traj_home.points.back().velocities[i],    state->desired.velocities[i],    EPS);
    EXPECT_NEAR(traj_home.points.back().accelerations[i], state->desired.accelerations[i], EPS);
  }
}

TEST_F(JointTrajectoryControllerTest, emptyTrajectory)
{
  // Wait for state topic to become available
  ASSERT_TRUE(initState());

  // Start state
  StateConstPtr start_state = getState();

  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send an empty trajectory that preempts the previous one and stops the robot where it is
  trajectory_msgs::JointTrajectory traj_empty;
  traj_pub.publish(traj_empty);
  ros::Duration(2.0).sleep(); // Stopping takes some time

  // Check that we're not on the start state
  StateConstPtr state1 = getState();
  EXPECT_LT(traj.points.front().positions[0] * 0.9,
            std::abs(start_state->desired.positions[0] - state1->desired.positions[0]));

  // Check that we're not moving
  ros::Duration(0.5).sleep(); // Wait
  StateConstPtr state2 = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(state1->desired.positions[i],     state2->desired.positions[i],     EPS);
    EXPECT_NEAR(state1->desired.velocities[i],    state2->desired.velocities[i],    EPS);
    EXPECT_NEAR(state1->desired.accelerations[i], state2->desired.accelerations[i], EPS);
  }
}

TEST_F(JointTrajectoryControllerTest, oldTrajectory)
{
  // Wait for state topic to become available
  ASSERT_TRUE(initState());

  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory with all points occuring in the past. Should not preempts the previous one
  traj_home.header.stamp = ros::Time::now() - traj_home.points.back().time_from_start - ros::Duration(0.5);
  traj_pub.publish(traj_home);

  wait_duration = traj.points.back().time_from_start - traj.points.front().time_from_start + ros::Duration(0.5);
  wait_duration.sleep(); // Wait until first trajectory is done

  // Check that we're at the original trajectory end (NOT back home)
  StateConstPtr state = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(traj.points.back().positions[i],     state->desired.positions[i],     EPS);
    EXPECT_NEAR(traj.points.back().velocities[i],    state->desired.velocities[i],    EPS);
    EXPECT_NEAR(traj.points.back().accelerations[i], state->desired.accelerations[i], EPS);
  }
}


/*
TEST_F(JointTrajectoryControllerTest, actionInterface)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(initActionClient());

  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ros::Duration wait_duration = traj.points.back().time_from_start + ros::Duration(0.5);
  ASSERT_TRUE(action_client->waitForResult(wait_duration));
  EXPECT_EQ(action_client->getState(), actionlib::SimpleClientGoalState::SUCCEEDED); // NOTE: Order reversed because of actionlib limitations

//  // Validate state topic values
//  StateConstPtr state = getState();
//  for (unsigned int i = 0; i < n_joints; ++i)
//  {
//    EXPECT_NEAR(traj.points.back().positions[i],     state->desired.positions[i],     EPS);
//    EXPECT_NEAR(traj.points.back().velocities[i],    state->desired.velocities[i],    EPS);
//    EXPECT_NEAR(traj.points.back().accelerations[i], state->desired.accelerations[i], EPS);

//    EXPECT_NEAR(traj.points.back().positions[i],  state->actual.positions[i],  EPS);
//    EXPECT_NEAR(traj.points.back().velocities[i], state->actual.velocities[i], EPS);

//    EXPECT_NEAR(0.0, state->error.positions[i],  EPS);
//    EXPECT_NEAR(0.0, state->error.velocities[i], EPS);
//  }
}
*/
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "joint_trajectory_controller_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}

