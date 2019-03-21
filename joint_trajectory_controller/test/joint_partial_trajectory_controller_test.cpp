///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016, Shadow Robot Company Ltd.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
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

/// \author Beatriz Leon

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>

// Floating-point value comparison threshold
const double EPS = 0.01;

using actionlib::SimpleClientGoalState;

class JointPartialTrajectoryControllerTest : public ::testing::Test
{
public:
  JointPartialTrajectoryControllerTest()
    : nh("rrbot_controller"),
      short_timeout(1.0),
      long_timeout(10.0),
      controller_state()
  {
    n_joints = (2);
    n_partial_joints = (2);
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

    // Partial trajectory
    traj_partial.joint_names.resize(1, "joint2");
    points.resize(3, point);
    points[0].positions[0] =  M_PI / 4.0;
    points[0].time_from_start = ros::Duration(1.0);

    points[1].positions[0] =  M_PI / 2.0;
    points[1].time_from_start = ros::Duration(2.0);

    points[2].positions[0] =  -M_PI / 2.0;
    points[2].time_from_start = ros::Duration(4.0);
    traj_partial.points = points;

    // Action goals
    traj_home_goal.trajectory    = traj_home;
    traj_goal.trajectory         = traj;
    traj_partial_goal.trajectory = traj_partial;

    // Smoothing publisher (determines how well the robot follows a trajectory)
    smoothing_pub = ros::NodeHandle().advertise<std_msgs::Float64>("smoothing", 1);

    // Trajectory publisher
    traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("command", 1);

    // State subscriber
    state_sub = nh.subscribe<control_msgs::JointTrajectoryControllerState>("state",
                                                                           1,
                                                                           &JointPartialTrajectoryControllerTest::stateCB,
                                                                           this);

    // Query state service client
    query_state_service = nh.serviceClient<control_msgs::QueryTrajectoryState>("query_state");

    // Action client
    const std::string action_server_name = nh.getNamespace() + "/follow_joint_trajectory";
    action_client.reset(new ActionClient(action_server_name));
    action_client2.reset(new ActionClient(action_server_name));
  }

  ~JointPartialTrajectoryControllerTest()
  {
    state_sub.shutdown(); // This is important, to make sure that the callback is not woken up later in the destructor
  }

protected:
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;
  typedef std::shared_ptr<ActionClient> ActionClientPtr;
  typedef control_msgs::FollowJointTrajectoryGoal ActionGoal;
  typedef control_msgs::JointTrajectoryControllerStateConstPtr StateConstPtr;

  std::mutex mutex;
  ros::NodeHandle nh;

  unsigned int n_joints;
  unsigned int n_partial_joints;
  std::vector<std::string> joint_names;
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  trajectory_msgs::JointTrajectory traj_home;
  trajectory_msgs::JointTrajectory traj;
  trajectory_msgs::JointTrajectory traj_partial;
  ActionGoal                       traj_home_goal;
  ActionGoal                       traj_goal;
  ActionGoal                       traj_partial_goal;

  ros::Duration short_timeout;
  ros::Duration long_timeout;

  ros::Publisher     smoothing_pub;
  ros::Publisher     traj_pub;
  ros::Subscriber    state_sub;
  ros::ServiceClient query_state_service;
  ActionClientPtr    action_client;
  ActionClientPtr    action_client2;


  StateConstPtr controller_state;

  void stateCB(const StateConstPtr& state)
  {
    std::lock_guard<std::mutex> lock(mutex);
    controller_state = state;
  }

  StateConstPtr getState()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return controller_state;
  }

  bool initState(const ros::Duration& timeout = ros::Duration(5.0))
  {
    bool init_ok = false;
    ros::Time start_time = ros::Time::now();
    while (!init_ok && (ros::Time::now() - start_time) < timeout)
    {
      {
        std::lock_guard<std::mutex> lock(mutex);
        init_ok = controller_state && !controller_state->joint_names.empty();
      }
      ros::Duration(0.1).sleep();
    }
    return init_ok;
  }

  static bool waitForState(const ActionClientPtr& action_client,
                           const actionlib::SimpleClientGoalState& state,
                           const ros::Duration& timeout)
  {
    using ros::Time;
    using ros::Duration;

    Time start_time = Time::now();
    while (action_client->getState() != state && ros::ok())
    {
      if (timeout >= Duration(0.0) && (Time::now() - start_time) > timeout) {return false;} // Timed-out
      ros::Duration(0.01).sleep();
    }
    return true;
  }
};

// Invalid messages ////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointPartialTrajectoryControllerTest, invalidMessages)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Incompatible joint names
  {
    ActionGoal bad_goal = traj_home_goal;
    bad_goal.trajectory.joint_names[0] = "bad_name";

    bad_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
    action_client->sendGoal(bad_goal);
    ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::REJECTED, long_timeout));
    EXPECT_EQ(action_client->getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS);
  }

  // No position data
  {
    ActionGoal bad_goal = traj_home_goal;
    bad_goal.trajectory.points[0].positions.clear();

    bad_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
    action_client->sendGoal(bad_goal);
    ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::REJECTED, long_timeout));
    EXPECT_EQ(action_client->getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_GOAL);
  }

  // Incompatible data sizes
  {
    ActionGoal bad_goal = traj_home_goal;
    bad_goal.trajectory.points[0].positions.pop_back();

    bad_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
    action_client->sendGoal(bad_goal);
    ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::REJECTED, long_timeout));
    EXPECT_EQ(action_client->getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_GOAL);
  }

  // Non-strictly increasing waypoint times
  {
    ActionGoal bad_goal = traj_goal;
    bad_goal.trajectory.points[2].time_from_start = bad_goal.trajectory.points[1].time_from_start;

    action_client->sendGoal(bad_goal);
    ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::REJECTED, long_timeout));
    EXPECT_EQ(action_client->getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_GOAL);
  }

  // Empty trajectory through action interface
  // NOTE: Sending an empty trajectory through the topic interface cancels execution of all queued segments, but
  // an empty trajectory is interpreted by the action interface as not having the correct joint names.
  {
    ActionGoal empty_goal;
    action_client->sendGoal(empty_goal);
    ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::REJECTED, long_timeout));
    EXPECT_EQ(action_client->getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS);
  }
}

// Partial trajectory execution   //////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointPartialTrajectoryControllerTest, allowPartialTrajIfSpecified)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send partial trajectory.
  action_client->sendGoal(traj_partial_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, long_timeout));
}

// Uninterrupted Partial trajectory execution //////////////////////////////////////////////////////////////////////////////////

TEST_F(JointPartialTrajectoryControllerTest, topicSinglePartialTraj)
{
  ASSERT_TRUE(initState());

  // Send partial trajectory
  traj_partial.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj_partial);
  ros::Duration wait_duration = traj_partial.points.back().time_from_start + ros::Duration(0.5);
  wait_duration.sleep(); // Wait until done

  // Validate state topic values
  StateConstPtr state = getState();

  EXPECT_NEAR(traj_partial.points.back().positions[0],     state->desired.positions[1],     EPS);
  EXPECT_NEAR(traj_partial.points.back().velocities[0],    state->desired.velocities[1],    EPS);
  EXPECT_NEAR(traj_partial.points.back().accelerations[0], state->desired.accelerations[1], EPS);

  EXPECT_NEAR(traj_partial.points.back().positions[0],  state->actual.positions[1],  EPS);
  EXPECT_NEAR(traj_partial.points.back().velocities[0], state->actual.velocities[1], EPS);

  EXPECT_NEAR(0.0, state->error.positions[1],  EPS);
  EXPECT_NEAR(0.0, state->error.velocities[1], EPS);

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

TEST_F(JointPartialTrajectoryControllerTest, actionSinglePartialTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send partial trajectory
  traj_partial_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_partial_goal);

  // Wait until done
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));
  EXPECT_EQ(action_client->getResult()->error_code, control_msgs::FollowJointTrajectoryResult::SUCCESSFUL);
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Validate state topic values
  StateConstPtr state = getState();

  EXPECT_NEAR(traj_partial.points.back().positions[0],     state->desired.positions[1],     EPS);
  EXPECT_NEAR(traj_partial.points.back().velocities[0],    state->desired.velocities[1],    EPS);
  EXPECT_NEAR(traj_partial.points.back().accelerations[0], state->desired.accelerations[1], EPS);

  EXPECT_NEAR(traj_partial.points.back().positions[0],  state->actual.positions[1],  EPS);
  EXPECT_NEAR(traj_partial.points.back().velocities[0], state->actual.velocities[1], EPS);

  EXPECT_NEAR(0.0, state->error.positions[1],  EPS);
  EXPECT_NEAR(0.0, state->error.velocities[1], EPS);
}

// Trajectory replacement //////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointPartialTrajectoryControllerTest, topicReplacesTopicTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send partial trajectory that preempts the previous one
  traj_partial.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj_partial);
  wait_duration = traj_partial.points.back().time_from_start + ros::Duration(0.5);
  wait_duration.sleep(); // Wait until done

  // Check that we're at the original trajectory for joint1 and on the partial trajectory for joint2
  StateConstPtr state = getState();
  EXPECT_NEAR(traj.points.back().positions[0],     state->desired.positions[0],     EPS);
  EXPECT_NEAR(traj.points.back().velocities[0],    state->desired.velocities[0],    EPS);
  EXPECT_NEAR(traj.points.back().accelerations[0], state->desired.accelerations[0], EPS);

  EXPECT_NEAR(traj_partial.points.back().positions[0],     state->desired.positions[1],     EPS);
  EXPECT_NEAR(traj_partial.points.back().velocities[0],    state->desired.velocities[1],    EPS);
  EXPECT_NEAR(traj_partial.points.back().accelerations[0], state->desired.accelerations[1], EPS);
}

TEST_F(JointPartialTrajectoryControllerTest, actionReplacesActionTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send partial trajectory that preempts the previous one
  traj_partial_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client2->sendGoal(traj_partial_goal);
  ASSERT_TRUE(waitForState(action_client2, SimpleClientGoalState::ACTIVE,    short_timeout));
  ASSERT_TRUE(waitForState(action_client,  SimpleClientGoalState::PREEMPTED, short_timeout));
  //ROS_ERROR_STREAM("state: "<< action_client->getState().toString());

  // Wait until done
  ASSERT_TRUE(waitForState(action_client2, SimpleClientGoalState::SUCCEEDED, long_timeout));
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Check that we're at the original trajectory for joint1 and on the partial trajectory for joint2
  StateConstPtr state = getState();
  EXPECT_NEAR(traj.points.back().positions[0],     state->desired.positions[0],     EPS);
  EXPECT_NEAR(traj.points.back().velocities[0],    state->desired.velocities[0],    EPS);
  EXPECT_NEAR(traj.points.back().accelerations[0], state->desired.accelerations[0], EPS);

  EXPECT_NEAR(traj_partial.points.back().positions[0],     state->desired.positions[1],     EPS);
  EXPECT_NEAR(traj_partial.points.back().velocities[0],    state->desired.velocities[1],    EPS);
  EXPECT_NEAR(traj_partial.points.back().accelerations[0], state->desired.accelerations[1], EPS);
}

TEST_F(JointPartialTrajectoryControllerTest, actionReplacesTopicTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send partial trajectory that preempts the previous one
  traj_partial_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_partial_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE,    short_timeout));

  // Wait until done
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Check that we're at the original trajectory for joint1 and on the partial trajectory for joint2
  StateConstPtr state = getState();
  EXPECT_NEAR(traj.points.back().positions[0],     state->desired.positions[0],     EPS);
  EXPECT_NEAR(traj.points.back().velocities[0],    state->desired.velocities[0],    EPS);
  EXPECT_NEAR(traj.points.back().accelerations[0], state->desired.accelerations[0], EPS);

  EXPECT_NEAR(traj_partial.points.back().positions[0],     state->desired.positions[1],     EPS);
  EXPECT_NEAR(traj_partial.points.back().velocities[0],    state->desired.velocities[1],    EPS);
  EXPECT_NEAR(traj_partial.points.back().accelerations[0], state->desired.accelerations[1], EPS);
}

TEST_F(JointPartialTrajectoryControllerTest, topicReplacesActionTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send partial trajectory that preempts the previous one
  traj_partial.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj_partial);

  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::PREEMPTED, short_timeout));

  wait_duration = traj_partial.points.back().time_from_start + ros::Duration(0.5);
  wait_duration.sleep(); // Wait until done

  // Check that we're at the original trajectory for joint1 and on the partial trajectory for joint2
  StateConstPtr state = getState();
  EXPECT_NEAR(traj.points.back().positions[0],     state->desired.positions[0],     EPS);
  EXPECT_NEAR(traj.points.back().velocities[0],    state->desired.velocities[0],    EPS);
  EXPECT_NEAR(traj.points.back().accelerations[0], state->desired.accelerations[0], EPS);

  EXPECT_NEAR(traj_partial.points.back().positions[0],     state->desired.positions[1],     EPS);
  EXPECT_NEAR(traj_partial.points.back().velocities[0],    state->desired.velocities[1],    EPS);
  EXPECT_NEAR(traj_partial.points.back().accelerations[0], state->desired.accelerations[1], EPS);
}

// Ignore old trajectory points ////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointPartialTrajectoryControllerTest, ignoreOldTopicTraj)
{
  ASSERT_TRUE(initState());

  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send partial trajectory with all points occuring in the past. Should not preempts the previous one
  traj_partial.header.stamp = ros::Time::now() - traj_partial.points.back().time_from_start - ros::Duration(0.5);
  traj_pub.publish(traj_partial);

  wait_duration = traj.points.back().time_from_start - traj.points.front().time_from_start + ros::Duration(0.5);
  wait_duration.sleep(); // Wait until first trajectory is done

  // Check that we're at the original trajectory end (Joint2 has not moved according to traj_partial)
  StateConstPtr state = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(traj.points.back().positions[i],     state->desired.positions[i],     EPS);
    EXPECT_NEAR(traj.points.back().velocities[i],    state->desired.velocities[i],    EPS);
    EXPECT_NEAR(traj.points.back().accelerations[i], state->desired.accelerations[i], EPS);
  }
}

TEST_F(JointPartialTrajectoryControllerTest, ignoreOldActionTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send partial trajectory with all points occuring in the past. Should not preempts the previous one
  traj_partial_goal.trajectory.header.stamp = ros::Time::now() - traj_partial.points.back().time_from_start - ros::Duration(0.5);
  action_client2->sendGoal(traj_partial_goal);
  ASSERT_TRUE(waitForState(action_client2, SimpleClientGoalState::REJECTED,  short_timeout));

  // Wait until done
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Check that we're at the original trajectory end (Joint2 has not moved according to traj_partial)
  StateConstPtr state = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(traj.points.back().positions[i],     state->desired.positions[i],     EPS);
    EXPECT_NEAR(traj.points.back().velocities[i],    state->desired.velocities[i],    EPS);
    EXPECT_NEAR(traj.points.back().accelerations[i], state->desired.accelerations[i], EPS);
  }
}

TEST_F(JointPartialTrajectoryControllerTest, ignorePartiallyOldActionTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  ActionGoal first_goal = traj_goal;
  first_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(first_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  ros::Duration wait_duration = first_goal.trajectory.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send partial trajectory with only the last point not occuring in the past. Only the last point should be executed
  traj_partial_goal.trajectory.header.stamp = ros::Time::now() - traj.points.back().time_from_start + ros::Duration(1.0);
  action_client2->sendGoal(traj_partial_goal);
  ASSERT_TRUE(waitForState(action_client,  SimpleClientGoalState::PREEMPTED, short_timeout));
  ASSERT_TRUE(waitForState(action_client2, SimpleClientGoalState::ACTIVE,    short_timeout));

  // Wait until done
  ASSERT_TRUE(waitForState(action_client2, SimpleClientGoalState::SUCCEEDED, long_timeout));
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Check that we're at the original trajectory for joint1 and on the partial trajectory for joint2
  StateConstPtr state = getState();
  EXPECT_NEAR(traj.points.back().positions[0],     state->desired.positions[0],     EPS);
  EXPECT_NEAR(traj.points.back().velocities[0],    state->desired.velocities[0],    EPS);
  EXPECT_NEAR(traj.points.back().accelerations[0], state->desired.accelerations[0], EPS);

  EXPECT_NEAR(traj_partial.points.back().positions[0],     state->desired.positions[1],     EPS);
  EXPECT_NEAR(traj_partial.points.back().velocities[0],    state->desired.velocities[1],    EPS);
  EXPECT_NEAR(traj_partial.points.back().accelerations[0], state->desired.accelerations[1], EPS);
}

TEST_F(JointPartialTrajectoryControllerTest, executeParitalActionTrajInFuture)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  ActionGoal first_goal = traj_goal;
  first_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(first_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  ros::Duration wait_duration = first_goal.trajectory.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send partial trajectory with only the last point not occuring in the past. Only the last point should be executed
  traj_partial_goal.trajectory.header.stamp = ros::Time::now() + traj.points.back().time_from_start + ros::Duration(2.0);
  action_client2->sendGoal(traj_partial_goal);
  ASSERT_TRUE(waitForState(action_client,  SimpleClientGoalState::PREEMPTED, short_timeout));
  ASSERT_TRUE(waitForState(action_client2, SimpleClientGoalState::ACTIVE,    short_timeout));

  // Wait until first traj is done
  wait_duration = first_goal.trajectory.points.back().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  StateConstPtr state = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(traj.points.back().positions[i],     state->desired.positions[i],     EPS);
    EXPECT_NEAR(traj.points.back().velocities[i],    state->desired.velocities[i],    EPS);
    EXPECT_NEAR(traj.points.back().accelerations[i], state->desired.accelerations[i], EPS);
  }

  // Wait until done
  ASSERT_TRUE(waitForState(action_client2, SimpleClientGoalState::SUCCEEDED, long_timeout));
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Check that we're at the original trajectory for joint1 and on the partial trajectory for joint2
  state = getState();
  EXPECT_NEAR(traj.points.back().positions[0],     state->desired.positions[0],     EPS);
  EXPECT_NEAR(traj.points.back().velocities[0],    state->desired.velocities[0],    EPS);
  EXPECT_NEAR(traj.points.back().accelerations[0], state->desired.accelerations[0], EPS);

  EXPECT_NEAR(traj_partial.points.back().positions[0],     state->desired.positions[1],     EPS);
  EXPECT_NEAR(traj_partial.points.back().velocities[0],    state->desired.velocities[1],    EPS);
  EXPECT_NEAR(traj_partial.points.back().accelerations[0], state->desired.accelerations[1], EPS);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "joint_partial_trajectory_controller_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
