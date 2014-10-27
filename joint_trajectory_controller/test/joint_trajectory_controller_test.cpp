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

/// \author Adolfo Rodriguez Tsouroukdissian

#include <algorithm>
#include <cmath>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

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

class JointTrajectoryControllerTest : public ::testing::Test
{
public:
  JointTrajectoryControllerTest()
    : nh("rrbot_controller"),
      short_timeout(1.0),
      long_timeout(10.0),
      controller_state()
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

    // Smoothing publisher (determines how well the robot follows a trajectory)
    smoothing_pub = ros::NodeHandle().advertise<std_msgs::Float64>("smoothing", 1);

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
    const std::string action_server_name = nh.getNamespace() + "/follow_joint_trajectory";
    action_client.reset(new ActionClient(action_server_name));
    action_client2.reset(new ActionClient(action_server_name));
  }

  ~JointTrajectoryControllerTest()
  {
    state_sub.shutdown(); // This is important, to make sure that the callback is not woken up later in the destructor
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

// Controller state ROS API ////////////////////////////////////////////////////////////////////////////////////////////

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

// Invalid messages ////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, invalidMessages)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Invalid size
  {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(1, 0.0);
    point.velocities.resize(1, 0.0);
    point.accelerations.resize(1, 0.0);

    trajectory_msgs::JointTrajectory bad_traj;
    bad_traj.joint_names.resize(1, "joint1");
    bad_traj.points.resize(1, point);
    bad_traj.points[0].time_from_start = ros::Duration(1.0);
    ActionGoal bad_goal;
    bad_goal.trajectory = bad_traj;

    bad_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
    action_client->sendGoal(bad_goal);
    ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::REJECTED, long_timeout));
    EXPECT_EQ(action_client->getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS);
  }

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

// Uninterrupted trajectory execution //////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, topicSingleTraj)
{
  ASSERT_TRUE(initState());

  // Send trajectory
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

TEST_F(JointTrajectoryControllerTest, actionSingleTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);

  // Wait until done
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));
  EXPECT_EQ(action_client->getResult()->error_code, control_msgs::FollowJointTrajectoryResult::SUCCESSFUL);
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

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
}

// Joint reordering ////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, jointReordering)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Message joints are ordered differently than in controller
  ActionGoal reorder_goal = traj_home_goal;
  std::swap(reorder_goal.trajectory.joint_names[0], reorder_goal.trajectory.joint_names[1]);
  reorder_goal.trajectory.points.front().positions[0] = M_PI / 4.0;
  reorder_goal.trajectory.points.front().positions[1] = 0.0;


  reorder_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(reorder_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Validate state topic values
  StateConstPtr state = getState();
  EXPECT_NEAR(reorder_goal.trajectory.points.back().positions[0],     state->desired.positions[1], EPS);
  EXPECT_NEAR(reorder_goal.trajectory.points.back().positions[1],     state->desired.positions[0], EPS);
}

// Joint wraparound ////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, jointWraparound)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Go to home configuration, we need known initial conditions
  traj_home_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_home_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));

  // Trajectory goals that trigger wrapping. Between the first and second goals:
  // - First joint command has a wraparound of -2 loops
  // - Second joint command has a wraparound of +1 loop
  // - Both joints should end up doing an angular displacement of |pi/4| for the second goal
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.resize(n_joints, 0.0);
  point.velocities.resize(n_joints, 0.0);
  point.accelerations.resize(n_joints, 0.0);

  ActionGoal goal1;
  goal1.trajectory.joint_names = joint_names;
  goal1.trajectory.points.resize(1, point);
  goal1.trajectory.points[0].positions[0] = 0.25 * M_PI; //-M_PI / 2.0;
  goal1.trajectory.points[0].positions[1] = 0.50 * M_PI; // M_PI / 2.0;
  goal1.trajectory.points[0].time_from_start = ros::Duration(1.0);

  ActionGoal goal2;
  goal2.trajectory.joint_names = joint_names;
  goal2.trajectory.points.resize(1, point);
  goal2.trajectory.points[0].positions[0] =  4.50 * M_PI;//M_PI / 2.0;
  goal2.trajectory.points[0].positions[1] = -1.75 * M_PI; //0.0;
  goal2.trajectory.points[0].time_from_start = ros::Duration(1.0);

  // Send trajectory
  goal1.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(goal1);

  // Wait until done
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Send trajectory
  goal2.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(goal2);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));

  // Validate state topic values
  StateConstPtr state = getState();
  EXPECT_NEAR(goal1.trajectory.points[0].positions[0] + 0.25 * M_PI, state->desired.positions[0],     EPS);
  EXPECT_NEAR(goal1.trajectory.points[0].positions[1] - 0.25 * M_PI, state->desired.positions[1],     EPS);
}

TEST_F(JointTrajectoryControllerTest, jointWraparoundPiSingularity)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Go to home configuration, we need known initial conditions
  traj_home_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_home_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));

  // Trajectory goals that trigger wrapping.
  // When moving a wrapping joint pi radians (eg. from -pi/2 to pi/2), numeric errors can make the controller think
  // that it has to wrap when in fact it doesn't. This comes from a call to angles::shortest_angular_distance, which
  // sometimes wields pi, and sometimes -pi.
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.resize(n_joints, 0.0);
  point.velocities.resize(n_joints, 0.0);
  point.accelerations.resize(n_joints, 0.0);

  ActionGoal goal1;
  goal1.trajectory.joint_names = joint_names;
  goal1.trajectory.points.resize(1, point);
  goal1.trajectory.points[0].positions[0] = -M_PI / 2.0;
  goal1.trajectory.points[0].positions[1] =  0.0;
  goal1.trajectory.points[0].time_from_start = ros::Duration(1.0);

  ActionGoal goal2;
  goal2.trajectory.joint_names = joint_names;
  goal2.trajectory.points.resize(1, point);
  goal2.trajectory.points[0].positions[0] = M_PI / 2.0;
  goal2.trajectory.points[0].positions[1] = 0.0;
  goal2.trajectory.points[0].time_from_start = ros::Duration(2.0);

  // Send trajectory
  goal1.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(goal1);

  // Wait until done
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Send trajectory
  goal2.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(goal2);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));

  // Validate state topic values
  StateConstPtr state = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(goal2.trajectory.points[0].positions[i], state->desired.positions[i], EPS);
  }
}

// Trajectory replacement //////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, topicReplacesTopicTraj)
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

TEST_F(JointTrajectoryControllerTest, actionReplacesActionTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory that preempts the previous one
  traj_home_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client2->sendGoal(traj_home_goal);
  ASSERT_TRUE(waitForState(action_client2, SimpleClientGoalState::ACTIVE,    short_timeout));
  ASSERT_TRUE(waitForState(action_client,  SimpleClientGoalState::PREEMPTED, short_timeout));

  // Wait until done
  ASSERT_TRUE(waitForState(action_client2, SimpleClientGoalState::SUCCEEDED, long_timeout));
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Check that we're back home
  StateConstPtr state = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(traj_home.points.back().positions[i],     state->desired.positions[i],     EPS);
    EXPECT_NEAR(traj_home.points.back().velocities[i],    state->desired.velocities[i],    EPS);
    EXPECT_NEAR(traj_home.points.back().accelerations[i], state->desired.accelerations[i], EPS);
  }
}

TEST_F(JointTrajectoryControllerTest, actionReplacesTopicTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory that preempts the previous one
  traj_home_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_home_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE,    short_timeout));

  // Wait until done
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Check that we're back home
  StateConstPtr state = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(traj_home.points.back().positions[i],     state->desired.positions[i],     EPS);
    EXPECT_NEAR(traj_home.points.back().velocities[i],    state->desired.velocities[i],    EPS);
    EXPECT_NEAR(traj_home.points.back().accelerations[i], state->desired.accelerations[i], EPS);
  }
}

TEST_F(JointTrajectoryControllerTest, topicReplacesActionTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory that preempts the previous one
  traj_home.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj_home);

  ASSERT_TRUE(waitForState(action_client,  SimpleClientGoalState::PREEMPTED, short_timeout));

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

// Cancel execution ////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, emptyTopicCancelsTopicTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Go to home configuration, we need known initial conditions
  traj_home_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_home_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));

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

TEST_F(JointTrajectoryControllerTest, emptyTopicCancelsActionTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Go to home configuration, we need known initial conditions
  traj_home_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_home_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));

  // Start state
  StateConstPtr start_state = getState();

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send an empty trajectory that preempts the previous one and stops the robot where it is
  trajectory_msgs::JointTrajectory traj_empty;
  traj_pub.publish(traj_empty);
  ASSERT_TRUE(waitForState(action_client,  SimpleClientGoalState::PREEMPTED, short_timeout));
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

TEST_F(JointTrajectoryControllerTest, cancelActionGoal)
{
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  action_client->cancelGoal();
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::PREEMPTED, short_timeout));
}

// Ignore old trajectory points ////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, ignoreOldTopicTraj)
{
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

TEST_F(JointTrajectoryControllerTest, ignoreOldActionTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory with all points occuring in the past. Should not preempts the previous one
  traj_home_goal.trajectory.header.stamp = ros::Time::now() - traj_home.points.back().time_from_start - ros::Duration(0.5);
  action_client2->sendGoal(traj_home_goal);
  ASSERT_TRUE(waitForState(action_client2, SimpleClientGoalState::REJECTED,  short_timeout));

  // Wait until done
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Check that we're at the original trajectory end (NOT back home)
  StateConstPtr state = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(traj.points.back().positions[i],     state->desired.positions[i],     EPS);
    EXPECT_NEAR(traj.points.back().velocities[i],    state->desired.velocities[i],    EPS);
    EXPECT_NEAR(traj.points.back().accelerations[i], state->desired.accelerations[i], EPS);
  }
}

TEST_F(JointTrajectoryControllerTest, ignorePartiallyOldActionTraj)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Send trajectory
  ActionGoal first_goal = traj_goal;
  first_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  first_goal.trajectory.points.pop_back();           // Remove last point
  action_client->sendGoal(first_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  ros::Duration wait_duration = first_goal.trajectory.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory with only the last point not occuring in the past. Only the last point should be executed
  traj_goal.trajectory.header.stamp = ros::Time::now() - traj.points.back().time_from_start + ros::Duration(1.0);
  action_client2->sendGoal(traj_goal);
  ASSERT_TRUE(waitForState(action_client,  SimpleClientGoalState::PREEMPTED, short_timeout));
  ASSERT_TRUE(waitForState(action_client2, SimpleClientGoalState::ACTIVE,    short_timeout));

  // Wait until done
  ASSERT_TRUE(waitForState(action_client2, SimpleClientGoalState::SUCCEEDED, long_timeout));
  ros::Duration(0.5).sleep(); // Allows values to settle to within EPS, especially accelerations

  // Check that we're at the trajectory end
  StateConstPtr state = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(traj.points.back().positions[i],     state->desired.positions[i],     EPS);
    EXPECT_NEAR(traj.points.back().velocities[i],    state->desired.velocities[i],    EPS);
    EXPECT_NEAR(traj.points.back().accelerations[i], state->desired.accelerations[i], EPS);
  }
}

// Tolerance checking //////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, pathToleranceViolation)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Go to home configuration, we need known initial conditions
  traj_home_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_home_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));

  // Make robot respond with a delay
  {
    std_msgs::Float64 smoothing;
    smoothing.data = 0.9;
    smoothing_pub.publish(smoothing);
    ros::Duration(0.5).sleep();
  }

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  EXPECT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  // Wait until done
  EXPECT_TRUE(waitForState(action_client, SimpleClientGoalState::ABORTED, long_timeout));
  EXPECT_EQ(action_client->getResult()->error_code, control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED);

  // Restore perfect control
  {
    std_msgs::Float64 smoothing;
    smoothing.data = 0.0;
    smoothing_pub.publish(smoothing);
    ros::Duration(0.5).sleep();
  }
}

TEST_F(JointTrajectoryControllerTest, goalToleranceViolation)
{
  ASSERT_TRUE(initState());
  ASSERT_TRUE(action_client->waitForServer(long_timeout));

  // Go to home configuration, we need known initial conditions
  traj_home_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_home_goal);
  ASSERT_TRUE(waitForState(action_client, SimpleClientGoalState::SUCCEEDED, long_timeout));

  // Make robot respond with a delay
  {
    std_msgs::Float64 smoothing;
    smoothing.data = 0.95;
    smoothing_pub.publish(smoothing);
    ros::Duration(0.5).sleep();
  }

  // Disable path constraints
  traj_goal.path_tolerance.resize(2);
  traj_goal.path_tolerance[0].name     = "joint1";
  traj_goal.path_tolerance[0].position = -1.0;
  traj_goal.path_tolerance[0].velocity = -1.0;
  traj_goal.path_tolerance[1].name     = "joint2";
  traj_goal.path_tolerance[1].position = -1.0;
  traj_goal.path_tolerance[1].velocity = -1.0;

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  EXPECT_TRUE(waitForState(action_client, SimpleClientGoalState::ACTIVE, short_timeout));

  // Wait until done
  EXPECT_TRUE(waitForState(action_client, SimpleClientGoalState::ABORTED, long_timeout));
  EXPECT_EQ(action_client->getResult()->error_code, control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED);

  // Restore perfect control
  {
    std_msgs::Float64 smoothing;
    smoothing.data = 0.0;
    smoothing_pub.publish(smoothing);
    ros::Duration(0.5).sleep();
  }
}

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
