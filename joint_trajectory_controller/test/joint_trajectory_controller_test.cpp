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
#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>

#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>

#include "test_common.h"

using actionlib::SimpleClientGoalState;
using testing::AssertionResult;
using testing::AssertionFailure;
using testing::AssertionSuccess;

using namespace joint_trajectory_controller_tests;

static constexpr double TIMEOUT_TRAJ_EXECUTION_S = 5.0;

class JointTrajectoryControllerTest : public ::testing::Test
{
public:
  JointTrajectoryControllerTest()
    : controller_nh("rrbot_controller"),
      controller_state(),
      stop_trajectory_duration(0.0)
  {
    n_joints = (2);
    joint_names.resize(n_joints);
    joint_names[0] = "joint1";
    joint_names[1] = "joint2";

    controller_min_actual_velocity.resize(n_joints, std::numeric_limits<double>::infinity());

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

    ros::NodeHandle nh{};

    // Smoothing publisher (determines how well the robot follows a trajectory)
    smoothing_pub = nh.advertise<std_msgs::Float64>("smoothing", 1);
    smoothings_pub = nh.advertise<std_msgs::Float64MultiArray>("smoothings", 1);

    // Delay publisher (allows to simulate a delay of one cycle in the hardware interface)
    delay_pub = nh.advertise<std_msgs::Bool>("delay", 1);

    // Upper bound publisher (allows to simulate a wall)
    upper_bound_pub = nh.advertise<std_msgs::Float64>("upper_bound", 1);

    // Trajectory publisher
    traj_pub = controller_nh.advertise<trajectory_msgs::JointTrajectory>("command", 1);

    // State subscriber
    state_sub = controller_nh.subscribe<control_msgs::JointTrajectoryControllerState>("state",
                                                                           1,
                                                                           &JointTrajectoryControllerTest::stateCB,
                                                                           this);

    // Robot ready subscriber (gets notified about successful parameter update)
    robot_ready_sub = nh.subscribe<std_msgs::Empty>("parameter_updated",
                                                    1,
                                                    &JointTrajectoryControllerTest::robotReadyCB,
                                                    this);

    // Query state service client
    query_state_service = controller_nh.serviceClient<control_msgs::QueryTrajectoryState>("query_state");

    // Controller management services
    {
      using namespace controller_manager_msgs;
      load_controller_service = controller_nh.serviceClient<LoadController>("/controller_manager/load_controller");
      unload_controller_service = controller_nh.serviceClient<UnloadController>("/controller_manager/unload_controller");
      switch_controller_service = controller_nh.serviceClient<SwitchController>("/controller_manager/switch_controller");
    }

    // Action client
    const std::string action_server_name = controller_nh.getNamespace() + "/follow_joint_trajectory";
    action_client.reset(new ActionClient(action_server_name));
    action_client2.reset(new ActionClient(action_server_name));

    controller_nh.getParam("stop_trajectory_duration", stop_trajectory_duration);
  }

  ~JointTrajectoryControllerTest()
  {
    state_sub.shutdown(); // This is important, to make sure that the callback is not woken up later in the destructor
  }

  /**
   * \brief Wait for connection to controller and move robot to home position (if not already there).
   */
  void SetUp() override
  {
    ASSERT_TRUE(waitForActionServer(action_client));
    ASSERT_TRUE(waitForInitializedState());

    if (!checkPointReached(traj_home.points.back()))
    {
      traj_home_goal.trajectory.header.stamp = ros::Time(0);
      action_client->sendGoal(traj_home_goal);

      auto timeout = getTrajectoryDuration(traj_home_goal.trajectory) + ros::Duration(TIMEOUT_ACTION_RESULT_S);
      ASSERT_TRUE(waitForActionResult(action_client, timeout));
      EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::SUCCEEDED));
    }
  }

protected:
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;
  typedef std::shared_ptr<ActionClient> ActionClientPtr;
  typedef control_msgs::FollowJointTrajectoryGoal ActionGoal;
  typedef control_msgs::JointTrajectoryControllerState State;
  typedef control_msgs::JointTrajectoryControllerStateConstPtr StateConstPtr;

  std::mutex mutex;
  ros::NodeHandle controller_nh;

  unsigned int n_joints;
  std::vector<std::string> joint_names;
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  trajectory_msgs::JointTrajectory traj_home;
  trajectory_msgs::JointTrajectory traj;
  ActionGoal                       traj_home_goal;
  ActionGoal                       traj_goal;

  ros::Publisher     smoothing_pub;
  ros::Publisher     smoothings_pub;
  ros::Publisher     delay_pub;
  ros::Publisher     upper_bound_pub;
  ros::Publisher     traj_pub;
  ros::Subscriber    state_sub;
  ros::Subscriber    robot_ready_sub;
  ros::ServiceClient query_state_service;
  ros::ServiceClient load_controller_service;
  ros::ServiceClient unload_controller_service;
  ros::ServiceClient switch_controller_service;
  ActionClientPtr    action_client;
  ActionClientPtr    action_client2;


  StateConstPtr controller_state;
  std::vector<double> controller_min_actual_velocity;

  double stop_trajectory_duration;

  std::atomic_bool robot_ready{false};

  void robotReadyCB(const std_msgs::EmptyConstPtr& msg)
  {
    robot_ready = true;
  }

  void stateCB(const StateConstPtr& state)
  {
    std::lock_guard<std::mutex> lock(mutex);
    controller_state = state;

    std::transform(controller_min_actual_velocity.begin(), controller_min_actual_velocity.end(),
                   state->actual.velocities.begin(), controller_min_actual_velocity.begin(),
                   [](double a, double b){return std::min(a, b);});
  }

  const State getState()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return *controller_state;
  }

  AssertionResult waitForRobotReady(const ros::Duration& timeout = ros::Duration(TIMEOUT_CONNECTIONS_S))
  {
    AssertionResult result = waitForEvent([this](){return robot_ready.load();}, "robot ready (after parameter update)", timeout);
    robot_ready = false;
    return result;
  }

  AssertionResult waitForInitializedState(const ros::Duration& timeout = ros::Duration(TIMEOUT_CONNECTIONS_S))
  {
    auto state_initialized = [this]()
    {
      std::lock_guard<std::mutex> lock(mutex);
      return (controller_state && !controller_state->joint_names.empty());
    };
    return waitForEvent(state_initialized, "controller-state initialized", timeout);
  }

  AssertionResult waitForQueryStateService(const ros::Duration& timeout = ros::Duration(TIMEOUT_CONNECTIONS_S))
  {
    if (!query_state_service.waitForExistence(timeout))
    {
      return AssertionFailure() << "Timed out after " << timeout.toSec()
                                << "s waiting for existence of wuery-state service.";
    }
    return AssertionSuccess();
  }

  std::vector<double> getMinActualVelocity()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return controller_min_actual_velocity;
  }

  void resetControllerStateObserver()
  {
    std::lock_guard<std::mutex> lock(mutex);
    controller_state.reset();

    for(size_t i = 0; i < controller_min_actual_velocity.size(); ++i)
    {
      controller_min_actual_velocity[i] = std::numeric_limits<double>::infinity();
    }
  }

  AssertionResult waitForTrajectoryExecution(const ros::Duration& timeout = ros::Duration(TIMEOUT_TRAJ_EXECUTION_S))
  {
    State start_state = getState();
    auto executing = [this, &start_state]()
    {
      State current_state = getState();
      return !vectorsAlmostEqual(start_state.desired.positions, current_state.desired.positions);
    };
    return waitForEvent(executing, "trajectory execution", timeout);
  }

  bool checkPointReached(const trajectory_msgs::JointTrajectoryPoint& point)
  {
    State current_state = getState();
    return trajectoryPointsAlmostEqual(current_state.desired, point) &&
           vectorsAlmostEqual(current_state.actual.positions, point.positions) &&
           vectorsAlmostEqual(current_state.actual.velocities, point.velocities);
  }

  bool checkStopped()
  {
    std::vector<double> zero_vec(n_joints, 0.0);
    State current_state = getState();
    return vectorsAlmostEqual(current_state.actual.velocities, zero_vec) &&
           vectorsAlmostEqual(current_state.desired.velocities, zero_vec) &&
           vectorsAlmostEqual(current_state.desired.accelerations, zero_vec);
  }

  AssertionResult waitForStop(const ros::Duration& timeout = ros::Duration(TIMEOUT_TRAJ_EXECUTION_S))
  {
    return waitForEvent(std::bind(&JointTrajectoryControllerTest::checkStopped, this), "stop", timeout, 3U);
  }

  bool reloadController(const std::string& name)
  {
    controller_manager_msgs::SwitchController stop_controller;
    stop_controller.request.stop_controllers.push_back(name);
    stop_controller.request.strictness = stop_controller.request.STRICT;
    if(!switch_controller_service.call(stop_controller)) return false;
    if(!stop_controller.response.ok) return false;

    controller_manager_msgs::UnloadController unload_controller;
    unload_controller.request.name = name;
    if(!unload_controller_service.call(unload_controller)) return false;
    if(!unload_controller.response.ok) return false;

    controller_manager_msgs::LoadController load_controller;
    load_controller.request.name = name;
    if(!load_controller_service.call(load_controller)) return false;
    if(!load_controller.response.ok) return false;

    controller_manager_msgs::SwitchController start_controller;
    start_controller.request.start_controllers.push_back(name);
    start_controller.request.strictness = start_controller.request.STRICT;
    if(!switch_controller_service.call(start_controller)) return false;
    if(!start_controller.response.ok) return false;
    return true;
  }
};

// Controller state ROS API ////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, stateTopicConsistency)
{
  // Get current controller state
  State state = getState();

  // Checks that are valid for all state messages
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_EQ(joint_names[i], state.joint_names[i]);
  }

  EXPECT_EQ(n_joints, state.desired.positions.size());
  EXPECT_EQ(n_joints, state.desired.velocities.size());
  EXPECT_EQ(n_joints, state.desired.accelerations.size());

  EXPECT_EQ(n_joints, state.actual.positions.size());
  EXPECT_EQ(n_joints, state.actual.velocities.size());
  EXPECT_TRUE(state.actual.accelerations.empty());

  EXPECT_EQ(n_joints, state.error.positions.size());
  EXPECT_EQ(n_joints, state.error.velocities.size());
  EXPECT_TRUE(state.error.accelerations.empty());
}

TEST_F(JointTrajectoryControllerTest, queryStateServiceConsistency)
{
  ASSERT_TRUE(waitForQueryStateService());
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
  using control_msgs::FollowJointTrajectoryResult;

  // Invalid size (No partial joints goals allowed)
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
    ASSERT_TRUE(waitForActionResult(action_client));
    EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::REJECTED));
    EXPECT_TRUE(checkActionResultErrorCode(action_client, FollowJointTrajectoryResult::INVALID_JOINTS));
  }

  // Incompatible joint names
  {
    ActionGoal bad_goal = traj_home_goal;
    bad_goal.trajectory.joint_names[0] = "bad_name";

    bad_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
    action_client->sendGoal(bad_goal);
    ASSERT_TRUE(waitForActionResult(action_client));
    EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::REJECTED));
    EXPECT_TRUE(checkActionResultErrorCode(action_client, FollowJointTrajectoryResult::INVALID_JOINTS));
  }

  // No position data
  {
    ActionGoal bad_goal = traj_home_goal;
    bad_goal.trajectory.points[0].positions.clear();

    bad_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
    action_client->sendGoal(bad_goal);
    ASSERT_TRUE(waitForActionResult(action_client));
    EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::REJECTED));
    EXPECT_TRUE(checkActionResultErrorCode(action_client, FollowJointTrajectoryResult::INVALID_GOAL));
  }

  // Incompatible data sizes
  {
    ActionGoal bad_goal = traj_home_goal;
    bad_goal.trajectory.points[0].positions.pop_back();

    bad_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
    action_client->sendGoal(bad_goal);
    ASSERT_TRUE(waitForActionResult(action_client));
    EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::REJECTED));
    EXPECT_TRUE(checkActionResultErrorCode(action_client, FollowJointTrajectoryResult::INVALID_GOAL));
  }

  // Non-strictly increasing waypoint times
  {
    ActionGoal bad_goal = traj_goal;
    bad_goal.trajectory.points[2].time_from_start = bad_goal.trajectory.points[1].time_from_start;

    action_client->sendGoal(bad_goal);
    ASSERT_TRUE(waitForActionResult(action_client));
    EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::REJECTED));
    EXPECT_TRUE(checkActionResultErrorCode(action_client, FollowJointTrajectoryResult::INVALID_GOAL));
  }

  // Empty trajectory through action interface
  // NOTE: Sending an empty trajectory through the topic interface cancels execution of all queued segments, but
  // an empty trajectory is interpreted by the action interface as not having the correct joint names.
  {
    ActionGoal empty_goal;
    action_client->sendGoal(empty_goal);
    ASSERT_TRUE(waitForActionResult(action_client));
    EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::REJECTED));
    EXPECT_TRUE(checkActionResultErrorCode(action_client, FollowJointTrajectoryResult::INVALID_JOINTS));
  }
}

// Uninterrupted trajectory execution //////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, topicSingleTraj)
{
  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);
  ASSERT_TRUE(waitForTrajectoryExecution());

  auto timeout = getTrajectoryDuration(traj) + ros::Duration(TIMEOUT_TRAJ_EXECUTION_S);
  ASSERT_TRUE(waitForStop(timeout)); // Allows values to settle within JOINT_STATES_COMPARISON_EPS, especially accelerations

  // Validate state topic values
  State state = getState();

  EXPECT_TRUE(trajectoryPointsAlmostEqual(traj.points.back(), state.desired));
  EXPECT_TRUE(vectorsAlmostEqual(traj.points.back().positions, state.actual.positions));
  EXPECT_TRUE(vectorsAlmostEqual(traj.points.back().velocities, state.actual.velocities));

  std::vector<double> zero_vec(n_joints, 0.0);
  EXPECT_TRUE(vectorsAlmostEqual(state.error.positions, zero_vec));
  EXPECT_TRUE(vectorsAlmostEqual(state.error.velocities, zero_vec));

  // Validate query state service
  control_msgs::QueryTrajectoryState srv;
  srv.request.time = ros::Time::now();
  ASSERT_TRUE(query_state_service.call(srv));

  EXPECT_TRUE(vectorsAlmostEqual(state.desired.positions, srv.response.position));
  EXPECT_TRUE(vectorsAlmostEqual(state.desired.velocities, srv.response.velocity));
  EXPECT_TRUE(vectorsAlmostEqual(state.desired.accelerations, srv.response.acceleration));
}

TEST_F(JointTrajectoryControllerTest, actionSingleTraj)
{
  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  EXPECT_TRUE(waitForActionGoalState(action_client, SimpleClientGoalState::ACTIVE));

  auto timeout = getTrajectoryDuration(traj_goal.trajectory) + ros::Duration(TIMEOUT_ACTION_RESULT_S);
  ASSERT_TRUE(waitForActionResult(action_client, timeout));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::SUCCEEDED));
  EXPECT_TRUE(checkActionResultErrorCode(action_client, control_msgs::FollowJointTrajectoryResult::SUCCESSFUL));

  EXPECT_TRUE(waitForStop()); // Allows values to settle within JOINT_STATES_COMPARISON_EPS, especially accelerations

  // Validate state topic values
  State state = getState();

  EXPECT_TRUE(trajectoryPointsAlmostEqual(traj.points.back(), state.desired));
  EXPECT_TRUE(vectorsAlmostEqual(traj.points.back().positions, state.actual.positions));
  EXPECT_TRUE(vectorsAlmostEqual(traj.points.back().velocities, state.actual.velocities));

  std::vector<double> zero_vec(n_joints, 0.0);
  EXPECT_TRUE(vectorsAlmostEqual(state.error.positions, zero_vec));
  EXPECT_TRUE(vectorsAlmostEqual(state.error.velocities, zero_vec));
}

// Joint reordering ////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, jointReordering)
{
  // Message joints are ordered differently than in controller
  ActionGoal reorder_goal = traj_home_goal;
  std::swap(reorder_goal.trajectory.joint_names[0], reorder_goal.trajectory.joint_names[1]);
  reorder_goal.trajectory.points.front().positions[0] = M_PI / 4.0;
  reorder_goal.trajectory.points.front().positions[1] = 0.0;

  reorder_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(reorder_goal);

  auto timeout = getTrajectoryDuration(reorder_goal.trajectory) + ros::Duration(TIMEOUT_ACTION_RESULT_S);
  ASSERT_TRUE(waitForActionResult(action_client, timeout));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::SUCCEEDED));

  EXPECT_TRUE(waitForStop()); // Allows values to settle within JOINT_STATES_COMPARISON_EPS, especially accelerations

  // Validate state topic values
  State state = getState();
  EXPECT_NEAR(reorder_goal.trajectory.points.back().positions[0],
              state.desired.positions[1],
              JOINT_STATES_COMPARISON_EPS);
  EXPECT_NEAR(reorder_goal.trajectory.points.back().positions[1],
              state.desired.positions[0],
              JOINT_STATES_COMPARISON_EPS);
}

// Joint wraparound ////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, jointWraparound)
{
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

  auto timeout = getTrajectoryDuration(goal1.trajectory) + ros::Duration(TIMEOUT_ACTION_RESULT_S);
  ASSERT_TRUE(waitForActionResult(action_client, timeout));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::SUCCEEDED));

  // Send trajectory
  goal2.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(goal2);
  
  auto timeout2 = getTrajectoryDuration(goal2.trajectory) + ros::Duration(TIMEOUT_ACTION_RESULT_S);
  ASSERT_TRUE(waitForActionResult(action_client, timeout2));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::SUCCEEDED));

  EXPECT_TRUE(waitForStop()); // Allows values to settle within JOINT_STATES_COMPARISON_EPS, especially accelerations

  // Validate state topic values
  State state = getState();
  EXPECT_NEAR(goal1.trajectory.points[0].positions[0] + 0.25 * M_PI,
              state.desired.positions[0],
              JOINT_STATES_COMPARISON_EPS);
  EXPECT_NEAR(goal1.trajectory.points[0].positions[1] - 0.25 * M_PI,
              state.desired.positions[1],
              JOINT_STATES_COMPARISON_EPS);
}

TEST_F(JointTrajectoryControllerTest, jointWraparoundPiSingularity)
{
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

  auto timeout = getTrajectoryDuration(goal1.trajectory) + ros::Duration(TIMEOUT_ACTION_RESULT_S);
  ASSERT_TRUE(waitForActionResult(action_client, timeout));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::SUCCEEDED));

  // Send trajectory
  goal2.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(goal2);
  
  auto timeout2 = getTrajectoryDuration(goal2.trajectory) + ros::Duration(TIMEOUT_ACTION_RESULT_S);
  ASSERT_TRUE(waitForActionResult(action_client, timeout2));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::SUCCEEDED));

  EXPECT_TRUE(waitForStop()); // Allows values to settle within JOINT_STATES_COMPARISON_EPS, especially accelerations

  // Validate state topic values
  State state = getState();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_NEAR(goal2.trajectory.points[0].positions[i], state.desired.positions[i], JOINT_STATES_COMPARISON_EPS);
  }
}

// Trajectory replacement //////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, topicReplacesTopicTraj)
{
  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);

  ASSERT_TRUE(waitForTrajectoryExecution());
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory that preempts the previous one
  traj_home.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj_home);

  auto timeout = getTrajectoryDuration(traj) + ros::Duration(TIMEOUT_TRAJ_EXECUTION_S);
  EXPECT_TRUE(waitForStop(timeout));

  // Check that we're back home
  EXPECT_TRUE(checkPointReached(traj_home.points.back()));
}

TEST_F(JointTrajectoryControllerTest, actionReplacesActionTraj)
{
  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForTrajectoryExecution());

  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory that preempts the previous one
  traj_home_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client2->sendGoal(traj_home_goal);
  EXPECT_TRUE(waitForActionGoalState(action_client2, SimpleClientGoalState::ACTIVE));
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client,  SimpleClientGoalState::PREEMPTED));

  // Wait until done
  auto timeout = getTrajectoryDuration(traj_home_goal.trajectory) + ros::Duration(TIMEOUT_ACTION_RESULT_S);
  ASSERT_TRUE(waitForActionResult(action_client2, timeout));
  EXPECT_TRUE(checkActionGoalState(action_client2, SimpleClientGoalState::SUCCEEDED));

  EXPECT_TRUE(waitForStop());

  // Check that we're back home
  EXPECT_TRUE(checkPointReached(traj_home.points.back()));
}

TEST_F(JointTrajectoryControllerTest, actionReplacesTopicTraj)
{
  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);

  ASSERT_TRUE(waitForTrajectoryExecution());
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory that preempts the previous one
  traj_home_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_home_goal);
  EXPECT_TRUE(waitForActionGoalState(action_client, SimpleClientGoalState::ACTIVE));

  // Wait until done
  auto timeout = getTrajectoryDuration(traj_home_goal.trajectory) + ros::Duration(TIMEOUT_ACTION_RESULT_S);
  ASSERT_TRUE(waitForActionResult(action_client, timeout));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::SUCCEEDED));

  EXPECT_TRUE(waitForStop());

  // Check that we're back home
  EXPECT_TRUE(checkPointReached(traj_home.points.back()));
}

TEST_F(JointTrajectoryControllerTest, topicReplacesActionTraj)
{
  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForTrajectoryExecution());

  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory that preempts the previous one
  traj_home.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj_home);

  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client,  SimpleClientGoalState::PREEMPTED));

  auto timeout = getTrajectoryDuration(traj) + ros::Duration(TIMEOUT_TRAJ_EXECUTION_S);
  EXPECT_TRUE(waitForStop(timeout));

  // Check that we're back home
  EXPECT_TRUE(checkPointReached(traj_home.points.back()));
}

// Cancel execution ////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, emptyTopicCancelsTopicTraj)
{
  // Start state
  State start_state = getState();

  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);
  ASSERT_TRUE(waitForTrajectoryExecution());
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send an empty trajectory that preempts the previous one and stops the robot where it is
  trajectory_msgs::JointTrajectory traj_empty;
  traj_pub.publish(traj_empty);
  ros::Duration(stop_trajectory_duration).sleep(); // Stopping takes some time

  EXPECT_TRUE(waitForStop());

  // Check that we're not on the start state
  State current_state = getState();
  EXPECT_FALSE(vectorsAlmostEqual(start_state.actual.positions, current_state.actual.positions))
    << "Failed to move away from start state.";
}

TEST_F(JointTrajectoryControllerTest, emptyTopicCancelsActionTraj)
{
  // Start state
  State start_state = getState();

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForTrajectoryExecution());
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send an empty trajectory that preempts the previous one and stops the robot where it is
  trajectory_msgs::JointTrajectory traj_empty;
  traj_pub.publish(traj_empty);
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client,  SimpleClientGoalState::PREEMPTED));
  ros::Duration(stop_trajectory_duration).sleep(); // Stopping takes some time

  EXPECT_TRUE(waitForStop());

  // Check that we're not on the start state
  State current_state = getState();
  EXPECT_FALSE(vectorsAlmostEqual(start_state.actual.positions, current_state.actual.positions))
    << "Failed to move away from start state.";
}

TEST_F(JointTrajectoryControllerTest, emptyTopicCancelsActionTrajWithDelay)
{
  // check stop ramp for trajectory duration > 0
  if(stop_trajectory_duration > 0)
  {
    // Make robot respond with a delay
    {
      std_msgs::Bool delay;
      delay.data = true;
      delay_pub.publish(delay);
      ASSERT_TRUE(waitForRobotReady());
    }
    resetControllerStateObserver();

    // Send trajectory (avoid changing sign of velocity -> stretch first segment)
    traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
    traj_goal.trajectory.points.resize(1);
    traj_goal.trajectory.points.at(0).time_from_start = ros::Duration(4.0);
    action_client->sendGoal(traj_goal);
    EXPECT_TRUE(waitForActionGoalState(action_client, SimpleClientGoalState::ACTIVE));

    ros::Duration wait_duration = traj_goal.trajectory.points.front().time_from_start * 0.5;
    wait_duration.sleep(); // Wait until half of first segment

    trajectory_msgs::JointTrajectory traj_empty;
    traj_pub.publish(traj_empty);
    ASSERT_TRUE(waitForActionResult(action_client));
    EXPECT_TRUE(checkActionGoalState(action_client,  SimpleClientGoalState::PREEMPTED));

    EXPECT_TRUE(waitForStop());

    // Velocity should be continuous so all axes >= 0
    std::vector<double> minVelocity = getMinActualVelocity();
    for(size_t i = 0; i < minVelocity.size(); ++i)
    {
      EXPECT_GE(minVelocity[i], -EPS);
    }

    // Restore perfect control
    {
      std_msgs::Bool delay;
      delay.data = false;
      delay_pub.publish(delay);
      EXPECT_TRUE(waitForRobotReady());
    }
  }
  else
  {
    SUCCEED();
  }
}

TEST_F(JointTrajectoryControllerTest, emptyTopicCancelsActionTrajWithDelayStopZero)
{
  // check set position = actual position for stop_duration == 0

  // Make robot respond with a delay and clip position at a wall
  std_msgs::Float64 upper_bound;
  {
    std_msgs::Bool delay;
    delay.data = true;
    delay_pub.publish(delay);
    ASSERT_TRUE(waitForRobotReady());

    upper_bound.data = traj_goal.trajectory.points.front().positions.front() / 3.;
    upper_bound_pub.publish(upper_bound);    
    ASSERT_TRUE(waitForRobotReady());
  }

  // Increase path tolerance
  double old_path_tolerance;
  ASSERT_TRUE(ros::param::get("/rrbot_controller/constraints/joint1/trajectory", old_path_tolerance));
  ros::param::set("/rrbot_controller/constraints/joint1/trajectory", 1.0);
  ASSERT_TRUE(reloadController("rrbot_controller"));

  resetControllerStateObserver();
  ASSERT_TRUE(waitForActionServer(action_client));
  ASSERT_TRUE(waitForInitializedState());

  // Send trajectory (only first segment)
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  traj_goal.trajectory.points.resize(1);
  action_client->sendGoal(traj_goal);
  EXPECT_TRUE(waitForTrajectoryExecution());

  ros::Duration wait_duration = traj_goal.trajectory.points.front().time_from_start * 0.5;
  wait_duration.sleep(); // Wait until half of first segment

  trajectory_msgs::JointTrajectory traj_empty;
  traj_pub.publish(traj_empty);
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client,  SimpleClientGoalState::PREEMPTED));

  EXPECT_TRUE(waitForStop());

  State state = getState();
  if(stop_trajectory_duration < EPS)
  {
    // Here we expect that the desired position is equal to the actual position of the robot,
    // which is given through upper_bound by construction.
    EXPECT_NEAR(state.desired.positions[0], upper_bound.data, JOINT_STATES_COMPARISON_EPS);
  }
  else
  {
    // stop ramp should be calculated using the desired position
    // so it is greater than the upper bound
    EXPECT_GT(state.desired.positions[0], upper_bound.data);
  }

  // Restore perfect control
  {
    std_msgs::Bool delay;
    delay.data = false;
    delay_pub.publish(delay);
    std_msgs::Float64 upper_bound;
    EXPECT_TRUE(waitForRobotReady());
    upper_bound.data = std::numeric_limits<double>::infinity();
    upper_bound_pub.publish(upper_bound);
    EXPECT_TRUE(waitForRobotReady());
  }

  // Restore path tolerance
  ros::param::set("/rrbot_controller/constraints/joint1/trajectory", old_path_tolerance);
  ASSERT_TRUE(reloadController("rrbot_controller"));
}

TEST_F(JointTrajectoryControllerTest, emptyActionCancelsTopicTraj)
{
  // Start state
  State start_state = getState();

  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);
  ASSERT_TRUE(waitForTrajectoryExecution());
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send an empty trajectory goal that preempts the previous one and stops the robot where it is
  ActionGoal empty_goal;
  empty_goal.trajectory.joint_names = traj.joint_names;
  action_client->sendGoal(empty_goal);

  ros::Duration(stop_trajectory_duration).sleep();
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::SUCCEEDED));
  
  EXPECT_TRUE(waitForStop());

  // Check that we're not on the start state
  State current_state = getState();
  EXPECT_FALSE(vectorsAlmostEqual(start_state.actual.positions, current_state.actual.positions))
    << "Failed to move away from start state.";
}

TEST_F(JointTrajectoryControllerTest, emptyActionCancelsActionTraj)
{
  // Start state
  State start_state = getState();

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForTrajectoryExecution());
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send an empty trajectory goal that preempts the previous one and stops the robot where it is
  ActionGoal empty_goal;
  empty_goal.trajectory.joint_names = traj.joint_names;
  action_client2->sendGoal(empty_goal);

  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client,  SimpleClientGoalState::PREEMPTED));

  ros::Duration(stop_trajectory_duration).sleep();
  ASSERT_TRUE(waitForActionResult(action_client2));
  EXPECT_TRUE(checkActionGoalState(action_client2, SimpleClientGoalState::SUCCEEDED));

  EXPECT_TRUE(waitForStop());

  // Check that we're not on the start state
  State current_state = getState();
  EXPECT_FALSE(vectorsAlmostEqual(start_state.actual.positions, current_state.actual.positions))
    << "Failed to move away from start state.";
}

TEST_F(JointTrajectoryControllerTest, cancelActionGoal)
{
  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForTrajectoryExecution());

  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  action_client->cancelGoal();
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::PREEMPTED));

  ros::Duration(stop_trajectory_duration).sleep();
  EXPECT_TRUE(waitForStop());
}

// Ignore old trajectory points ////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, ignoreOldTopicTraj)
{
  // Send trajectory
  traj.header.stamp = ros::Time(0); // Start immediately
  traj_pub.publish(traj);
  ASSERT_TRUE(waitForTrajectoryExecution());
  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory with all points occuring in the past. Should not preempt the previous one
  traj_home.header.stamp = ros::Time::now() - traj_home.points.back().time_from_start;
  traj_pub.publish(traj_home);

  wait_duration = traj.points.back().time_from_start - traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until first trajectory is done

  EXPECT_TRUE(waitForStop());

  // Check that we're at the original trajectory end (NOT back home)
  EXPECT_TRUE(checkPointReached(traj.points.back()));
}

TEST_F(JointTrajectoryControllerTest, ignoreOldActionTraj)
{
  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  ASSERT_TRUE(waitForTrajectoryExecution());

  ros::Duration wait_duration = traj.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory with all points occuring in the past. Should not preempt the previous one
  traj_home_goal.trajectory.header.stamp = ros::Time::now() - traj_home.points.back().time_from_start;
  action_client2->sendGoal(traj_home_goal);
  ASSERT_TRUE(waitForActionResult(action_client2));
  EXPECT_TRUE(checkActionGoalState(action_client2, SimpleClientGoalState::REJECTED));

  // Wait until done
  wait_duration = traj.points.back().time_from_start - traj.points.front().time_from_start;
  wait_duration.sleep(); // remaining execution time

  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::SUCCEEDED));
  
  EXPECT_TRUE(waitForStop());

  // Check that we're at the original trajectory end (NOT back home)
  EXPECT_TRUE(checkPointReached(traj.points.back()));
}

TEST_F(JointTrajectoryControllerTest, ignoreSingleOldActionTraj)
{
  // Send trajectory
  traj_home_goal.trajectory.header.stamp = ros::Time::now() - traj_home.points.back().time_from_start;
  action_client->sendGoal(traj_home_goal);
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::REJECTED));
}

TEST_F(JointTrajectoryControllerTest, ignorePartiallyOldActionTraj)
{
  // Send trajectory
  ActionGoal first_goal = traj_goal;
  first_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  first_goal.trajectory.points.pop_back();           // Remove last point
  action_client->sendGoal(first_goal);
  ASSERT_TRUE(waitForTrajectoryExecution());

  ros::Duration wait_duration = first_goal.trajectory.points.front().time_from_start;
  wait_duration.sleep(); // Wait until ~first waypoint

  // Send another trajectory only partially occuring in the past. The last point should be executed
  traj_goal.trajectory.header.stamp = ros::Time(0);
  traj_goal.trajectory.points.front().time_from_start = ros::Duration(-0.1);
  action_client2->sendGoal(traj_goal);
  EXPECT_TRUE(waitForActionGoalState(action_client2, SimpleClientGoalState::ACTIVE));
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::PREEMPTED));

  // Wait until done
  auto timeout = getTrajectoryDuration(traj_goal.trajectory) + ros::Duration(TIMEOUT_ACTION_RESULT_S);
  ASSERT_TRUE(waitForActionResult(action_client2, timeout));
  EXPECT_TRUE(checkActionGoalState(action_client2, SimpleClientGoalState::SUCCEEDED));

  EXPECT_TRUE(waitForStop()); // Allows values to settle within JOINT_STATES_COMPARISON_EPS, especially accelerations

  // Check that we're at the trajectory end
  EXPECT_TRUE(checkPointReached(traj_goal.trajectory.points.back()));
}

// Velocity FF parameter ///////////////////////////////////////////////////////////////////////////////////////////////
// This test will only be built and run for the VelocityJointInterface-based version of the JointTrajectoryController

#if TEST_VELOCITY_FF

TEST_F(JointTrajectoryControllerTest, jointVelocityFeedForward)
{
  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  EXPECT_TRUE(waitForActionGoalState(action_client, SimpleClientGoalState::ACTIVE));

  // Wait until done
  auto timeout = getTrajectoryDuration(traj_goal.trajectory) + ros::Duration(TIMEOUT_ACTION_RESULT_S);
  ASSERT_TRUE(waitForActionResult(action_client, timeout));
  EXPECT_TRUE(waitForActionGoalState(action_client, SimpleClientGoalState::SUCCEEDED));

  // Go to home configuration, we need known initial conditions
  traj_home_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_home_goal);
  
  auto timeout2 = getTrajectoryDuration(traj_home_goal.trajectory) + ros::Duration(TIMEOUT_ACTION_RESULT_S);
  ASSERT_TRUE(waitForActionResult(action_client, timeout2));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::SUCCEEDED));

  // Disable velocity feedforward
  ros::param::set("/rrbot_controller/velocity_ff/joint1", 0.0);
  ros::param::set("/rrbot_controller/velocity_ff/joint2", 0.0);
  ASSERT_TRUE(reloadController("rrbot_controller"));

  resetControllerStateObserver();
  ASSERT_TRUE(waitForActionServer(action_client));
  ASSERT_TRUE(waitForInitializedState());

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  EXPECT_TRUE(waitForActionGoalState(action_client, SimpleClientGoalState::ACTIVE));

  // Wait until done
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::ABORTED));
  EXPECT_TRUE(checkActionResultErrorCode(action_client,
              control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED));

  // Re-enable velocity feedforward
  ros::param::set("/rrbot_controller/velocity_ff/joint1", 1.0);
  ros::param::set("/rrbot_controller/velocity_ff/joint2", 1.0);
  ASSERT_TRUE(reloadController("rrbot_controller"));
}

#endif // TEST_VELOCITY_FF

// Tolerance checking //////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(JointTrajectoryControllerTest, pathToleranceViolation)
{
  // Make robot respond with a delay
  {
    std_msgs::Float64 smoothing;
    smoothing.data = 0.9;
    smoothing_pub.publish(smoothing);
    ASSERT_TRUE(waitForRobotReady());
  }

  // Send trajectory
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  action_client->sendGoal(traj_goal);
  EXPECT_TRUE(waitForActionGoalState(action_client, SimpleClientGoalState::ACTIVE));

  // Wait until done
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::ABORTED));
  EXPECT_TRUE(checkActionResultErrorCode(action_client,
                                         control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED));

  // Controller continues execution, see https://github.com/ros-controls/ros_controllers/issues/48
  // Make sure to restore an error-less state for the tests to continue

  // Restore perfect control
  {
    std_msgs::Float64 smoothing;
    smoothing.data = 0.0;
    smoothing_pub.publish(smoothing);
    EXPECT_TRUE(waitForRobotReady());
  }

  ros::Duration timeout = getTrajectoryDuration(traj_goal.trajectory) + ros::Duration(TIMEOUT_TRAJ_EXECUTION_S);
  EXPECT_TRUE(waitForStop(timeout));
}

TEST_F(JointTrajectoryControllerTest, pathToleranceViolationSingleJoint)
{
  // Prepare common variables
  std_msgs::Float64MultiArray smoothings;
  smoothings.data.resize(2);
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  const ros::Duration restore_timeout = getTrajectoryDuration(traj_goal.trajectory) + ros::Duration(TIMEOUT_TRAJ_EXECUTION_S);

  /*** JOINT0 ***/
  // Make robot respond with a delay on joint0
  smoothings.data.assign({0.9, 0.});
  smoothings_pub.publish(smoothings);
  ASSERT_TRUE(waitForRobotReady());

  // Send trajectory
  action_client->sendGoal(traj_goal);
  EXPECT_TRUE(waitForActionGoalState(action_client, SimpleClientGoalState::ACTIVE));

  // Wait until done
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::ABORTED));
  EXPECT_TRUE(checkActionResultErrorCode(action_client,
                                         control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED));

  // Controller continues execution, see https://github.com/ros-controls/ros_controllers/issues/48
  // Make sure to restore an error-less state for the tests to continue

  // Restore perfect control
  smoothings.data.assign({0., 0.});
  smoothings_pub.publish(smoothings);
  ASSERT_TRUE(waitForRobotReady());
  EXPECT_TRUE(waitForStop(restore_timeout));

  /*** JOINT1 ***/
    // Make robot respond with a delay on joint1
  smoothings.data.assign({0., 0.9});
  smoothings_pub.publish(smoothings);
  ASSERT_TRUE(waitForRobotReady());

  // Send trajectory
  action_client->sendGoal(traj_goal);
  EXPECT_TRUE(waitForActionGoalState(action_client, SimpleClientGoalState::ACTIVE));

  // Wait until done
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::ABORTED));
  EXPECT_TRUE(checkActionResultErrorCode(action_client,
                                         control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED));

  // Controller continues execution, see https://github.com/ros-controls/ros_controllers/issues/48
  // Make sure to restore an error-less state for the tests to continue

  // Restore perfect control
  smoothings.data.assign({0., 0.});
  smoothings_pub.publish(smoothings);
  ASSERT_TRUE(waitForRobotReady());
  EXPECT_TRUE(waitForStop(restore_timeout));
}

TEST_F(JointTrajectoryControllerTest, goalToleranceViolation)
{
  // Make robot respond with a delay
  {
    std_msgs::Float64 smoothing;
    smoothing.data = 0.95;
    smoothing_pub.publish(smoothing);
    ASSERT_TRUE(waitForRobotReady());
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
  EXPECT_TRUE(waitForActionGoalState(action_client, SimpleClientGoalState::ACTIVE));

  // Wait until done
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::ABORTED));
  EXPECT_TRUE(checkActionResultErrorCode(action_client,
                                         control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED));

  // Controller continues execution, see https://github.com/ros-controls/ros_controllers/issues/48
  // Make sure to restore an error-less state for the tests to continue

  // Restore perfect control
  {
    std_msgs::Float64 smoothing;
    smoothing.data = 0.0;
    smoothing_pub.publish(smoothing);
    EXPECT_TRUE(waitForRobotReady());
  }

  ros::Duration timeout = getTrajectoryDuration(traj_goal.trajectory) + ros::Duration(TIMEOUT_TRAJ_EXECUTION_S);
  EXPECT_TRUE(waitForStop(timeout));
}

TEST_F(JointTrajectoryControllerTest, goalToleranceViolationSingleJoint)
{
  // PREPARE COMMON VARIABLES
  std_msgs::Float64MultiArray smoothings;
  smoothings.data.resize(2);
  traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
  const ros::Duration restore_timeout = getTrajectoryDuration(traj_goal.trajectory) + ros::Duration(TIMEOUT_TRAJ_EXECUTION_S);

  // Disable path constraints
  traj_goal.path_tolerance.resize(2);
  traj_goal.path_tolerance[0].name     = "joint1";
  traj_goal.path_tolerance[0].position = -1.0;
  traj_goal.path_tolerance[0].velocity = -1.0;
  traj_goal.path_tolerance[1].name     = "joint2";
  traj_goal.path_tolerance[1].position = -1.0;
  traj_goal.path_tolerance[1].velocity = -1.0;

  /*** JOINT0 ***/
  // Make robot respond with a delay on joint0
  smoothings.data.assign({0.97, 0.});
  smoothings_pub.publish(smoothings);
  ASSERT_TRUE(waitForRobotReady());

  // Send trajectory
  action_client->sendGoal(traj_goal);
  EXPECT_TRUE(waitForActionGoalState(action_client, SimpleClientGoalState::ACTIVE));

  // Wait until done
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::ABORTED));
  EXPECT_TRUE(checkActionResultErrorCode(action_client,
                                         control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED));

  // Controller continues execution, see https://github.com/ros-controls/ros_controllers/issues/48
  // Make sure to restore an error-less state for the tests to continue

  // Restore perfect control
  smoothings.data.assign({0., 0.});
  smoothings_pub.publish(smoothings);
  ASSERT_TRUE(waitForRobotReady());
  EXPECT_TRUE(waitForStop(restore_timeout));

  /*** JOINT1 ***/
  // Make robot respond with a delay on joint1
  smoothings.data.assign({0., 0.95});
  smoothings_pub.publish(smoothings);
  ASSERT_TRUE(waitForRobotReady());

  // Send trajectory
  action_client->sendGoal(traj_goal);
  EXPECT_TRUE(waitForActionGoalState(action_client, SimpleClientGoalState::ACTIVE));

  // Wait until done
  ASSERT_TRUE(waitForActionResult(action_client));
  EXPECT_TRUE(checkActionGoalState(action_client, SimpleClientGoalState::ABORTED));
  EXPECT_TRUE(checkActionResultErrorCode(action_client,
                                         control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED));

  // Controller continues execution, see https://github.com/ros-controls/ros_controllers/issues/48
  // Make sure to restore an error-less state for the tests to continue

  // Restore perfect control
  smoothings.data.assign({0., 0.});
  smoothings_pub.publish(smoothings);
  ASSERT_TRUE(waitForRobotReady());
  EXPECT_TRUE(waitForStop(restore_timeout));
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
