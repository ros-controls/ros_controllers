///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
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

/// \author Adolfo Rodriguez Tsouroukdissian, Stuart Glaser

#pragma once


// C++ standard
#include <cassert>
#include <stdexcept>
#include <string>
#include <memory>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/dynamic_bitset.hpp>

// ROS
#include <ros/node_handle.h>

// URDF
#include <urdf/model.h>

// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>

// actionlib
#include <actionlib/server/action_server.h>

// realtime_tools
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>

// Project
#include <trajectory_interface/trajectory_interface.h>

#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <joint_trajectory_controller/hold_trajectory_builder.h>
#include <joint_trajectory_controller/stop_trajectory_builder.h>

namespace joint_trajectory_controller
{

/**
 * \brief Controller for executing joint-space trajectories on a group of joints.
 *
 * \note Non-developer documentation and usage instructions can be found in the package's ROS wiki page.
 *
 * \tparam SegmentImpl Trajectory segment representation to use. The type must comply with the following structure:
 * \code
 * class FooSegment
 * {
 * public:
 *   // Required types
 *   typedef double                 Scalar; // Scalar can be anything convertible to double
 *   typedef Scalar                 Time;
 *   typedef PosVelAccState<Scalar> State;
 *
 *   // Default constructor
 *   FooSegment();
 *
 *   // Constructor from start and end states (boundary conditions)
 *   FooSegment(const Time&  start_time,
 *              const State& start_state,
 *              const Time&  end_time,
 *              const State& end_state);
 *
 *   // Start and end states initializer (the guts of the above constructor)
 *   // May throw std::invalid_argument if parameters are invalid
 *   void init(const Time&  start_time,
 *             const State& start_state,
 *             const Time&  end_time,
 *             const State& end_state);
 *
 *   // Sampler (realtime-safe)
 *   void sample(const Time& time, State& state) const;
 *
 *   // Accesors (realtime-safe)
 *   Time startTime()    const;
 *   Time endTime()      const;
 *   unsigned int size() const;
 * };
 * \endcode
 *
 * \tparam HardwareInterface Controller hardware interface. Currently \p hardware_interface::PositionJointInterface,
 * \p hardware_interface::VelocityJointInterface, and \p hardware_interface::EffortJointInterface are supported
 * out-of-the-box.
 */
template <class SegmentImpl, class HardwareInterface>
class JointTrajectoryController : public controller_interface::Controller<HardwareInterface>
{
public:

  JointTrajectoryController();

  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  /*\}*/

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Holds the current position. */
  void starting(const ros::Time& time);

  /** \brief Cancels the active action goal, if any. */
  void stopping(const ros::Time& /*time*/);

  void update(const ros::Time& time, const ros::Duration& period);
  /*\}*/

protected:

  struct TimeData
  {
    TimeData() : time(0.0), period(0.0), uptime(0.0) {}

    ros::Time     time;   ///< Time of last update cycle
    ros::Duration period; ///< Period of last update cycle
    ros::Time     uptime; ///< Controller uptime. Set to zero at every restart.
  };

  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
  typedef std::shared_ptr<ActionServer>                                                       ActionServerPtr;
  typedef ActionServer::GoalHandle                                                            GoalHandle;
  typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
  typedef trajectory_msgs::JointTrajectory::ConstPtr                                          JointTrajectoryConstPtr;
  typedef realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>     StatePublisher;
  typedef std::unique_ptr<StatePublisher>                                                     StatePublisherPtr;

  typedef JointTrajectorySegment<SegmentImpl> Segment;
  typedef std::vector<Segment> TrajectoryPerJoint;
  typedef std::vector<TrajectoryPerJoint> Trajectory;
  typedef std::shared_ptr<Trajectory> TrajectoryPtr;
  typedef std::shared_ptr<TrajectoryPerJoint> TrajectoryPerJointPtr;
  typedef realtime_tools::RealtimeBox<TrajectoryPtr> TrajectoryBox;
  typedef typename Segment::Scalar Scalar;

  typedef HardwareInterfaceAdapter<HardwareInterface, typename Segment::State> HwIfaceAdapter;
  typedef typename HardwareInterface::ResourceHandleType JointHandle;

  bool                      verbose_;            ///< Hard coded verbose flag to help in debugging
  std::string               name_;               ///< Controller name.
  std::vector<JointHandle>  joints_;             ///< Handles to controlled joints.
  std::vector<bool>         angle_wraparound_;   ///< Whether controlled joints wrap around or not.
  std::vector<std::string>  joint_names_;        ///< Controlled joint names.
  SegmentTolerances<Scalar> default_tolerances_; ///< Default trajectory segment tolerances.
  HwIfaceAdapter            hw_iface_adapter_;   ///< Adapts desired trajectory state to HW interface.

  RealtimeGoalHandlePtr     rt_active_goal_;     ///< Currently active action goal, if any.

  /**
   * Thread-safe container with a smart pointer to trajectory currently being followed.
   * Can be either a hold trajectory or a trajectory received from a ROS message.
   *
   * We store the hold trajectory in a separate class member because the \p starting(time) method must be realtime-safe.
   * The (single segment) hold trajectory is preallocated at initialization time and its size is kept unchanged.
   */
  TrajectoryBox curr_trajectory_box_;
  TrajectoryPtr hold_trajectory_ptr_; ///< Last hold trajectory values.

  typename Segment::State current_state_;         ///< Preallocated workspace variable.
  typename Segment::State desired_state_;         ///< Preallocated workspace variable.
  typename Segment::State old_desired_state_;     ///< Preallocated workspace variable.
  typename Segment::State state_error_;           ///< Preallocated workspace variable.
  typename Segment::State desired_joint_state_;   ///< Preallocated workspace variable.
  typename Segment::State state_joint_error_;     ///< Preallocated workspace variable.

  std::unique_ptr<TrajectoryBuilder<SegmentImpl> > hold_traj_builder_;

  realtime_tools::RealtimeBuffer<TimeData> time_data_;
  TimeData old_time_data_;

  ros::Duration state_publisher_period_;
  ros::Duration action_monitor_period_;

  typename Segment::Time stop_trajectory_duration_;  ///< Duration for stop ramp. If zero, the controller stops at the actual position.
  boost::dynamic_bitset<> successful_joint_traj_;
  bool allow_partial_joints_goal_;

  // ROS API
  ros::NodeHandle    controller_nh_;
  ros::Subscriber    trajectory_command_sub_;
  ActionServerPtr    action_server_;
  ros::ServiceServer query_state_service_;
  StatePublisherPtr  state_publisher_;

  ros::Timer         goal_handle_timer_;
  ros::Time          last_state_publish_time_;

  virtual bool updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string = nullptr);
  virtual void trajectoryCommandCB(const JointTrajectoryConstPtr& msg);
  virtual void goalCB(GoalHandle gh);
  virtual void cancelCB(GoalHandle gh);
  virtual void preemptActiveGoal();
  virtual bool queryStateService(control_msgs::QueryTrajectoryState::Request&  req,
                                 control_msgs::QueryTrajectoryState::Response& resp);

  /**
   * \brief Publish current controller state at a throttled frequency.
   * \note This method is realtime-safe and is meant to be called from \ref update, as it shares data with it without
   * any locking.
   */
  void publishState(const ros::Time& time);

  /**
   * \brief Hold the current position.
   *
   * Substitutes the current trajectory with a single-segment one going from the current position and velocity to
   * zero velocity.
   * \see parameter stop_trajectory_duration
   * \note This method is realtime-safe.
   */
  void setHoldPosition(const ros::Time& time, RealtimeGoalHandlePtr gh=RealtimeGoalHandlePtr());

protected:
  /**
   * @brief Returns the number of joints of the robot.
   */
  unsigned int getNumberOfJoints() const;

  /**
   * @brief Updates the states by sampling the specified trajectory for each joint
   * at the specified sampling time.
   *
   * The current state is updated based on the values transmitted by the
   * corresponding JointHandles.
   *
   * @param sample_time Time point at which the joint trajectories have to be sampled.
   * @param traj Trajectory containing all joint trajectories currently under execution.
   *
   * @note This function is NOT thread safe but intended to be used in the
   * update-function.
   */
  void updateStates(const ros::Time& sample_time, const Trajectory* const traj);

protected:
  /**
   * @brief Returns a trajectory consisting of joint trajectories with one pre-allocated
   * segment.
   */
  static TrajectoryPtr createHoldTrajectory(const unsigned int& number_of_joints);

private:
  /**
   * @brief Allows derived classes to perform additional checks
   * and to e.g. replace the newly calculated desired value before
   * the hardware interface is finally updated.
   *
   * @param curr_traj The trajectory that is executed during the current update.
   * @param time_data Updated time data.
   */
  virtual void updateFuncExtensionPoint(const Trajectory& curr_traj, const TimeData& time_data);

private:
  /**
   * @brief Updates the pre-allocated feedback of the current active goal (if any)
   * based on the current state values.
   *
   * @note This function is NOT thread safe but intended to be used in the
   * update-function.
   */
  void setActionFeedback();

};

} // namespace

#include <joint_trajectory_controller/joint_trajectory_controller_impl.h>
