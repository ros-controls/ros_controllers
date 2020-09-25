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


namespace joint_trajectory_controller
{

namespace internal
{

template <class Enclosure, class Member>
inline boost::shared_ptr<Member> share_member(boost::shared_ptr<Enclosure> enclosure, Member &member)
{
  actionlib::EnclosureDeleter<Enclosure> d(enclosure);
  boost::shared_ptr<Member> p(&member, d);
  return p;
}

std::vector<std::string> getStrings(const ros::NodeHandle& nh, const std::string& param_name)
{
  using namespace XmlRpc;
  XmlRpcValue xml_array;
  if (!nh.getParam(param_name, xml_array))
  {
    ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
    return std::vector<std::string>();
  }
  if (xml_array.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("The '" << param_name << "' parameter is not an array (namespace: " <<
                     nh.getNamespace() << ").");
    return std::vector<std::string>();
  }

  std::vector<std::string> out;
  for (int i = 0; i < xml_array.size(); ++i)
  {
    XmlRpc::XmlRpcValue& elem = xml_array[i];
    if (elem.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("The '" << param_name << "' parameter contains a non-string element (namespace: " <<
                       nh.getNamespace() << ").");
      return std::vector<std::string>();
    }
    out.push_back(static_cast<std::string>(elem));
  }
  return out;
}

urdf::ModelSharedPtr getUrdf(const ros::NodeHandle& nh, const std::string& param_name)
{
  urdf::ModelSharedPtr urdf(new urdf::Model);

  std::string urdf_str;
  // Check for robot_description in proper namespace
  if (nh.getParam(param_name, urdf_str))
  {
    if (!urdf->initString(urdf_str))
    {
      ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter (namespace: " <<
        nh.getNamespace() << ").");
      return urdf::ModelSharedPtr();
    }
  }
  // Check for robot_description in root
  else if (!urdf->initParam("robot_description"))
  {
    ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter");
    return urdf::ModelSharedPtr();
  }
  return urdf;
}

std::vector<urdf::JointConstSharedPtr> getUrdfJoints(const urdf::Model& urdf, const std::vector<std::string>& joint_names)
{
  std::vector<urdf::JointConstSharedPtr> out;
  for (const auto& joint_name : joint_names)
  {
    urdf::JointConstSharedPtr urdf_joint = urdf.getJoint(joint_name);
    if (urdf_joint)
    {
      out.push_back(urdf_joint);
    }
    else
    {
      ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in URDF model.");
      return std::vector<urdf::JointConstSharedPtr>();
    }
  }
  return out;
}

std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

} // namespace

template <class SegmentImpl, class HardwareInterface>
inline void JointTrajectoryController<SegmentImpl, HardwareInterface>::
starting(const ros::Time& time)
{
  // Update time data
  TimeData time_data;
  time_data.time   = time;
  time_data.uptime = ros::Time(0.0);
  time_data_.initRT(time_data);

  // Initialize the desired_state with the current state on startup
  for (unsigned int i = 0; i < getNumberOfJoints(); ++i)
  {
    desired_state_.position[i] = joints_[i].getPosition();
    desired_state_.velocity[i] = joints_[i].getVelocity();
  }

  // Hold current position
  setHoldPosition(time_data.uptime);

  // Initialize last state update time
  last_state_publish_time_ = time_data.uptime;

  // Hardware interface adapter
  hw_iface_adapter_.starting(time_data.uptime);
}

template <class SegmentImpl, class HardwareInterface>
inline void JointTrajectoryController<SegmentImpl, HardwareInterface>::
stopping(const ros::Time& /*time*/)
{
  preemptActiveGoal();
}

template <class SegmentImpl, class HardwareInterface>
inline void JointTrajectoryController<SegmentImpl, HardwareInterface>::
trajectoryCommandCB(const JointTrajectoryConstPtr& msg)
{
  const bool update_ok = updateTrajectoryCommand(msg, RealtimeGoalHandlePtr());
  if (update_ok) {preemptActiveGoal();}
}

template <class SegmentImpl, class HardwareInterface>
inline void JointTrajectoryController<SegmentImpl, HardwareInterface>::
preemptActiveGoal()
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Cancels the currently active goal
  if (current_active_goal)
  {
    // Marks the current goal as canceled
    rt_active_goal_.reset();
    current_active_goal->gh_.setCanceled();
  }
}

template <class SegmentImpl, class HardwareInterface>
JointTrajectoryController<SegmentImpl, HardwareInterface>::
JointTrajectoryController()
  : verbose_(false) // Set to true during debugging
{
  // The verbose parameter is for advanced use as it breaks real-time safety
  // by enabling ROS logging services
  if (verbose_)
  {
    ROS_WARN_STREAM(
        "The joint_trajectory_controller verbose flag is enabled. "
        << "This flag breaks real-time safety and should only be "
        << "used for debugging");
  }
}

template <class SegmentImpl, class HardwareInterface>
bool JointTrajectoryController<SegmentImpl, HardwareInterface>::init(HardwareInterface* hw,
                                                                     ros::NodeHandle&   root_nh,
                                                                     ros::NodeHandle&   controller_nh)
{
  using namespace internal;

  // Cache controller node handle
  controller_nh_ = controller_nh;

  // Controller name
  name_ = getLeafNamespace(controller_nh_);

  // State publish rate
  double state_publish_rate = 50.0;
  controller_nh_.getParam("state_publish_rate", state_publish_rate);
  ROS_DEBUG_STREAM_NAMED(name_, "Controller state will be published at " << state_publish_rate << "Hz.");
  state_publisher_period_ = ros::Duration(1.0 / state_publish_rate);

  // Action status checking update rate
  double action_monitor_rate = 20.0;
  controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
  action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
  ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");

  // Stop trajectory duration
  stop_trajectory_duration_ = 0.0;
  controller_nh_.getParam("stop_trajectory_duration", stop_trajectory_duration_);
  ROS_DEBUG_STREAM_NAMED(name_, "Stop trajectory has a duration of " << stop_trajectory_duration_ << "s.");

  // Checking if partial trajectories are allowed
  controller_nh_.param<bool>("allow_partial_joints_goal", allow_partial_joints_goal_, false);
  if (allow_partial_joints_goal_)
  {
    ROS_DEBUG_NAMED(name_, "Goals with partial set of joints are allowed");
  }

  // List of controlled joints
  joint_names_ = getStrings(controller_nh_, "joints");
  if (joint_names_.empty()) {return false;}
  const unsigned int n_joints = joint_names_.size();

  // URDF joints
  urdf::ModelSharedPtr urdf = getUrdf(root_nh, "robot_description");
  if (!urdf) {return false;}

  std::vector<urdf::JointConstSharedPtr> urdf_joints = getUrdfJoints(*urdf, joint_names_);
  if (urdf_joints.empty()) {return false;}
  assert(n_joints == urdf_joints.size());

  // Initialize members
  joints_.resize(n_joints);
  angle_wraparound_.resize(n_joints);
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    // Joint handle
    try {joints_[i] = hw->getHandle(joint_names_[i]);}
    catch (...)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << joint_names_[i] << "' in '" <<
                                    this->getHardwareInterfaceType() << "'.");
      return false;
    }

    // Whether a joint is continuous (ie. has angle wraparound)
    angle_wraparound_[i] = urdf_joints[i]->type == urdf::Joint::CONTINUOUS;
    const std::string not_if = angle_wraparound_[i] ? "" : "non-";

    ROS_DEBUG_STREAM_NAMED(name_, "Found " << not_if << "continuous joint '" << joint_names_[i] << "' in '" <<
                                  this->getHardwareInterfaceType() << "'.");
  }

  assert(joints_.size() == angle_wraparound_.size());
  ROS_DEBUG_STREAM_NAMED(name_, "Initialized controller '" << name_ << "' with:" <<
                         "\n- Number of joints: " << getNumberOfJoints() <<
                         "\n- Hardware interface type: '" << this->getHardwareInterfaceType() << "'" <<
                         "\n- Trajectory segment type: '" << hardware_interface::internal::demangledTypeName<SegmentImpl>() << "'");

  // Default tolerances
  ros::NodeHandle tol_nh(controller_nh_, "constraints");
  default_tolerances_ = getSegmentTolerances<Scalar>(tol_nh, joint_names_);

  // Hardware interface adapter
  hw_iface_adapter_.init(joints_, controller_nh_);

  // ROS API: Subscribed topics
  trajectory_command_sub_ = controller_nh_.subscribe("command", 1, &JointTrajectoryController::trajectoryCommandCB, this);

  // ROS API: Published topics
  state_publisher_.reset(new StatePublisher(controller_nh_, "state", 1));

  // ROS API: Action interface
  action_server_.reset(new ActionServer(controller_nh_, "follow_joint_trajectory",
                                        boost::bind(&JointTrajectoryController::goalCB,   this, _1),
                                        boost::bind(&JointTrajectoryController::cancelCB, this, _1),
                                        false));
  action_server_->start();

  // ROS API: Provided services
  query_state_service_ = controller_nh_.advertiseService("query_state",
                                                         &JointTrajectoryController::queryStateService,
                                                         this);

  // Preeallocate resources
  current_state_       = typename Segment::State(n_joints);
  old_desired_state_   = typename Segment::State(n_joints);
  desired_state_       = typename Segment::State(n_joints);
  state_error_         = typename Segment::State(n_joints);
  desired_joint_state_ = typename Segment::State(1);
  state_joint_error_   = typename Segment::State(1);

  successful_joint_traj_ = boost::dynamic_bitset<>(getNumberOfJoints());

  hold_trajectory_ptr_ = createHoldTrajectory(n_joints);
  assert(joint_names_.size() == hold_trajectory_ptr_->size());

  if (stop_trajectory_duration_ == 0.0)
  {
    hold_traj_builder_ = std::unique_ptr<TrajectoryBuilder<SegmentImpl> >(new HoldTrajectoryBuilder<SegmentImpl, HardwareInterface>(joints_));
  }
  else
  {
    hold_traj_builder_ = std::unique_ptr<TrajectoryBuilder<SegmentImpl> >(new StopTrajectoryBuilder<SegmentImpl>(stop_trajectory_duration_, desired_state_));
  }

  {
    state_publisher_->lock();
    state_publisher_->msg_.joint_names = joint_names_;
    state_publisher_->msg_.desired.positions.resize(n_joints);
    state_publisher_->msg_.desired.velocities.resize(n_joints);
    state_publisher_->msg_.desired.accelerations.resize(n_joints);
    state_publisher_->msg_.actual.positions.resize(n_joints);
    state_publisher_->msg_.actual.velocities.resize(n_joints);
    state_publisher_->msg_.error.positions.resize(n_joints);
    state_publisher_->msg_.error.velocities.resize(n_joints);
    state_publisher_->unlock();
  }

  return true;
}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryController<SegmentImpl, HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  // Get currently followed trajectory
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);
  Trajectory& curr_traj = *curr_traj_ptr;

  old_time_data_ = *(time_data_.readFromRT());

  // Update time data
  TimeData time_data;
  time_data.time   = time;                                     // Cache current time
  time_data.period = period;                                   // Cache current control period
  time_data.uptime = old_time_data_.uptime + period; // Update controller uptime
  time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!

  // NOTE: It is very important to execute the two above code blocks in the specified sequence: first get current
  // trajectory, then update time data. Hopefully the following paragraph sheds a bit of light on the rationale.
  // The non-rt thread responsible for processing new commands enqueues trajectories that can start at the _next_
  // control cycle (eg. zero start time) or later (eg. when we explicitly request a start time in the future).
  // If we reverse the order of the two blocks above, and update the time data first; it's possible that by the time we
  // fetch the currently followed trajectory, it has been updated by the non-rt thread with something that starts in the
  // next control cycle, leaving the current cycle without a valid trajectory.

  updateStates(time_data.uptime, curr_traj_ptr.get());

  // Update current state and state error
  for (unsigned int i = 0; i < getNumberOfJoints(); ++i)
  {
    typename TrajectoryPerJoint::const_iterator segment_it = sample(curr_traj[i], time_data.uptime.toSec(), desired_joint_state_);
    if (curr_traj[i].end() == segment_it)
    {
      // Non-realtime safe, but should never happen under normal operation
      ROS_ERROR_NAMED(name_,
                      "Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
      return;
    }

    // Get state error for current joint
    state_joint_error_.position[0] = state_error_.position[i];
    state_joint_error_.velocity[0] = state_error_.velocity[i];
    state_joint_error_.acceleration[0] = state_error_.acceleration[i];

    //Check tolerances
    const RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
    if (rt_segment_goal && rt_segment_goal == rt_active_goal_)
    {
      // Check tolerances
      if (time_data.uptime.toSec() < segment_it->endTime())
      {
        // Currently executing a segment: check path tolerances
        const SegmentTolerancesPerJoint<Scalar>& joint_tolerances = segment_it->getTolerances();
        if (!checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance))
        {
          if (verbose_)
          {
            ROS_ERROR_STREAM_NAMED(name_,"Path tolerances failed for joint: " << joint_names_[i]);
            checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance, true);
          }
          rt_segment_goal->preallocated_result_->error_code =
          control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
          rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
          rt_active_goal_.reset();
          successful_joint_traj_.reset();
        }
      }
      else if (segment_it == --curr_traj[i].end())
      {
        if (verbose_)
          ROS_DEBUG_STREAM_THROTTLE_NAMED(1,name_,"Finished executing last segment, checking goal tolerances");

        // Controller uptime
        const ros::Time uptime = time_data_.readFromRT()->uptime;

        // Checks that we have ended inside the goal tolerances
        const SegmentTolerancesPerJoint<Scalar>& tolerances = segment_it->getTolerances();
        const bool inside_goal_tolerances = checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance);

        if (inside_goal_tolerances)
        {
          successful_joint_traj_[i] = 1;
        }
        else if (uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance)
        {
          // Still have some time left to meet the goal state tolerances
        }
        else
        {
          if (verbose_)
          {
            ROS_ERROR_STREAM_NAMED(name_,"Goal tolerances failed for joint: "<< joint_names_[i]);
            // Check the tolerances one more time to output the errors that occurs
            checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance, true);
          }

          rt_segment_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
          rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
          rt_active_goal_.reset();
          successful_joint_traj_.reset();
        }
      }
    }
  }

  //If there is an active goal and all segments finished successfully then set goal as succeeded
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  if (current_active_goal && successful_joint_traj_.count() == getNumberOfJoints())
  {
    current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
    current_active_goal.reset(); // do not publish feedback
    rt_active_goal_.reset();
    successful_joint_traj_.reset();
  }

  updateFuncExtensionPoint(curr_traj, time_data);

  // Hardware interface adapter: Generate and send commands
  hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period,
                                  desired_state_, state_error_);

  setActionFeedback();

  publishState(time_data.uptime);
}

template <class SegmentImpl, class HardwareInterface>
bool JointTrajectoryController<SegmentImpl, HardwareInterface>::
updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string)
{
  typedef InitJointTrajectoryOptions<Trajectory> Options;
  Options options;
  options.error_string              = error_string;
  std::string error_string_tmp;

  // Preconditions
  if (!this->isRunning())
  {
    error_string_tmp = "Can't accept new commands. Controller is not running.";
    ROS_ERROR_STREAM_NAMED(name_, error_string_tmp);
    options.setErrorString(error_string_tmp);
    return false;
  }

  if (!msg)
  {
    error_string_tmp = "Received null-pointer trajectory message, skipping.";
    ROS_WARN_STREAM_NAMED(name_, error_string_tmp);
    options.setErrorString(error_string_tmp);
    return false;
  }

  // Time data
  TimeData* time_data = time_data_.readFromRT(); // TODO: Grrr, we need a lock-free data structure here!

  // Time of the next update
  const ros::Time next_update_time = time_data->time + time_data->period;

  // Uptime of the next update
  ros::Time next_update_uptime = time_data->uptime + time_data->period;

  // Hold current position if trajectory is empty
  if (msg->points.empty())
  {
    setHoldPosition(time_data->uptime, gh);
    ROS_DEBUG_NAMED(name_, "Empty trajectory command, stopping.");
    return true;
  }

  // Trajectory initialization options
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);

  options.other_time_base           = &next_update_uptime;
  options.current_trajectory        = curr_traj_ptr.get();
  options.joint_names               = &joint_names_;
  options.angle_wraparound          = &angle_wraparound_;
  options.rt_goal_handle            = gh;
  options.default_tolerances        = &default_tolerances_;
  options.allow_partial_joints_goal = allow_partial_joints_goal_;

  // Update currently executing trajectory
  try
  {
    TrajectoryPtr traj_ptr(new Trajectory);
    *traj_ptr = initJointTrajectory<Trajectory>(*msg, next_update_time, options);
    if (!traj_ptr->empty())
    {
      curr_trajectory_box_.set(traj_ptr);
    }
    else
    {
      return false;
    }
  }
  catch(const std::exception& ex)
  {
    ROS_ERROR_STREAM_NAMED(name_, ex.what());
    options.setErrorString(ex.what());
    return false;
  }
  catch(...)
  {
    error_string_tmp = "Unexpected exception caught when initializing trajectory from ROS message data.";
    ROS_ERROR_STREAM_NAMED(name_, error_string_tmp);
    options.setErrorString(error_string_tmp);
    return false;
  }

  return true;
}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryController<SegmentImpl, HardwareInterface>::
goalCB(GoalHandle gh)
{
  ROS_DEBUG_STREAM_NAMED(name_,"Received new action goal");

  // Precondition: Running controller
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(name_, "Can't accept new action goals. Controller is not running.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL; // TODO: Add better error status to msg?
    gh.setRejected(result);
    return;
  }

  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!allow_partial_joints_goal_)
  {
    if (gh.getGoal()->trajectory.joint_names.size() != joint_names_.size())
    {
      ROS_ERROR_NAMED(name_, "Joints on incoming goal don't match the controller joints.");
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(result);
      return;
    }
  }

  // Goal should specify valid controller joints (they can be ordered differently). Reject if this is not the case
  using internal::mapping;
  std::vector<unsigned int> mapping_vector = mapping(gh.getGoal()->trajectory.joint_names, joint_names_);

  if (mapping_vector.empty())
  {
    ROS_ERROR_NAMED(name_, "Joints on incoming goal don't match the controller joints.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    gh.setRejected(result);
    return;
  }

  // Try to update new trajectory
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
  std::string error_string;
  const bool update_ok = updateTrajectoryCommand(internal::share_member(gh.getGoal(), gh.getGoal()->trajectory),
                                                 rt_goal,
                                                 &error_string);
  rt_goal->preallocated_feedback_->joint_names = joint_names_;

  if (update_ok)
  {
    // Accept new goal
    preemptActiveGoal();
    gh.setAccepted();
    rt_active_goal_ = rt_goal;

    // Setup goal status checking timer
    goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_,
                                                    &RealtimeGoalHandle::runNonRealtime,
                                                    rt_goal);
    goal_handle_timer_.start();
  }
  else
  {
    // Reject invalid goal
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = error_string;
    gh.setRejected(result);
  }
}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryController<SegmentImpl, HardwareInterface>::
cancelCB(GoalHandle gh)
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Check that cancel request refers to currently active goal (if any)
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    // Reset current goal
    rt_active_goal_.reset();

    // Controller uptime
    const ros::Time uptime = time_data_.readFromRT()->uptime;

    // Enter hold current position mode
    setHoldPosition(uptime);
    ROS_DEBUG_NAMED(name_, "Canceling active action goal because cancel callback recieved from actionlib.");

    // Mark the current goal as canceled
    current_active_goal->gh_.setCanceled();
  }
}

template <class SegmentImpl, class HardwareInterface>
bool JointTrajectoryController<SegmentImpl, HardwareInterface>::
queryStateService(control_msgs::QueryTrajectoryState::Request&  req,
                  control_msgs::QueryTrajectoryState::Response& resp)
{
  // Preconditions
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(name_, "Can't sample trajectory. Controller is not running.");
    return false;
  }

  // Convert request time to internal monotonic representation
  TimeData* time_data = time_data_.readFromRT();
  const ros::Duration time_offset = req.time - time_data->time;
  const ros::Time sample_time = time_data->uptime + time_offset;

  // Sample trajectory at requested time
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);
  Trajectory& curr_traj = *curr_traj_ptr;

  typename Segment::State response_point = typename Segment::State(joint_names_.size());

  for (unsigned int i = 0; i < getNumberOfJoints(); ++i)
  {
    typename Segment::State state;
    typename TrajectoryPerJoint::const_iterator segment_it = sample(curr_traj[i], sample_time.toSec(), state);
    if (curr_traj[i].end() == segment_it)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Requested sample time precedes trajectory start time.");
      return false;
    }

    response_point.position[i]     = state.position[0];
    response_point.velocity[i]     = state.velocity[0];
    response_point.acceleration[i] = state.acceleration[0];
  }

  // Populate response
  resp.name         = joint_names_;
  resp.position     = response_point.position;
  resp.velocity     = response_point.velocity;
  resp.acceleration = response_point.acceleration;

  return true;
}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryController<SegmentImpl, HardwareInterface>::
publishState(const ros::Time& time)
{
  // Check if it's time to publish
  if (!state_publisher_period_.isZero() && last_state_publish_time_ + state_publisher_period_ < time)
  {
    if (state_publisher_ && state_publisher_->trylock())
    {
      last_state_publish_time_ += state_publisher_period_;

      state_publisher_->msg_.header.stamp          = time_data_.readFromRT()->time;
      state_publisher_->msg_.desired.positions     = desired_state_.position;
      state_publisher_->msg_.desired.velocities    = desired_state_.velocity;
      state_publisher_->msg_.desired.accelerations = desired_state_.acceleration;
      state_publisher_->msg_.actual.positions      = current_state_.position;
      state_publisher_->msg_.actual.velocities     = current_state_.velocity;
      state_publisher_->msg_.error.positions       = state_error_.position;
      state_publisher_->msg_.error.velocities      = state_error_.velocity;

      state_publisher_->unlockAndPublish();
    }
  }
}

template <class SegmentImpl, class HardwareInterface>
inline void JointTrajectoryController<SegmentImpl, HardwareInterface>::
setHoldPosition(const ros::Time& time, RealtimeGoalHandlePtr gh)
{
  hold_traj_builder_
      ->setStartTime(time.toSec())
      ->setGoalHandle(gh)
      ->buildTrajectory(hold_trajectory_ptr_.get());
  hold_traj_builder_->reset();
  curr_trajectory_box_.set(hold_trajectory_ptr_);
}

template <class SegmentImpl, class HardwareInterface>
inline void JointTrajectoryController<SegmentImpl, HardwareInterface>::
updateFuncExtensionPoint(const Trajectory& curr_traj, const TimeData& time_data)
{
  // To be implemented by derived class
}

template <class SegmentImpl, class HardwareInterface>
inline unsigned int JointTrajectoryController<SegmentImpl, HardwareInterface>::
getNumberOfJoints() const
{
  return joints_.size();
}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryController<SegmentImpl, HardwareInterface>::
updateStates(const ros::Time& sample_time, const Trajectory* const traj)
{
  old_desired_state_ = desired_state_;

  for (unsigned int joint_index = 0; joint_index < getNumberOfJoints(); ++joint_index)
  {
    sample( (*traj)[joint_index], sample_time.toSec(), desired_joint_state_);

    current_state_.position[joint_index] = joints_[joint_index].getPosition();
    current_state_.velocity[joint_index] = joints_[joint_index].getVelocity();
    // There's no acceleration data available in a joint handle

    desired_state_.position[joint_index] = desired_joint_state_.position[0];
    desired_state_.velocity[joint_index] = desired_joint_state_.velocity[0];
    desired_state_.acceleration[joint_index] = desired_joint_state_.acceleration[0];

    state_error_.position[joint_index] = angles::shortest_angular_distance(current_state_.position[joint_index],desired_joint_state_.position[0]);
    state_error_.velocity[joint_index] = desired_joint_state_.velocity[0] - current_state_.velocity[joint_index];
    state_error_.acceleration[joint_index] = 0.0;
  }
}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryController<SegmentImpl, HardwareInterface>::
setActionFeedback()
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  if (!current_active_goal)
  {
    return;
  }

  current_active_goal->preallocated_feedback_->header.stamp          = time_data_.readFromRT()->time;
  current_active_goal->preallocated_feedback_->desired.positions     = desired_state_.position;
  current_active_goal->preallocated_feedback_->desired.velocities    = desired_state_.velocity;
  current_active_goal->preallocated_feedback_->desired.accelerations = desired_state_.acceleration;
  current_active_goal->preallocated_feedback_->actual.positions      = current_state_.position;
  current_active_goal->preallocated_feedback_->actual.velocities     = current_state_.velocity;
  current_active_goal->preallocated_feedback_->error.positions       = state_error_.position;
  current_active_goal->preallocated_feedback_->error.velocities      = state_error_.velocity;
  current_active_goal->setFeedback( current_active_goal->preallocated_feedback_ );

}

template <class SegmentImpl, class HardwareInterface>
typename JointTrajectoryController<SegmentImpl, HardwareInterface>::TrajectoryPtr
JointTrajectoryController<SegmentImpl, HardwareInterface>::createHoldTrajectory(const unsigned int& number_of_joints)
{
  TrajectoryPtr hold_traj {new Trajectory()};

  typename Segment::State default_state       = typename Segment::State(number_of_joints);
  typename Segment::State default_joint_state = typename Segment::State(1);
  for (unsigned int i = 0; i < number_of_joints; ++i)
  {
    default_joint_state.position[0]= default_state.position[i];
    default_joint_state.velocity[0]= default_state.velocity[i];
    Segment hold_segment(0.0, default_joint_state, 0.0, default_joint_state);

    TrajectoryPerJoint joint_segment;
    joint_segment.resize(1, hold_segment);
    hold_traj->push_back(joint_segment);
  }

  return hold_traj;
}

} // namespace
