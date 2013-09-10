## Joint Trajectory Controller ##

### Overview and features ###

Controller for executing joint-space trajectories on a group of joints.

- Trajectory segment type
  - Controller is templated on the **segment type**.
  - Multi-dof **quintic spline** segment implementation provided by default.

- Hardware interface type
  - Controller is templated on the **hardware interface type**.
  - **Position** and **effort** control joint interfaces provided by default.

- **Realtime-safe**.

- Proper handling of **wrapping** (continuous) joints.

- **Online trajectory replacement**: New commands are smoothly combined with currently executed trajectory.

- **Robust to system clock changes:** Discontinuous system clock changes do not cause discontinuities in the execution of already queued trajectory segments.

### ROS API ###
  - Commands:
    - [control_msgs::FollowJointTrajectory](http://docs.ros.org/groovy/api/control_msgs/html/action/FollowJointTrajectory.html) action interface.
    - [trajectory_msgs::JointTrajectory](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html) topic interface.
    - Support for the *deprecated* [pr2_controllers_msgs::JointTrajectory](http://docs.ros.org/api/pr2_controllers_msgs/html/msg/JointTrajectoryAction.html) action interface has been dropped.

  - Controller state querries:
    - Current state is available in a [control_msgs::JointTrajectoryControllerState](http://docs.ros.org/api/control_msgs/html/msg/JointTrajectoryControllerState.html) topic.
    - State at any future time can be queried through a [control_msgs::QueryTrajectoryState](http://docs.ros.org/api/control_msgs/html/srv/QueryTrajectoryState.html) service.

### Controller configuration examples ###

#### Minimal description, position interface ####

```yaml
head_controller:
  type: "position_controllers/JointTrajectoryController"
  joints:
    - head_1_joint
    - head_2_joint
```

#### Complete description, effort interface ####

```yaml
head_controller:
  type: "effort_controllers/JointTrajectoryController"
  joints:
    - head_1_joint
    - head_2_joint

  constraints:
    goal_time: 0.5                   # Defaults to zero
    stopped_velocity_tolerance: 0.02 # Defaults to 0.01
    head_1_joint:
      trajectory: 0.05 # Not enforced if unspecified
      goal: 0.02       # Not enforced if unspecified
    head_2_joint:
      goal: 0.01
      
  gains: # Needed to map position+velocity commands to an effort command
    head_1_joint: {p: 100,  d: 1, i: 1, i_clamp: 1}
    head_2_joint: {p: 100,  d: 1, i: 1, i_clamp: 1}

  state_publish_rate:  25 # Defaults to 50
  action_monitor_rate: 10 # Defaults to 20
  hold_trajectory_duration: 0 # Defaults to 0.5
```
