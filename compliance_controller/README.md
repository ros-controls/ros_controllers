### Setup for a UR robot

This should be similar for other ROS robots that have velocity_controllers/JointGroupVelocityController and velocity_controllers/JointTrajectoryController available.

### Building

* sudo apt install ros-melodic-ros-contro*

* Clone universal_robot (melodic-devel branch), https://github.com/ros-industrial/universal_robot/tree/melodic-devel

* Clone ur_modern_driver (kinetic-devel branch), https://github.com/ros-industrial/ur_modern_driver

* Clone ros_controllers repo (melodic-devel branch), https://github.com/ros-controls/ros_controllers

* catkin build

### Config

urXX below means ur5 for a UR5 robot, for example.

You may want to save a copy of each file in a new directory, rather than modifying the original.

* Edit the yaml file that defines the MoveIt trajectory follower. It is in /universal_robot/urXX_moveit_config/config/controllers.yaml. It should look like this:

		controller_list:
		  - name: "compliance_controller"
		    action_ns: follow_joint_trajectory
		    type: FollowJointTrajectory
		    joints:
		      - shoulder_pan_joint
		      - shoulder_lift_joint
		      - elbow_joint
		      - wrist_1_joint
		      - wrist_2_joint
		      - wrist_3_joint

* Edit the yaml file that defines the ros_controllers to be launched. It is in /ur_modern_driver/config/urXX_controllers.yaml. Add this:

		compliance_controller:
		   type: velocity_controllers/ComplianceController
		   joints:
		     - shoulder_pan_joint
		     - shoulder_lift_joint
		     - elbow_joint
		     - wrist_1_joint
		     - wrist_2_joint
		     - wrist_3_joint
		   compliance_command_timeout: 1.5
		   constraints:
		      goal_time: 0.6
		      stopped_velocity_tolerance: 0.05
		      shoulder_pan_joint: {trajectory: 0.1, goal: 0.1}
		      shoulder_lift_joint: {trajectory: 0.1, goal: 0.1}
		      elbow_joint: {trajectory: 0.1, goal: 0.1}
		      wrist_1_joint: {trajectory: 0.1, goal: 0.1}
		      wrist_2_joint: {trajectory: 0.1, goal: 0.1}
		      wrist_3_joint: {trajectory: 0.1, goal: 0.1}
		   stop_trajectory_duration: 0.5
		   state_publish_rate:  125
		   action_monitor_rate: 10
		   gains:
		      #!!These values have not been optimized!!
		      shoulder_pan_joint:  {p: 5.0,  i: 0.05, d: 0.1, i_clamp: 1}
		      shoulder_lift_joint: {p: 5.0,  i: 0.05, d: 0.1, i_clamp: 1}
		      elbow_joint:         {p: 5.0,  i: 0.05, d: 0.1, i_clamp: 1}
		      wrist_1_joint:       {p: 5.0,  i: 0.05, d: 0.1, i_clamp: 1}
		      wrist_2_joint:       {p: 5.0,  i: 0.05, d: 0.1, i_clamp: 1}
		      wrist_3_joint:       {p: 5.0,  i: 0.05, d: 0.1, i_clamp: 1}

		   # Use a feedforward term to reduce the size of PID gains
		   velocity_ff:
		    shoulder_pan_joint: 1.0
		    shoulder_lift_joint: 1.0
		    elbow_joint: 1.0
		    wrist_1_joint: 1.0
		    wrist_2_joint: 1.0
		    wrist_3_joint: 1.0

* Edit the ros_control launch file. It is in /ur_modern_driver/launch/urXX_ros_control.launch. Change these lines:

		<arg name="controllers" default="joint_state_controller force_torque_sensor_controller compliance_controller"/>

		<arg name="stopped_controllers" default="pos_based_pos_traj_controller joint_group_vel_controller vel_based_pos_traj_controller"/>

* Check that you can move the robot with ros_control. Launch these files then test a move in Rviz:
		roslaunch ur_modern_driver urXX_ros_control.launch robot_ip:=192.168.1.102
		roslaunch urXX_moveit_config urXX_moveit_planning_execution.launch
		roslaunch urXX_moveit_config moveit_rviz.launch config:=true

* Move the robot to a non-singular configuration (compliance is paused near singularities to avoid large unwanted movements)

* Set up a node which converts the UR force/torque topic to delta-velocity commands and publishes them:

		roslaunch wrench_to_joint_vel_pub ur_compliance.launch

Push on the end-effector and the robot should move with you. You can also execute a trajectory, just like normal.

### Summary

The 4 launch commands are:

		roslaunch ur_modern_driver urXX_ros_control.launch robot_ip:=192.168.1.102
		roslaunch urXX_moveit_config urXX_moveit_planning_execution.launch
		roslaunch urXX_moveit_config moveit_rviz.launch config:=true
		roslaunch wrench_to_joint_vel_pub ur_compliance.launch

The rosservice call to toggle compliance ON/OFF is:

		rosservice call /compliance_controller/toggle_compliance "{}"

It defaults to no compliance.
