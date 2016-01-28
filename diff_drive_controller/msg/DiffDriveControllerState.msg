# This initial part is the same as JointTrajectoryControllerState.msg:
Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint desired
trajectory_msgs/JointTrajectoryPoint actual
trajectory_msgs/JointTrajectoryPoint error  # Redundant, but useful

# Actual wheel joint trajectory controller state estimated from the positions,
# i.e. the velocities are NOT the ones directly reported from the joints, but
# they're differentiated from the positions instead:
trajectory_msgs/JointTrajectoryPoint actual_estimated
trajectory_msgs/JointTrajectoryPoint error_estimated  # Redundant, but useful

# Actual wheel joint trajectory controller state averaged for all the wheel
# joints on each side of the robot, so it can be used on the unicycle model
# odometry equations; this is the actual state used for the odometry
# computation if the wheel joint velocity is NOT estimated, but read from the
# joints:
trajectory_msgs/JointTrajectoryPoint actual_side_average
trajectory_msgs/JointTrajectoryPoint error_side_average

# Actual wheel joint trajectory controller state estimated from the positions
# and averaged for all the wheel joints on each side of the robot, so it can be
# used on the unicycle model odometry equations; this is the actual state used
# for the odometry computation if the wheel joint velocity is NOT read from the
# joints, but estimated:
trajectory_msgs/JointTrajectoryPoint actual_estimated_side_average
trajectory_msgs/JointTrajectoryPoint error_estimated_side_average
