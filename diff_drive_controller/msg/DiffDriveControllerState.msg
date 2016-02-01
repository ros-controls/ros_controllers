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

# Control period:
float64 control_period_desired
float64 control_period_actual
float64 control_period_error

# Control time (of the actual code):
# Note that the resolution of the wall time is approx. 0.5us, but the user and
# system is only 10ms. See:
# http://www.boost.org/doc/libs/1_48_0/libs/timer/doc/cpu_timers.html#Resolution
#
# For this reason, and given that the control time is much less than 10ms,
# the user and system time aren't accurate; note that we can't take an average
# of multiple cycles because we still have to call stop at the end of the cycle,
# so the internal count of the timer would still be wrong.
# Consequently, ignore the user and system time, unless the wall time becomes
# greater than 10ms.
float64 control_time_wall
float64 control_time_user
float64 control_time_system
