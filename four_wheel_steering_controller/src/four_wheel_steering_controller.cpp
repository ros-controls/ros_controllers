/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Irstea
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Irstea nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <cmath>
#include <four_wheel_steering_controller/four_wheel_steering_controller.h>
#include <tf/transform_datatypes.h>
#include <urdf_geometry_parser/urdf_geometry_parser.h>

namespace four_wheel_steering_controller{

  FourWheelSteeringController::FourWheelSteeringController()
    : command_struct_twist_()
    , command_struct_four_wheel_steering_()
    , track_(0.0)
    , wheel_steering_y_offset_(0.0)
    , wheel_radius_(0.0)
    , wheel_base_(0.0)
    , cmd_vel_timeout_(0.5)
    , base_frame_id_("base_link")
    , enable_odom_tf_(true)
    , enable_twist_cmd_(false)
  {
  }

  bool FourWheelSteeringController::init(hardware_interface::RobotHW *robot_hw,
                                         ros::NodeHandle& root_nh,
                                         ros::NodeHandle &controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Get joint names from the parameter server
    std::vector<std::string> front_wheel_names, rear_wheel_names;
    if (!getWheelNames(controller_nh, "front_wheel", front_wheel_names) ||
        !getWheelNames(controller_nh, "rear_wheel", rear_wheel_names))
    {
      return false;
    }

    if (front_wheel_names.size() != rear_wheel_names.size())
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#front wheels (" << front_wheel_names.size() << ") != " <<
          "#rear wheels (" << rear_wheel_names.size() << ").");
      return false;
    }
    else if (front_wheel_names.size() != 2)
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#two wheels by axle (left and right) is needed; now : "<<front_wheel_names.size()<<" .");
      return false;
    }
    else
    {
      front_wheel_joints_.resize(front_wheel_names.size());
      rear_wheel_joints_.resize(front_wheel_names.size());
    }

    // Get steering joint names from the parameter server
    std::vector<std::string> front_steering_names, rear_steering_names;
    if (!getWheelNames(controller_nh, "front_steering", front_steering_names) ||
        !getWheelNames(controller_nh, "rear_steering", rear_steering_names))
    {
      return false;
    }

    if (front_steering_names.size() != rear_steering_names.size())
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#left steerings (" << front_steering_names.size() << ") != " <<
          "#right steerings (" << rear_steering_names.size() << ").");
      return false;
    }
    else if (front_steering_names.size() != 2)
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#two steering by axle (left and right) is needed; now : "<<front_steering_names.size()<<" .");
      return false;
    }
    else
    {
      front_steering_joints_.resize(front_steering_names.size());
      rear_steering_joints_.resize(front_steering_names.size());
    }

    // Odometry related:
    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                          << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);

    controller_nh.param("open_loop", open_loop_, open_loop_);

    int velocity_rolling_window_size = 10;
    controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
    ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of "
                          << velocity_rolling_window_size << ".");

    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    // Twist command related:
    controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
    ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                          << cmd_vel_timeout_ << "s.");

    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

    controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
    ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

    // Velocity and acceleration limits:
    controller_nh.param("linear/x/has_velocity_limits"    , limiter_lin_.has_velocity_limits    , limiter_lin_.has_velocity_limits    );
    controller_nh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
    controller_nh.param("linear/x/max_velocity"           , limiter_lin_.max_velocity           ,  limiter_lin_.max_velocity          );
    controller_nh.param("linear/x/min_velocity"           , limiter_lin_.min_velocity           , -limiter_lin_.max_velocity          );
    controller_nh.param("linear/x/max_acceleration"       , limiter_lin_.max_acceleration       ,  limiter_lin_.max_acceleration      );
    controller_nh.param("linear/x/min_acceleration"       , limiter_lin_.min_acceleration       , -limiter_lin_.max_acceleration      );

    controller_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
    controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
    controller_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );

    // If either parameter is not available, we need to look up the value in the URDF
    bool lookup_track = !controller_nh.getParam("track", track_);
    bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", wheel_radius_);
    bool lookup_wheel_base = !controller_nh.getParam("wheel_base", wheel_base_);

    urdf_geometry_parser::UrdfGeometryParser uvk(root_nh, base_frame_id_);
    if(lookup_track)
      if(!uvk.getDistanceBetweenJoints(front_wheel_names[0], front_wheel_names[1], track_))
        return false;
      else
        controller_nh.setParam("track",track_);

    if(!uvk.getDistanceBetweenJoints(front_steering_names[0], front_wheel_names[0], wheel_steering_y_offset_))
      return false;
    else
      controller_nh.setParam("wheel_steering_y_offset",wheel_steering_y_offset_);

    if(lookup_wheel_radius)
      if(!uvk.getJointRadius(front_wheel_names[0], wheel_radius_))
        return false;
      else
        controller_nh.setParam("wheel_radius",wheel_radius_);

    if(lookup_wheel_base)
      if(!uvk.getDistanceBetweenJoints(front_wheel_names[0], rear_wheel_names[0], wheel_base_))
        return false;
      else
        controller_nh.setParam("wheel_base",wheel_base_);

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    odometry_.setWheelParams(track_-2*wheel_steering_y_offset_, wheel_steering_y_offset_, wheel_radius_, wheel_base_);
    ROS_INFO_STREAM_NAMED(name_,
                          "Odometry params : track " << track_
                          << ", wheel radius " << wheel_radius_
                          << ", wheel base " << wheel_base_
                          << ", wheel steering offset " << wheel_steering_y_offset_);

    setOdomPubFields(root_nh, controller_nh);


    hardware_interface::VelocityJointInterface *const vel_joint_hw = robot_hw->get<hardware_interface::VelocityJointInterface>();
    hardware_interface::PositionJointInterface *const pos_joint_hw = robot_hw->get<hardware_interface::PositionJointInterface>();

    // Get the joint object to use in the realtime loop
    for (size_t i = 0; i < front_wheel_joints_.size(); ++i)
    {
      ROS_INFO_STREAM_NAMED(name_,
                            "Adding front wheel with joint name: " << front_wheel_names[i]
                            << " and rear wheel with joint name: " << rear_wheel_names[i]);
      front_wheel_joints_[i] = vel_joint_hw->getHandle(front_wheel_names[i]);  // throws on failure
      rear_wheel_joints_[i] = vel_joint_hw->getHandle(rear_wheel_names[i]);  // throws on failure
    }

    // Get the steering joint object to use in the realtime loop
    for (size_t i = 0; i < front_steering_joints_.size(); ++i)
    {
      ROS_INFO_STREAM_NAMED(name_,
                            "Adding left steering with joint name: " << front_steering_names[i]
                            << " and right steering with joint name: " << rear_steering_names[i]);
      front_steering_joints_[i] = pos_joint_hw->getHandle(front_steering_names[i]);  // throws on failure
      rear_steering_joints_[i] = pos_joint_hw->getHandle(rear_steering_names[i]);  // throws on failure
    }

    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &FourWheelSteeringController::cmdVelCallback, this);
    sub_command_four_wheel_steering_ = controller_nh.subscribe("cmd_four_wheel_steering", 1, &FourWheelSteeringController::cmdFourWheelSteeringCallback, this);

    return true;
  }

  void FourWheelSteeringController::update(const ros::Time& time, const ros::Duration& period)
  {
    updateOdometry(time);
    updateCommand(time, period);
  }

  void FourWheelSteeringController::starting(const ros::Time& time)
  {
    brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;

    odometry_.init(time);
  }

  void FourWheelSteeringController::stopping(const ros::Time& /*time*/)
  {
    brake();
  }

  void FourWheelSteeringController::updateOdometry(const ros::Time& time)
  {
    // COMPUTE AND PUBLISH ODOMETRY
    const double fl_speed = front_wheel_joints_[0].getVelocity();
    const double fr_speed = front_wheel_joints_[1].getVelocity();
    const double rl_speed = rear_wheel_joints_[0].getVelocity();
    const double rr_speed = rear_wheel_joints_[1].getVelocity();
    if (std::isnan(fl_speed) || std::isnan(fr_speed)
        || std::isnan(rl_speed) || std::isnan(rr_speed))
      return;

    const double fl_steering = front_steering_joints_[0].getPosition();
    const double fr_steering = front_steering_joints_[1].getPosition();
    const double rl_steering = rear_steering_joints_[0].getPosition();
    const double rr_steering = rear_steering_joints_[1].getPosition();
    if (std::isnan(fl_steering) || std::isnan(fr_steering)
        || std::isnan(rl_steering) || std::isnan(rr_steering))
      return;
    double front_steering_pos = 0.0;
    if(fabs(fl_steering) > 0.001 || fabs(fr_steering) > 0.001)
    {
      front_steering_pos = atan(2*tan(fl_steering)*tan(fr_steering)/
                                      (tan(fl_steering) + tan(fr_steering)));
    }
    double rear_steering_pos = 0.0;
    if(fabs(rl_steering) > 0.001 || fabs(rr_steering) > 0.001)
    {
      rear_steering_pos = atan(2*tan(rl_steering)*tan(rr_steering)/
                                     (tan(rl_steering) + tan(rr_steering)));
    }

    ROS_DEBUG_STREAM_THROTTLE(1, "rl_steering "<<rl_steering<<" rr_steering "<<rr_steering<<" rear_steering_pos "<<rear_steering_pos);
    // Estimate linear and angular velocity using joint information
    odometry_.update(fl_speed, fr_speed, rl_speed, rr_speed,
                     front_steering_pos, rear_steering_pos, time);

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
      last_state_publish_time_ += publish_period_;
      // Compute and store orientation info
      const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

      // Populate odom message and publish
      if (odom_pub_->trylock())
      {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
        odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinearX();
        odom_pub_->msg_.twist.twist.linear.y  = odometry_.getLinearY();
        odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
        odom_pub_->unlockAndPublish();
      }
      if (odom_4ws_pub_->trylock())
      {
        odom_4ws_pub_->msg_.header.stamp = time;
        odom_4ws_pub_->msg_.data.speed = odometry_.getLinear();
        odom_4ws_pub_->msg_.data.acceleration = odometry_.getLinearAcceleration();
        odom_4ws_pub_->msg_.data.jerk = odometry_.getLinearJerk();
        odom_4ws_pub_->msg_.data.front_steering_angle = front_steering_pos;
        odom_4ws_pub_->msg_.data.front_steering_angle_velocity = odometry_.getFrontSteerVel();
        odom_4ws_pub_->msg_.data.rear_steering_angle = rear_steering_pos;
        odom_4ws_pub_->msg_.data.rear_steering_angle_velocity = odometry_.getRearSteerVel();
        odom_4ws_pub_->unlockAndPublish();
      }

      // Publish tf /odom frame
      if (enable_odom_tf_ && tf_odom_pub_->trylock())
      {
        geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->unlockAndPublish();
      }
    }
  }

  void FourWheelSteeringController::updateCommand(const ros::Time& time, const ros::Duration& period)
  {
    // Retreive current velocity command and time step:
    Command* cmd;
    CommandTwist curr_cmd_twist = *(command_twist_.readFromRT());
    Command4ws curr_cmd_4ws = *(command_four_wheel_steering_.readFromRT());

    if(curr_cmd_4ws.stamp >= curr_cmd_twist.stamp)
    {
      cmd = &curr_cmd_4ws;
      enable_twist_cmd_ = false;
    }
    else
    {
      cmd = &curr_cmd_twist;
      enable_twist_cmd_ = true;
    }

    const double dt = (time - cmd->stamp).toSec();
    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd_twist.lin_x = 0.0;
      curr_cmd_twist.lin_y = 0.0;
      curr_cmd_twist.ang = 0.0;
      curr_cmd_4ws.lin = 0.0;
      curr_cmd_4ws.front_steering = 0.0;
      curr_cmd_4ws.rear_steering = 0.0;
    }

    const double cmd_dt(period.toSec());

    const double angular_speed = odometry_.getAngular();
    const double steering_track = track_-2*wheel_steering_y_offset_;

    ROS_DEBUG_STREAM("angular_speed "<<angular_speed<< " wheel_radius_ "<<wheel_radius_);
    double vel_left_front = 0, vel_right_front = 0;
    double vel_left_rear = 0, vel_right_rear = 0;
    double front_left_steering = 0, front_right_steering = 0;
    double rear_left_steering = 0, rear_right_steering = 0;

    if(enable_twist_cmd_ == true)
    {
      // Limit velocities and accelerations:
      limiter_lin_.limit(curr_cmd_twist.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x, cmd_dt);
      limiter_ang_.limit(curr_cmd_twist.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);
      last1_cmd_ = last0_cmd_;
      last0_cmd_ = curr_cmd_twist;

      // Compute wheels velocities:
      if(fabs(curr_cmd_twist.lin_x) > 0.001)
      {
        const double vel_steering_offset = (curr_cmd_twist.ang*wheel_steering_y_offset_)/wheel_radius_;
        const double sign = copysign(1.0, curr_cmd_twist.lin_x);
        vel_left_front  = sign * std::hypot((curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track/2),
                                            (wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_
                          - vel_steering_offset;
        vel_right_front = sign * std::hypot((curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track/2),
                                            (wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_
                          + vel_steering_offset;
        vel_left_rear = sign * std::hypot((curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track/2),
                                          (wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_
                        - vel_steering_offset;
        vel_right_rear = sign * std::hypot((curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track/2),
                                           (wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_
                         + vel_steering_offset;
      }

      // Compute steering angles
      if(fabs(2.0*curr_cmd_twist.lin_x) > fabs(curr_cmd_twist.ang*steering_track))
      {
        front_left_steering = atan(curr_cmd_twist.ang*wheel_base_ /
                                    (2.0*curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track));
        front_right_steering = atan(curr_cmd_twist.ang*wheel_base_ /
                                     (2.0*curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track));
      }
      else if(fabs(curr_cmd_twist.lin_x) > 0.001)
      {
        front_left_steering = copysign(M_PI_2, curr_cmd_twist.ang);
        front_right_steering = copysign(M_PI_2, curr_cmd_twist.ang);
      }
      rear_left_steering = -front_left_steering;
      rear_right_steering = -front_right_steering;
    }
    else
    {
      // Limit velocities and accelerations:
      limiter_lin_.limit(curr_cmd_4ws.lin, last0_cmd_.lin_x, last1_cmd_.lin_x, cmd_dt);
      last1_cmd_ = last0_cmd_;
      last0_cmd_.lin_x = curr_cmd_4ws.lin;
      curr_cmd_4ws.front_steering = clamp(curr_cmd_4ws.front_steering, -M_PI_2, M_PI_2);
      curr_cmd_4ws.rear_steering = clamp(curr_cmd_4ws.rear_steering, -M_PI_2, M_PI_2);

      // Compute steering angles
      const double tan_front_steering = tan(curr_cmd_4ws.front_steering);
      const double tan_rear_steering  = tan(curr_cmd_4ws.rear_steering);

      const double steering_diff =  steering_track*(tan_front_steering - tan_rear_steering)/2.0;
      if(fabs(wheel_base_ - fabs(steering_diff)) > 0.001)
      {
        front_left_steering = atan(wheel_base_*tan_front_steering/(wheel_base_-steering_diff));
        front_right_steering = atan(wheel_base_*tan_front_steering/(wheel_base_+steering_diff));
        rear_left_steering = atan(wheel_base_*tan_rear_steering/(wheel_base_-steering_diff));
        rear_right_steering = atan(wheel_base_*tan_rear_steering/(wheel_base_+steering_diff));
      }

      // Compute wheels velocities:
      if(fabs(curr_cmd_4ws.lin) > 0.001)
      {
        //Virutal front and rear wheelbase
        // distance between the projection of the CIR on the wheelbase and the front axle
        double l_front = 0;
        if(fabs(tan(front_left_steering) - tan(front_right_steering)) > 0.01)
        {
          l_front = tan(front_right_steering) * tan(front_left_steering) * steering_track
              / (tan(front_left_steering) - tan(front_right_steering));
        }
        // distance between the projection of the CIR on the wheelbase and the rear axle
        double l_rear = 0;
        if(fabs(tan(rear_left_steering) - tan(rear_right_steering)) > 0.01)
        {
          l_rear = tan(rear_right_steering) * tan(rear_left_steering) * steering_track
              / (tan(rear_left_steering) - tan(rear_right_steering));
        }

        const double angular_speed_cmd = curr_cmd_4ws.lin * (tan_front_steering-tan_rear_steering)/wheel_base_;
        const double vel_steering_offset = (angular_speed_cmd*wheel_steering_y_offset_)/wheel_radius_;
        const double sign = copysign(1.0, curr_cmd_4ws.lin);

        vel_left_front  = sign * std::hypot((curr_cmd_4ws.lin - angular_speed_cmd*steering_track/2),
                                            (l_front*angular_speed_cmd))/wheel_radius_
                          - vel_steering_offset;
        vel_right_front = sign * std::hypot((curr_cmd_4ws.lin + angular_speed_cmd*steering_track/2),
                                            (l_front*angular_speed_cmd))/wheel_radius_
                          + vel_steering_offset;
        vel_left_rear = sign * std::hypot((curr_cmd_4ws.lin - angular_speed_cmd*steering_track/2),
                                          (l_rear*angular_speed_cmd))/wheel_radius_
                        - vel_steering_offset;
        vel_right_rear = sign * std::hypot((curr_cmd_4ws.lin + angular_speed_cmd*steering_track/2),
                                           (l_rear*angular_speed_cmd))/wheel_radius_
                         + vel_steering_offset;
      }
    }

    ROS_DEBUG_STREAM_THROTTLE(1, "vel_left_rear "<<vel_left_rear<<" front_right_steering "<<front_right_steering);
    // Set wheels velocities:
    if(front_wheel_joints_.size() == 2 && rear_wheel_joints_.size() == 2)
    {
      front_wheel_joints_[0].setCommand(vel_left_front);
      front_wheel_joints_[1].setCommand(vel_right_front);
      rear_wheel_joints_[0].setCommand(vel_left_rear);
      rear_wheel_joints_[1].setCommand(vel_right_rear);
    }

    /// TODO check limits to not apply the same steering on right and left when saturated !
    if(front_steering_joints_.size() == 2 && rear_steering_joints_.size() == 2)
    {
      front_steering_joints_[0].setCommand(front_left_steering);
      front_steering_joints_[1].setCommand(front_right_steering);
      rear_steering_joints_[0].setCommand(rear_left_steering);
      rear_steering_joints_[1].setCommand(rear_right_steering);
    }
  }

  void FourWheelSteeringController::brake()
  {
    const double vel = 0.0;
    for (size_t i = 0; i < front_wheel_joints_.size(); ++i)
    {
      front_wheel_joints_[i].setCommand(vel);
      rear_wheel_joints_[i].setCommand(vel);
    }

    const double pos = 0.0;
    for (size_t i = 0; i < front_steering_joints_.size(); ++i)
    {
      front_steering_joints_[i].setCommand(pos);
      rear_steering_joints_[i].setCommand(pos);
    }
  }

  void FourWheelSteeringController::cmdVelCallback(const geometry_msgs::Twist& command)
  {
    if (isRunning())
    {
      if(std::isnan(command.angular.z) || std::isnan(command.linear.x))
      {
        ROS_WARN("Received NaN in geometry_msgs::Twist. Ignoring command.");
        return;
      }
      command_struct_twist_.ang   = command.angular.z;
      command_struct_twist_.lin_x   = command.linear.x;
      command_struct_twist_.lin_y   = command.linear.y;
      command_struct_twist_.stamp = ros::Time::now();
      command_twist_.writeFromNonRT (command_struct_twist_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Ang: "   << command_struct_twist_.ang << ", "
                             << "Lin x: " << command_struct_twist_.lin_x << ", "
                             << "Lin y: " << command_struct_twist_.lin_y << ", "
                             << "Stamp: " << command_struct_twist_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

  void FourWheelSteeringController::cmdFourWheelSteeringCallback(const four_wheel_steering_msgs::FourWheelSteering& command)
  {
    if (isRunning())
    {
      if(std::isnan(command.front_steering_angle) || std::isnan(command.rear_steering_angle)
         || std::isnan(command.speed))
      {
        ROS_WARN("Received NaN in four_wheel_steering_msgs::FourWheelSteering. Ignoring command.");
        return;
      }
      command_struct_four_wheel_steering_.front_steering   = command.front_steering_angle;
      command_struct_four_wheel_steering_.rear_steering   = command.rear_steering_angle;
      command_struct_four_wheel_steering_.lin   = command.speed;
      command_struct_four_wheel_steering_.stamp = ros::Time::now();
      command_four_wheel_steering_.writeFromNonRT (command_struct_four_wheel_steering_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Steering front : "   << command_struct_four_wheel_steering_.front_steering << ", "
                             << "Steering rear : "   << command_struct_four_wheel_steering_.rear_steering << ", "
                             << "Lin: "   << command_struct_four_wheel_steering_.lin << ", "
                             << "Stamp: " << command_struct_four_wheel_steering_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

  bool FourWheelSteeringController::getWheelNames(ros::NodeHandle& controller_nh,
                              const std::string& wheel_param,
                              std::vector<std::string>& wheel_names)
  {
      XmlRpc::XmlRpcValue wheel_list;
      if (!controller_nh.getParam(wheel_param, wheel_list))
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Couldn't retrieve wheel param '" << wheel_param << "'.");
        return false;
      }

      if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        if (wheel_list.size() == 0)
        {
          ROS_ERROR_STREAM_NAMED(name_,
              "Wheel param '" << wheel_param << "' is an empty list");
          return false;
        }

        for (int i = 0; i < wheel_list.size(); ++i)
        {
          if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
          {
            ROS_ERROR_STREAM_NAMED(name_,
                "Wheel param '" << wheel_param << "' #" << i <<
                " isn't a string.");
            return false;
          }
        }

        wheel_names.resize(wheel_list.size());
        for (int i = 0; i < wheel_list.size(); ++i)
        {
          wheel_names[i] = static_cast<std::string>(wheel_list[i]);
          //ROS_INFO_STREAM("wheel name "<<i<<" " << wheel_names[i]);
        }
      }
      else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        wheel_names.push_back(wheel_list);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Wheel param '" << wheel_param <<
            "' is neither a list of strings nor a string.");
        return false;
      }
      return true;
  }

  void FourWheelSteeringController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
      ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = {
        static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5]) };
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = {
        static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5]) };
    odom_4ws_pub_.reset(new realtime_tools::RealtimePublisher<four_wheel_steering_msgs::FourWheelSteeringStamped>(controller_nh, "odom_steer", 100));
    odom_4ws_pub_->msg_.header.frame_id = "odom";

    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
  }

} // namespace four_wheel_steering_controller
