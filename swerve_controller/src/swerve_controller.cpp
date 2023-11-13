/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Exobotic
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
 *   * Neither the name of Exobotic nor the names of its
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

#include <swerve_controller/swerve_controller.h>
#include <string>

namespace swerve_controller
{

    SwerveController::SwerveController()
        : command_struct_twist_(),
          track_(0.0),
          wheel_steering_y_offset_(0.0),
          wheel_radius_(0.0),
          wheel_base_(0.0),
          min_steering_angle_(-M_PI),
          max_steering_angle_(M_PI),
          cmd_vel_timeout_(0.5),
          base_frame_id_("base_link"),
          enable_odom_tf_(true)
    {
    }

    bool SwerveController::init(hardware_interface::RobotHW *robot_hw,
                                ros::NodeHandle &root_nh,
                                ros::NodeHandle &controller_nh)
    {
        // Get namespace
        std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        name_ = complete_ns.substr(id + 1);

        // Get wheel joint names from the parameter server
        std::string lf_wheel_name, rf_wheel_name, lh_wheel_name, rh_wheel_name;
        if (!controller_nh.param("lf_wheel", lf_wheel_name, lf_wheel_name) ||
            !controller_nh.param("rf_wheel", rf_wheel_name, rf_wheel_name) ||
            !controller_nh.param("lh_wheel", lh_wheel_name, lh_wheel_name) ||
            !controller_nh.param("rh_wheel", rh_wheel_name, rh_wheel_name))
        {
            ROS_ERROR_STREAM_NAMED(name_,
                                   "Couldn't retrieve wheel joint params !");
            return false;
        }

        // Get steering joint names from the parameter server
        std::string lf_steering_name, rf_steering_name, lh_steering_name, rh_steering_name;
        if (!controller_nh.param("lf_steering", lf_steering_name, lf_steering_name) ||
            !controller_nh.param("rf_steering", rf_steering_name, rf_steering_name) ||
            !controller_nh.param("lh_steering", lh_steering_name, lh_steering_name) ||
            !controller_nh.param("rh_steering", rh_steering_name, rh_steering_name))
        {
            ROS_ERROR_STREAM_NAMED(name_,
                                   "Couldn't retrieve steering joint params !");
            return false;
        }

        // Get maximal steering angle from the parameter server
        controller_nh.param("min_steering_angle", min_steering_angle_, min_steering_angle_);
        controller_nh.param("max_steering_angle", max_steering_angle_, max_steering_angle_);

        // Get publisher related from the parameter server
        double publish_rate;
        controller_nh.param("publish_rate", publish_rate, 50.0);
        ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                                         << publish_rate << "Hz.");
        publish_period_ = ros::Duration(1.0 / publish_rate);

        // Get twist related from the parameter server
        controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
        ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are "
                                         << "older than " << cmd_vel_timeout_ << "s.");

        controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
        ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

        controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
        ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is "
                                         << (enable_odom_tf_ ? "enabled" : "disabled"));

        // Get velocity and acceleration limits from the parameter server
        controller_nh.param("linear/x/has_velocity_limits",
                            limiter_lin_.has_velocity_limits,
                            limiter_lin_.has_velocity_limits);
        controller_nh.param("linear/x/has_acceleration_limits",
                            limiter_lin_.has_acceleration_limits,
                            limiter_lin_.has_acceleration_limits);
        controller_nh.param("linear/x/max_velocity",
                            limiter_lin_.max_velocity,
                            limiter_lin_.max_velocity);
        controller_nh.param("linear/x/min_velocity",
                            limiter_lin_.min_velocity,
                            -limiter_lin_.max_velocity);
        controller_nh.param("linear/x/max_acceleration",
                            limiter_lin_.max_acceleration,
                            limiter_lin_.max_acceleration);
        controller_nh.param("linear/x/min_acceleration",
                            limiter_lin_.min_acceleration,
                            -limiter_lin_.max_acceleration);

        controller_nh.param("angular/z/has_velocity_limits",
                            limiter_ang_.has_velocity_limits,
                            limiter_ang_.has_velocity_limits);
        controller_nh.param("angular/z/has_acceleration_limits",
                            limiter_ang_.has_acceleration_limits,
                            limiter_ang_.has_acceleration_limits);
        controller_nh.param("angular/z/max_velocity",
                            limiter_ang_.max_velocity,
                            limiter_ang_.max_velocity);
        controller_nh.param("angular/z/min_velocity",
                            limiter_ang_.min_velocity,
                            -limiter_ang_.max_velocity);
        controller_nh.param("angular/z/max_acceleration",
                            limiter_ang_.max_acceleration,
                            limiter_ang_.max_acceleration);
        controller_nh.param("angular/z/min_acceleration",
                            limiter_ang_.min_acceleration,
                            -limiter_ang_.max_acceleration);

        // Get robot physical parameters from URDF or parameter server
        bool lookup_track = !controller_nh.getParam("track", track_);
        bool lookup_wheel_steering_y_offset = !controller_nh.getParam("wheel_steering_y_offset",
                                                                      wheel_steering_y_offset_);
        bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", wheel_radius_);
        bool lookup_wheel_base = !controller_nh.getParam("wheel_base", wheel_base_);

        if (lookup_track || lookup_wheel_steering_y_offset ||
            lookup_wheel_radius || lookup_wheel_base)
        {
            ROS_INFO_STREAM("Some geometric parameters are not provided in the config file."
                            << "Parsing from URDF!");
            urdf_geometry_parser::UrdfGeometryParser uvk(root_nh, base_frame_id_);
            if (lookup_track)
            {
                if (!uvk.getDistanceBetweenJoints(lf_wheel_name, rf_wheel_name, track_))
                    return false;
                else
                    controller_nh.setParam("track", track_);
            }
            if (lookup_wheel_steering_y_offset)
            {
                if (!uvk.getDistanceBetweenJoints(lf_steering_name, lf_wheel_name,
                                                  wheel_steering_y_offset_))
                    return false;
                else
                    controller_nh.setParam("wheel_steering_y_offset", wheel_steering_y_offset_);
            }
            if (lookup_wheel_radius)
            {
                if (!uvk.getJointRadius(lf_wheel_name, wheel_radius_))
                    return false;
                else
                    controller_nh.setParam("wheel_radius", wheel_radius_);
            }
            if (lookup_wheel_base)
            {
                if (!uvk.getDistanceBetweenJoints(lf_wheel_name, lh_wheel_name,
                                                  wheel_base_))
                    return false;
                else
                    controller_nh.setParam("wheel_base", wheel_base_);
            }
        }

        // Set physical parameters in odometry
        odometry_.setWheelParams(track_ - 2 * wheel_steering_y_offset_, wheel_radius_, wheel_base_);
        ROS_INFO_STREAM_NAMED(name_, "Odometry params : track "
                                         << track_ << ", wheel radius "
                                         << wheel_radius_ << ", wheel base " << wheel_base_
                                         << ", wheel steering offset " << wheel_steering_y_offset_);
        setOdomPubFields(root_nh, controller_nh);

        // Get hardware interface
        hardware_interface::VelocityJointInterface *const vel_joint_hw =
            robot_hw->get<hardware_interface::VelocityJointInterface>();
        hardware_interface::PositionJointInterface *const pos_joint_hw =
            robot_hw->get<hardware_interface::PositionJointInterface>();

        // Get the wheel joint object to use in the realtime loop
        ROS_INFO_STREAM_NAMED(name_, "Adding LF wheel with joint name: "
                                         << lf_wheel_name
                                         << ", RF wheel with joint name: " << rf_wheel_name
                                         << ", LH wheel with joint name: " << lh_wheel_name
                                         << ", RH wheel with joint name: " << rh_wheel_name);
        lf_wheel_joint_ = vel_joint_hw->getHandle(lf_wheel_name); // throws on failure
        rf_wheel_joint_ = vel_joint_hw->getHandle(rf_wheel_name); // throws on failure
        lh_wheel_joint_ = vel_joint_hw->getHandle(lh_wheel_name); // throws on failure
        rh_wheel_joint_ = vel_joint_hw->getHandle(rh_wheel_name); // throws on failure

        // Get the steering joint object to use in the realtime loop
        ROS_INFO_STREAM_NAMED(name_, "Adding LF steering with joint name: "
                                         << lf_steering_name
                                         << ", RF steering with joint name: " << rf_steering_name
                                         << ", LH steering with joint name: " << lh_steering_name
                                         << ", RH steering with joint name: " << rh_steering_name);
        lf_steering_joint_ = pos_joint_hw->getHandle(lf_steering_name); // throws on failure
        rf_steering_joint_ = pos_joint_hw->getHandle(rf_steering_name); // throws on failure
        lh_steering_joint_ = pos_joint_hw->getHandle(lh_steering_name); // throws on failure
        rh_steering_joint_ = pos_joint_hw->getHandle(rh_steering_name); // throws on failure

        // Subscribe to Twist messages
        sub_command_ = controller_nh.subscribe("cmd_vel", 1,
                                               &SwerveController::cmdVelCallback, this);
        return true;
    }

    void SwerveController::update(const ros::Time &time, const ros::Duration &period)
    {
        updateOdometry(time);
        updateCommand(time, period);
    }

    void SwerveController::starting(const ros::Time &time)
    {
        brake();

        // Register starting time used to keep fixed rate
        last_state_publish_time_ = time;

        odometry_.init(time);
    }

    void SwerveController::stopping(const ros::Time &time)
    {
        brake();
    }

    void SwerveController::updateOdometry(const ros::Time &time)
    {
        // Get current position and velocities
        const double lf_speed = lf_wheel_joint_->getVelocity();
        const double rf_speed = rf_wheel_joint_->getVelocity();
        const double lh_speed = lh_wheel_joint_->getVelocity();
        const double rh_speed = rh_wheel_joint_->getVelocity();
        if (std::isnan(lf_speed) || std::isnan(rf_speed) ||
            std::isnan(lh_speed) || std::isnan(rh_speed))
            return;

        const double lf_steering = lf_steering_joint_->getPosition();
        const double rf_steering = rf_steering_joint_->getPosition();
        const double lh_steering = lh_steering_joint_->getPosition();
        const double rh_steering = rh_steering_joint_->getPosition();
        if (std::isnan(lf_steering) || std::isnan(rf_steering) ||
            std::isnan(lh_steering) || std::isnan(rh_steering))
            return;

        // Estimate linear and angular velocity using joint information
        odometry_.update(lf_speed, rf_speed, lh_speed, rh_speed,
                         lf_steering, rf_steering, lh_steering, rh_steering, time);

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
                odom_pub_->msg_.twist.twist.linear.x = odometry_.getLinearX();
                odom_pub_->msg_.twist.twist.linear.y = odometry_.getLinearY();
                odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
                odom_pub_->unlockAndPublish();
            }

            // Publish tf /odom frame
            if (enable_odom_tf_ && tf_odom_pub_->trylock())
            {
                geometry_msgs::TransformStamped &odom_frame = tf_odom_pub_->msg_.transforms[0];
                odom_frame.header.stamp = time;
                odom_frame.transform.translation.x = odometry_.getX();
                odom_frame.transform.translation.y = odometry_.getY();
                odom_frame.transform.rotation = orientation;
                tf_odom_pub_->unlockAndPublish();
            }
        }
    }

    void SwerveController::updateCommand(const ros::Time &time, const ros::Duration &period)
    {
        // Retreive current velocity command and time step
        CommandTwist curr_cmd = *(command_twist_.readFromRT());
        const double dt = (time - curr_cmd.stamp).toSec();

        // Brake if cmd_vel has timeout
        if (dt > cmd_vel_timeout_)
        {
            brake();
            return;
        }

        // Create velocities and position variables
        const double cmd_dt(period.toSec());
        const double steering_track = track_ - 2 * wheel_steering_y_offset_;
        double lf_speed = 0, rf_speed = 0, lh_speed = 0, rh_speed = 0;
        double lf_steering = 0, rf_steering = 0, lh_steering = 0, rh_steering = 0;

        // Limit velocities and accelerations:
        limiter_lin_.limit(curr_cmd.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x, cmd_dt);
        limiter_lin_.limit(curr_cmd.lin_y, last0_cmd_.lin_y, last1_cmd_.lin_y, cmd_dt);
        limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);
        last1_cmd_ = last0_cmd_;
        last0_cmd_ = curr_cmd;

        // Compute wheels velocities and steering angles
        if ((fabs(curr_cmd.lin_x) > 0.001) || (fabs(curr_cmd.lin_y) > 0.001) ||
            (fabs(curr_cmd.ang) > 0.001))
        {
            double a = curr_cmd.lin_y - curr_cmd.ang * wheel_base_ / 2;
            double b = curr_cmd.lin_y + curr_cmd.ang * wheel_base_ / 2;
            double c = curr_cmd.lin_x - curr_cmd.ang * steering_track / 2;
            double d = curr_cmd.lin_x + curr_cmd.ang * steering_track / 2;

            lf_speed = sqrt(pow(b, 2) + pow(c, 2)) / wheel_radius_;
            rf_speed = sqrt(pow(b, 2) + pow(d, 2)) / wheel_radius_;
            lh_speed = sqrt(pow(a, 2) + pow(c, 2)) / wheel_radius_;
            rh_speed = sqrt(pow(a, 2) + pow(d, 2)) / wheel_radius_;

            lf_steering = atan2(b, c);
            rf_steering = atan2(b, d);
            lh_steering = atan2(a, c);
            rh_steering = atan2(a, d);
        }

        // Invert wheel speed or brake if steering angle exceeds desired limits
        if (!clipSteeringAngle(lf_steering, lf_speed) ||
            !clipSteeringAngle(rf_steering, rf_speed) ||
            !clipSteeringAngle(lh_steering, lh_speed) ||
            !clipSteeringAngle(rh_steering, rh_speed))
        {
            brake();
            return;
        }

        // Set wheels velocities
        if (lf_wheel_joint_ && rf_wheel_joint_ && lh_wheel_joint_ && rh_wheel_joint_)
        {
            lf_wheel_joint_->setCommand(lf_speed);
            rf_wheel_joint_->setCommand(rf_speed);
            lh_wheel_joint_->setCommand(lh_speed);
            rh_wheel_joint_->setCommand(rh_speed);
        }

        // Set wheels steering angles
        if (lf_steering_joint_ && rf_steering_joint_ && lh_steering_joint_ && rh_steering_joint_)
        {
            lf_steering_joint_->setCommand(lf_steering);
            rf_steering_joint_->setCommand(rf_steering);
            lh_steering_joint_->setCommand(lh_steering);
            rh_steering_joint_->setCommand(rh_steering);
        }
    }

    void SwerveController::brake()
    {
        // Set wheels velocities
        if (lf_wheel_joint_ && rf_wheel_joint_ && lh_wheel_joint_ && rh_wheel_joint_)
        {
            lf_wheel_joint_->setCommand(0.0);
            rf_wheel_joint_->setCommand(0.0);
            lh_wheel_joint_->setCommand(0.0);
            rh_wheel_joint_->setCommand(0.0);
        }
    }

    bool SwerveController::clipSteeringAngle(double &steering, double &speed)
    {
        if (steering > max_steering_angle_)
        {
            if (steering - M_PI > min_steering_angle_)
            {
                steering -= M_PI;
                speed = -speed;
                return true;
            }
            else
            {
                return false;
            }
        }

        if (steering < min_steering_angle_)
        {
            if (steering + M_PI < max_steering_angle_)
            {
                steering += M_PI;
                speed = -speed;
                return true;
            }
            else
            {
                return false;
            }
        }

        return true;
    }

    void SwerveController::cmdVelCallback(const geometry_msgs::Twist &command)
    {
        if (isRunning())
        {
            if (std::isnan(command.angular.z) || std::isnan(command.linear.x))
            {
                ROS_WARN("Received NaN in geometry_msgs::Twist. Ignoring command.");
                return;
            }
            command_struct_twist_.ang = command.angular.z;
            command_struct_twist_.lin_x = command.linear.x;
            command_struct_twist_.lin_y = command.linear.y;
            command_struct_twist_.stamp = ros::Time::now();
            command_twist_.writeFromNonRT(command_struct_twist_);
            ROS_DEBUG_STREAM_NAMED(name_, "Added values to command. "
                                              << "Ang: " << command_struct_twist_.ang
                                              << ", Lin x: " << command_struct_twist_.lin_x
                                              << ", Lin y: " << command_struct_twist_.lin_y
                                              << ", Stamp: " << command_struct_twist_.stamp);
        }
        else
        {
            ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
        }
    }

    void SwerveController::setOdomPubFields(ros::NodeHandle &root_nh,
                                            ros::NodeHandle &controller_nh)
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
        odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh,
                                                                                  "odom", 100));
        odom_pub_->msg_.header.frame_id = "odom";
        odom_pub_->msg_.child_frame_id = base_frame_id_;
        odom_pub_->msg_.pose.pose.position.z = 0;
        odom_pub_->msg_.pose.covariance =
            {static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
             0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
             0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
             0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
             0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
             0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5])};
        odom_pub_->msg_.twist.twist.linear.y = 0;
        odom_pub_->msg_.twist.twist.linear.z = 0;
        odom_pub_->msg_.twist.twist.angular.x = 0;
        odom_pub_->msg_.twist.twist.angular.y = 0;
        odom_pub_->msg_.twist.covariance =
            {static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
             0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
             0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
             0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
             0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
             0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5])};

        tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh,
                                                                                "/tf", 100));
        tf_odom_pub_->msg_.transforms.resize(1);
        tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
        tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
        tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
    }

} // namespace swerve_controller
