/*********************************************************************
 * Software License Agreement (BSD License)
 *
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
 *   * Neither the name of the PAL Robotics nor the names of its
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

/*
 * Author: Bence Magyar
 */

#include <tf/transform_datatypes.h>

#include <urdf_parser/urdf_parser.h>

#include <boost/assign.hpp>

#include <mecanum_drive_controller/mecanum_drive_controller.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static bool isCylinderOrSphere(const boost::shared_ptr<const urdf::Link>& link)
{
  if(!link)
  {
    ROS_ERROR("Link == NULL.");
    return false;
  }

  if(!link->collision)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
    return false;
  }

  if(!link->collision->geometry)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
    return false;
  }

  if(link->collision->geometry->type != urdf::Geometry::CYLINDER && link->collision->geometry->type != urdf::Geometry::SPHERE)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have cylinder nor sphere geometry");
    return false;
  }

  return true;
}

namespace mecanum_drive_controller
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MecanumDriveController::MecanumDriveController()
  : open_loop_(false)
  , command_struct_()
  , use_realigned_roller_joints_(false)
  , wheels_k_(0.0)
  , wheels_radius_(0.0)
  , cmd_vel_timeout_(0.5)
  , base_frame_id_("base_link")
  , enable_odom_tf_(true)
  , wheel_joints_size_(0)
{
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MecanumDriveController::init(hardware_interface::VelocityJointInterface* hw,
          ros::NodeHandle& root_nh,
          ros::NodeHandle &controller_nh)
{
  const std::string complete_ns = controller_nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  name_ = complete_ns.substr(id + 1);

  // Option use_realigned_roller_joints
  controller_nh.param("use_realigned_roller_joints", use_realigned_roller_joints_, use_realigned_roller_joints_);
  ROS_INFO_STREAM_NAMED(name_, "Use the roller's radius rather than the wheel's: " << use_realigned_roller_joints_ << ".");

  // Get joint names from the parameter server
  std::string wheel0_name;
  controller_nh.param("front_left_wheel_joint", wheel0_name, wheel0_name);
  ROS_INFO_STREAM_NAMED(name_, "Front left wheel joint (wheel0) is : " << wheel0_name);

  std::string wheel1_name;
  controller_nh.param("back_left_wheel_joint", wheel1_name, wheel1_name);
  ROS_INFO_STREAM_NAMED(name_, "Back left wheel joint (wheel1) is : " << wheel1_name);

  std::string wheel2_name;
  controller_nh.param("back_right_wheel_joint", wheel2_name, wheel2_name);
  ROS_INFO_STREAM_NAMED(name_, "Back right wheel joint (wheel2) is : " << wheel2_name);

  std::string wheel3_name;
  controller_nh.param("front_right_wheel_joint", wheel3_name, wheel3_name);
  ROS_INFO_STREAM_NAMED(name_, "Front right wheel joint (wheel3) is : " << wheel3_name);

  // Odometry related:
  double publish_rate;
  controller_nh.param("publish_rate", publish_rate, 50.0);
  ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                        << publish_rate << "Hz.");
  publish_period_ = ros::Duration(1.0 / publish_rate);

  controller_nh.param("open_loop", open_loop_, open_loop_);

  // Twist command related:
  controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
  ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                        << cmd_vel_timeout_ << "s.");

  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

  controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
  ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

  // Velocity and acceleration limits:
  controller_nh.param("linear/x/has_velocity_limits"    , limiter_linX_.has_velocity_limits    , limiter_linX_.has_velocity_limits    );
  controller_nh.param("linear/x/has_acceleration_limits", limiter_linX_.has_acceleration_limits, limiter_linX_.has_acceleration_limits);
  controller_nh.param("linear/x/max_velocity"           , limiter_linX_.max_velocity           ,  limiter_linX_.max_velocity          );
  controller_nh.param("linear/x/min_velocity"           , limiter_linX_.min_velocity           , -limiter_linX_.max_velocity          );
  controller_nh.param("linear/x/max_acceleration"       , limiter_linX_.max_acceleration       ,  limiter_linX_.max_acceleration      );
  controller_nh.param("linear/x/min_acceleration"       , limiter_linX_.min_acceleration       , -limiter_linX_.max_acceleration      );

  controller_nh.param("linear/y/has_velocity_limits"    , limiter_linY_.has_velocity_limits    , limiter_linY_.has_velocity_limits    );
  controller_nh.param("linear/y/has_acceleration_limits", limiter_linY_.has_acceleration_limits, limiter_linY_.has_acceleration_limits);
  controller_nh.param("linear/y/max_velocity"           , limiter_linY_.max_velocity           ,  limiter_linY_.max_velocity          );
  controller_nh.param("linear/y/min_velocity"           , limiter_linY_.min_velocity           , -limiter_linY_.max_velocity          );
  controller_nh.param("linear/y/max_acceleration"       , limiter_linY_.max_acceleration       ,  limiter_linY_.max_acceleration      );
  controller_nh.param("linear/y/min_acceleration"       , limiter_linY_.min_acceleration       , -limiter_linY_.max_acceleration      );

  controller_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
  controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
  controller_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
  controller_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
  controller_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
  controller_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );

  // Get the joint objects to use in the realtime loop
  wheel0_jointHandle_ = hw->getHandle(wheel0_name);  // throws on failure
  wheel1_jointHandle_ = hw->getHandle(wheel1_name);  // throws on failure
  wheel2_jointHandle_ = hw->getHandle(wheel2_name);  // throws on failure
  wheel3_jointHandle_ = hw->getHandle(wheel3_name);  // throws on failure

  // Pass params through and setup publishers and subscribers
  if (!setWheelParamsFromUrdf(root_nh, wheel0_name, wheel1_name, wheel2_name, wheel3_name))
    return false;

  setupRtPublishersMsg(root_nh, controller_nh);

  sub_command_ = controller_nh.subscribe("cmd_vel", 1, &MecanumDriveController::cmdVelCallback, this);

  return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MecanumDriveController::update(const ros::Time& time, const ros::Duration& period)
{
  // COMPUTE AND PUBLISH ODOMETRY
  if (open_loop_)
  {
    odometry_.updateOpenLoop(last_cmd_.linX, last_cmd_.linY, last_cmd_.ang, time);
  }
  else
  {
    double wheel0_vel = wheel0_jointHandle_.getVelocity();
    double wheel1_vel = wheel1_jointHandle_.getVelocity();
    double wheel2_vel = wheel2_jointHandle_.getVelocity();
    double wheel3_vel = wheel3_jointHandle_.getVelocity();

    if (std::isnan(wheel0_vel) || std::isnan(wheel1_vel) || std::isnan(wheel2_vel) || std::isnan(wheel3_vel))
      return;

    // Estimate twist (using joint information) and integrate
    odometry_.update(wheel0_vel, wheel1_vel, wheel2_vel, wheel3_vel, time);
  }

  // Publish odometry message
  if(last_state_publish_time_ + publish_period_ < time)
  {
    last_state_publish_time_ += publish_period_;
    // Compute and store orientation info
    const geometry_msgs::Quaternion orientation(
          tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

    // Populate odom message and publish
    if(odom_pub_->trylock())
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

  // MOVE ROBOT
  // Retrieve current velocity command and time step:
  Commands curr_cmd = *(command_.readFromRT());
  const double dt = (time - curr_cmd.stamp).toSec();

  // Brake if cmd_vel has timeout:
  if (dt > cmd_vel_timeout_)
  {
    curr_cmd.linX = 0.0;
    curr_cmd.linY = 0.0;
    curr_cmd.ang = 0.0;
  }

  // Limit velocities and accelerations:
  const double cmd_dt(period.toSec());
  limiter_linX_.limit(curr_cmd.linX, last_cmd_.linX, cmd_dt);
  limiter_linY_.limit(curr_cmd.linY, last_cmd_.linY, cmd_dt);
  limiter_ang_.limit(curr_cmd.ang, last_cmd_.ang, cmd_dt);
  last_cmd_ = curr_cmd;

  // Compute wheels velocities (this is the actual ik):
  // NOTE: the input desired twist (from topic /cmd_vel) is a body twist.
  const double w0_vel = 1.0 / wheels_radius_ * (curr_cmd.linX - curr_cmd.linY - wheels_k_ * curr_cmd.ang);
  const double w1_vel = 1.0 / wheels_radius_ * (curr_cmd.linX + curr_cmd.linY - wheels_k_ * curr_cmd.ang);
  const double w2_vel = 1.0 / wheels_radius_ * (curr_cmd.linX - curr_cmd.linY + wheels_k_ * curr_cmd.ang);
  const double w3_vel = 1.0 / wheels_radius_ * (curr_cmd.linX + curr_cmd.linY + wheels_k_ * curr_cmd.ang);

  // Set wheels velocities:
  wheel0_jointHandle_.setCommand(w0_vel);
  wheel1_jointHandle_.setCommand(w1_vel);
  wheel2_jointHandle_.setCommand(w2_vel);
  wheel3_jointHandle_.setCommand(w3_vel);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MecanumDriveController::starting(const ros::Time& time)
{
  brake();

  // Register starting time used to keep fixed rate
  last_state_publish_time_ = time;

  odometry_.init(time);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MecanumDriveController::stopping(const ros::Time& time)
{
  brake();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MecanumDriveController::brake()
{
  wheel0_jointHandle_.setCommand(0.0);
  wheel1_jointHandle_.setCommand(0.0);
  wheel2_jointHandle_.setCommand(0.0);
  wheel3_jointHandle_.setCommand(0.0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MecanumDriveController::cmdVelCallback(const geometry_msgs::Twist& command)
{
  if(isRunning())
  {
    command_struct_.ang   = command.angular.z;
    command_struct_.linX  = command.linear.x;
    command_struct_.linY  = command.linear.y;
    command_struct_.stamp = ros::Time::now();
    command_.writeFromNonRT (command_struct_);
    ROS_DEBUG_STREAM_NAMED(name_,
                           "Added values to command. "
                           << "Ang: "   << command_struct_.ang << ", "
                           << "Lin: "   << command_struct_.linX << ", "
                           << "Lin: "   << command_struct_.linY << ", "
                           << "Stamp: " << command_struct_.stamp);
  }
  else
  {
    ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MecanumDriveController::setWheelParamsFromUrdf(ros::NodeHandle& root_nh,
                                                   const std::string& wheel0_name,
                                                   const std::string& wheel1_name,
                                                   const std::string& wheel2_name,
                                                   const std::string& wheel3_name)
{
  // Parse robot description
  const std::string model_param_name = "robot_description";
  bool res = root_nh.hasParam(model_param_name);
  std::string robot_model_str="";
  if(!res || !root_nh.getParam(model_param_name,robot_model_str))
  {
    ROS_ERROR_NAMED(name_, "Robot descripion couldn't be retrieved from param server.");
    return false;
  }

  boost::shared_ptr<urdf::ModelInterface> model(urdf::parseURDF(robot_model_str));

  // Get wheels position and compute parameter k_ (used in mecanum wheels IK).
  boost::shared_ptr<const urdf::Joint> wheel0_urdfJoint(model->getJoint(wheel0_name));
  if(!wheel0_urdfJoint)
  {
    ROS_ERROR_STREAM_NAMED(name_, wheel0_name
                           << " couldn't be retrieved from model description");
    return false;
  }
  boost::shared_ptr<const urdf::Joint> wheel1_urdfJoint(model->getJoint(wheel1_name));
  if(!wheel1_urdfJoint)
  {
    ROS_ERROR_STREAM_NAMED(name_, wheel1_name
                           << " couldn't be retrieved from model description");
    return false;
  }
  boost::shared_ptr<const urdf::Joint> wheel2_urdfJoint(model->getJoint(wheel2_name));
  if(!wheel2_urdfJoint)
  {
    ROS_ERROR_STREAM_NAMED(name_, wheel2_name
                           << " couldn't be retrieved from model description");
    return false;
  }
  boost::shared_ptr<const urdf::Joint> wheel3_urdfJoint(model->getJoint(wheel3_name));
  if(!wheel3_urdfJoint)
  {
    ROS_ERROR_STREAM_NAMED(name_, wheel3_name
                           << " couldn't be retrieved from model description");
    return false;
  }

  ROS_INFO_STREAM("wheel0 to origin: "  << wheel0_urdfJoint->parent_to_joint_origin_transform.position.x << ","
                                        << wheel0_urdfJoint->parent_to_joint_origin_transform.position.y << ", "
                                        << wheel0_urdfJoint->parent_to_joint_origin_transform.position.z);
  ROS_INFO_STREAM("wheel1 to origin: "  << wheel1_urdfJoint->parent_to_joint_origin_transform.position.x << ","
                                        << wheel1_urdfJoint->parent_to_joint_origin_transform.position.y << ", "
                                        << wheel1_urdfJoint->parent_to_joint_origin_transform.position.z);
  ROS_INFO_STREAM("wheel2 to origin: "  << wheel2_urdfJoint->parent_to_joint_origin_transform.position.x << ","
                                        << wheel2_urdfJoint->parent_to_joint_origin_transform.position.y << ", "
                                        << wheel2_urdfJoint->parent_to_joint_origin_transform.position.z);
  ROS_INFO_STREAM("wheel3 to origin: "  << wheel3_urdfJoint->parent_to_joint_origin_transform.position.x << ","
                                        << wheel3_urdfJoint->parent_to_joint_origin_transform.position.y << ", "
                                        << wheel3_urdfJoint->parent_to_joint_origin_transform.position.z);

  double wheel0_x = wheel0_urdfJoint->parent_to_joint_origin_transform.position.x;
  double wheel0_y = wheel0_urdfJoint->parent_to_joint_origin_transform.position.y;
  double wheel1_x = wheel1_urdfJoint->parent_to_joint_origin_transform.position.x;
  double wheel1_y = wheel1_urdfJoint->parent_to_joint_origin_transform.position.y;
  double wheel2_x = wheel2_urdfJoint->parent_to_joint_origin_transform.position.x;
  double wheel2_y = wheel2_urdfJoint->parent_to_joint_origin_transform.position.y;
  double wheel3_x = wheel3_urdfJoint->parent_to_joint_origin_transform.position.x;
  double wheel3_y = wheel3_urdfJoint->parent_to_joint_origin_transform.position.y;

  wheels_k_ = (-(-wheel0_x - wheel0_y) - (wheel1_x - wheel1_y) + (-wheel2_x - wheel2_y) + (wheel3_x - wheel3_y)) / 4.0;

  // Get wheels radius
  double wheel0_radius = 0.0;
  double wheel1_radius = 0.0;
  double wheel2_radius = 0.0;
  double wheel3_radius = 0.0;

  if (!getWheelRadius(model, model->getLink(wheel0_urdfJoint->child_link_name), wheel0_radius) ||
      !getWheelRadius(model, model->getLink(wheel1_urdfJoint->child_link_name), wheel1_radius) ||
      !getWheelRadius(model, model->getLink(wheel2_urdfJoint->child_link_name), wheel2_radius) ||
      !getWheelRadius(model, model->getLink(wheel3_urdfJoint->child_link_name), wheel3_radius))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve wheels' radius");
    return false;
  }

  if (abs(wheel0_radius - wheel1_radius) > 1e-3 ||
      abs(wheel0_radius - wheel2_radius) > 1e-3 ||
      abs(wheel0_radius - wheel3_radius) > 1e-3)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Wheels radius are not egual");
    return false;
  }

  wheels_radius_ = wheel0_radius;

  ROS_INFO_STREAM("Wheel radius: " << wheels_radius_);

  // Set wheel params for the odometry computation
  odometry_.setWheelsParams(wheels_k_, wheels_radius_);

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MecanumDriveController::getWheelRadius(const boost::shared_ptr<urdf::ModelInterface> model, const boost::shared_ptr<const urdf::Link>& wheel_link, double& wheel_radius)
{
  boost::shared_ptr<const urdf::Link> radius_link = wheel_link;

  if (use_realigned_roller_joints_)
  {
      // This mode is used when the mecanum wheels are simulated and we use realigned rollers to mimic mecanum wheels.
      const boost::shared_ptr<const urdf::Joint>& roller_joint = radius_link->child_joints[0];
      if(!roller_joint)
      {
        ROS_ERROR_STREAM_NAMED(name_, "No roller joint could be retrieved for wheel : " << wheel_link->name << ". Are you sure mecanum wheels are simulated using realigned rollers?");
        return false;
      }

      radius_link = model->getLink(roller_joint->child_link_name);
      if(!radius_link)
      {
        ROS_ERROR_STREAM_NAMED(name_, "No roller link could be retrieved for wheel : " << wheel_link->name << ". Are you sure mecanum wheels are simulated using realigned rollers?");
        return false;
      }
  }

  if(!isCylinderOrSphere(radius_link))
  {
    ROS_ERROR_STREAM("Wheel link " << radius_link->name << " is NOT modeled as a cylinder!");
    return false;
  }

  if (radius_link->collision->geometry->type == urdf::Geometry::CYLINDER)
    wheel_radius = (static_cast<urdf::Cylinder*>(radius_link->collision->geometry.get()))->radius;
  else
    wheel_radius = (static_cast<urdf::Sphere*>(radius_link->collision->geometry.get()))->radius;

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MecanumDriveController::setupRtPublishersMsg(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Get covariance parameters for odometry.
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

  // Setup odometry msg.
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = base_frame_id_;
  odom_pub_->msg_.pose.pose.position.z = 0;
  odom_pub_->msg_.pose.covariance = boost::assign::list_of
      (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
      (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
      (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
      (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
      (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
      (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
  odom_pub_->msg_.twist.twist.linear.y  = 0;
  odom_pub_->msg_.twist.twist.linear.z  = 0;
  odom_pub_->msg_.twist.twist.angular.x = 0;
  odom_pub_->msg_.twist.twist.angular.y = 0;
  odom_pub_->msg_.twist.covariance = boost::assign::list_of
      (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
      (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
      (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
      (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
      (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
      (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));

  // Setup tf msg.
  tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
  tf_odom_pub_->msg_.transforms.resize(1);
  tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
  tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
  tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
}

} // namespace mecanum_drive_controller
