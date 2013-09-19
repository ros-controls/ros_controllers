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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <urdf_parser/urdf_parser.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <boost/assign.hpp>

#include <diff_drive_controller/odometry.h>

namespace diff_drive_controller{

  class DiffDriveController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    bool init(hardware_interface::VelocityJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh)
    {
      name_ = getLeafNamespace(controller_nh);
      // get joint names from the parameter server
      std::string left_wheel_name, right_wheel_name;

      bool res = controller_nh.hasParam("left_wheel");
      if(!res || !controller_nh.getParam("left_wheel", left_wheel_name))
      {
        ROS_ERROR("Couldn't retrieve left wheel name from param server.");
        return false;
      }
      res = controller_nh.hasParam("right_wheel");
      if(!res || !controller_nh.getParam("right_wheel", right_wheel_name))
      {
        ROS_ERROR("Couldn't retrieve right wheel name from param server.");
        return false;
      }

      // parse robot description
      const std::string model_param_name = "/robot_description";
      res = root_nh.hasParam(model_param_name);
      std::string robot_model_str="";
      if(!res || !root_nh.getParam(model_param_name,robot_model_str))
      {
        ROS_ERROR("Robot descripion couldn't be retrieved from param server.");
        return false;
      }

      boost::shared_ptr<urdf::ModelInterface> model_ptr(urdf::parseURDF(robot_model_str));
      boost::shared_ptr<const urdf::Joint> wheelJointPtr(model_ptr->getJoint(left_wheel_name));
      if(!wheelJointPtr.get())
      {
        ROS_ERROR_STREAM(left_wheel_name << " couldn't be retrieved from model description");
        return false;
      }

      wheel_radius_ = fabs(wheelJointPtr->parent_to_joint_origin_transform.position.z);
      wheel_separation_ = 2.0 * fabs(wheelJointPtr->parent_to_joint_origin_transform.position.y);
      ROS_DEBUG_STREAM("Odometry params : wheel separation " << wheel_separation_
                       << ", wheel radius " << wheel_radius_);

      // get the joint object to use in the realtime loop
      ROS_DEBUG_STREAM("Adding left wheel with joint name: " << left_wheel_name
                       << " and right wheel with joint name: " << right_wheel_name);
      left_wheel_joint_ = hw->getHandle(left_wheel_name);  // throws on failure
      right_wheel_joint_ = hw->getHandle(right_wheel_name);  // throws on failure

      // setup odometry realtime publisher + odom message constant fields
      odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
      odom_pub_->msg_.header.frame_id = "odom";
      odom_pub_->msg_.pose.pose.position.z = 0;
      odom_pub_->msg_.pose.covariance = boost::assign::list_of
          (1e-3) (0)   (0)  (0)  (0)  (0)
          (0) (1e-3)  (0)  (0)  (0)  (0)
          (0)   (0)  (1e6) (0)  (0)  (0)
          (0)   (0)   (0) (1e6) (0)  (0)
          (0)   (0)   (0)  (0) (1e6) (0)
          (0)   (0)   (0)  (0)  (0)  (1e3);
      odom_pub_->msg_.twist.twist.linear.y  = 0;
      odom_pub_->msg_.twist.twist.linear.z  = 0;
      odom_pub_->msg_.twist.twist.angular.x = 0;
      odom_pub_->msg_.twist.twist.angular.y = 0;
      odom_pub_->msg_.twist.covariance = boost::assign::list_of
          (1e-3) (0)   (0)  (0)  (0)  (0)
          (0) (1e-3)  (0)  (0)  (0)  (0)
          (0)   (0)  (1e6) (0)  (0)  (0)
          (0)   (0)   (0) (1e6) (0)  (0)
          (0)   (0)   (0)  (0) (1e6) (0)
          (0)   (0)   (0)  (0)  (0)  (1e3);
      tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(controller_nh, "/tf", 100));
      odom_frame.transform.translation.z = 0.0;
      odom_frame.child_frame_id = "base_footprint";
      odom_frame.header.frame_id = "odom";

      sub_command_ = controller_nh.subscribe("cmd_vel", 1, &DiffDriveController::cmdVelCallback, this);

      return true;
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
      // MOVE ROBOT
      // command the wheels according to the messages from the cmd_vel topic
      const Commands curr_cmd = *(command_.readFromRT());
      const double vel_right =
          (curr_cmd.lin + curr_cmd.ang * wheel_separation_ / 2.0)/wheel_radius_;
      const double vel_left =
          (curr_cmd.lin - curr_cmd.ang * wheel_separation_ / 2.0)/wheel_radius_;
      left_wheel_joint_.setCommand(vel_left);
      right_wheel_joint_.setCommand(vel_right);



      // estimate linear and angular velocity using joint information
      //----------------------------
      // get current wheel joint positions
      left_wheel_cur_pos_ = left_wheel_joint_.getPosition()*wheel_radius_;
      right_wheel_cur_pos_ = right_wheel_joint_.getPosition()*wheel_radius_;
      // estimate velocity of wheels using old and current position
      left_wheel_est_vel_ = left_wheel_cur_pos_ - left_wheel_old_pos_;
      right_wheel_est_vel_ = right_wheel_cur_pos_ - right_wheel_old_pos_;
      // update old position with current
      left_wheel_old_pos_ = left_wheel_cur_pos_;
      right_wheel_old_pos_ = right_wheel_cur_pos_;

      // compute linear and angular velocity
      const double linear = (left_wheel_est_vel_ + right_wheel_est_vel_) * 0.5 ;
      const double angular = (right_wheel_est_vel_ - left_wheel_est_vel_) / wheel_separation_;

      if((fabs(angular) < 1e-15) && (fabs(linear) < 1e-15)) // when both velocities are ~0
      {
        ROS_WARN("Dropped odom: velocities are 0.");
        return;
      }

      if(!odometryReading_.integrate(linear, angular, time))
      {
        ROS_WARN("Dropped odom: interval too small to integrate.");
        return;
      }
      // estimate speeds
      speedEstimation(odometryReading_.getLinear(), odometryReading_.getAngular());

      //----------------------------

      // compute and store orientation info
      const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometryReading_.getHeading()));
      // COMPUTE AND PUBLISH ODOMETRY
      if(odom_pub_->trylock())
      {
        // populate odom message and publish
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometryReading_.getPos().x;
        odom_pub_->msg_.pose.pose.position.y = odometryReading_.getPos().y;
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x  = linear_est_speed_;
        odom_pub_->msg_.twist.twist.angular.z = angular_est_speed_;
        odom_pub_->unlockAndPublish();
      }

      // publish tf /odom frame
      if(tf_odom_pub_->trylock())
      {
        //publish odom tf
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometryReading_.getPos().x;
        odom_frame.transform.translation.y = odometryReading_.getPos().y;
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->msg_.transforms.clear();
        tf_odom_pub_->msg_.transforms.push_back(odom_frame);
        tf_odom_pub_->unlockAndPublish();
      }
    }

    void starting(const ros::Time& time)
    {
      // set velocity to 0
      const double vel = 0.0;
      left_wheel_joint_.setCommand(vel);
      right_wheel_joint_.setCommand(vel);
    }

    void stopping(const ros::Time& time)
    {
      // set velocity to 0
      const double vel = 0.0;
      left_wheel_joint_.setCommand(vel);
      right_wheel_joint_.setCommand(vel);
    }

  private:
    std::string name_;

    struct Commands
    {
      double lin;
      double ang;
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    hardware_interface::JointHandle left_wheel_joint_;
    hardware_interface::JointHandle right_wheel_joint_;

    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

    Odometry odometryReading_;
    double wheel_separation_;
    double wheel_radius_;
    double left_wheel_old_pos_, right_wheel_old_pos_;
    double left_wheel_cur_pos_, right_wheel_cur_pos_;
    double left_wheel_est_vel_, right_wheel_est_vel_;
    double linear_est_speed_, angular_est_speed_;
    geometry_msgs::TransformStamped odom_frame;
    std::list<geometry_msgs::Point> lastSpeeds;
    typedef std::list<geometry_msgs::Point>::const_iterator speeds_it;

  private:
    void cmdVelCallback(const geometry_msgs::Twist& command)
    {
      if(isRunning())
      {
        command_struct_.ang = command.angular.z;
        command_struct_.lin = command.linear.x;
        command_.writeFromNonRT (command_struct_);
        ROS_DEBUG_STREAM("Added values to command. Ang: " << command_struct_.ang
                         << ", Lin: " << command_struct_.lin);
      }
      else
      {
        ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
      }
    }

    std::string getLeafNamespace(const ros::NodeHandle& nh)
    {
      const std::string complete_ns = nh.getNamespace();
      std::size_t id = complete_ns.find_last_of("/");
      return complete_ns.substr(id + 1);
    }

    void speedEstimation(const double &linear, double const &angular)
    {
      if(lastSpeeds.size()> 10)
        lastSpeeds.pop_front();

      geometry_msgs::Point speed;
      speed.x = linear;
      speed.y = angular;
      lastSpeeds.push_back(speed);

      double averageLinearSpeed = 0.0, averageAngularSpeed = 0.0;
      for(speeds_it it=lastSpeeds.begin();it!= lastSpeeds.end(); ++it)
      {
        averageLinearSpeed += it->x;
        averageAngularSpeed += it->y;
      }

      linear_est_speed_  =  averageLinearSpeed/lastSpeeds.size();
      angular_est_speed_ =  averageAngularSpeed/lastSpeeds.size();
    }
  };

  PLUGINLIB_DECLARE_CLASS(diff_drive_controller, DiffDriveController, diff_drive_controller::DiffDriveController, controller_interface::ControllerBase);
}//namespace
