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
      const std::string complete_ns = controller_nh.getNamespace();
      std::size_t id = complete_ns.find_last_of("/");
      name_ = complete_ns.substr(id + 1);
      // get joint names from the parameter server
      std::string left_wheel_name, right_wheel_name;

      bool res = controller_nh.hasParam("left_wheel");
      if(!res || !controller_nh.getParam("left_wheel", left_wheel_name))
      {
        ROS_ERROR_NAMED(name_, "Couldn't retrieve left wheel name from param server.");
        return false;
      }
      res = controller_nh.hasParam("right_wheel");
      if(!res || !controller_nh.getParam("right_wheel", right_wheel_name))
      {
        ROS_ERROR_NAMED(name_, "Couldn't retrieve right wheel name from param server.");
        return false;
      }

      double publish_rate;
      controller_nh.param("publish_rate", publish_rate, 50.0);
      ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                            << publish_rate << "Hz.");
      publish_period_ = ros::Duration(1.0 / publish_rate);

      // parse robot description
      const std::string model_param_name = "/robot_description";
      res = root_nh.hasParam(model_param_name);
      std::string robot_model_str="";
      if(!res || !root_nh.getParam(model_param_name,robot_model_str))
      {
        ROS_ERROR_NAMED(name_, "Robot descripion couldn't be retrieved from param server.");
        return false;
      }

      boost::shared_ptr<urdf::ModelInterface> model_ptr(urdf::parseURDF(robot_model_str));
      boost::shared_ptr<const urdf::Joint> wheelJointPtr(model_ptr->getJoint(left_wheel_name));
      if(!wheelJointPtr.get())
      {
        ROS_ERROR_STREAM_NAMED(name_, left_wheel_name
                               << " couldn't be retrieved from model description");
        return false;
      }

      wheel_radius_ = fabs(wheelJointPtr->parent_to_joint_origin_transform.position.z);
      wheel_separation_ = 2.0 * fabs(wheelJointPtr->parent_to_joint_origin_transform.position.y);
      odometry_.setWheelParams(wheel_separation_, wheel_radius_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Odometry params : wheel separation " << wheel_separation_
                             << ", wheel radius " << wheel_radius_);

      // get and check params for covariances
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

      // setup odometry realtime publisher + odom message constant fields
      odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "/odom", 100));
      odom_pub_->msg_.header.frame_id = "odom";
      odom_pub_->msg_.pose.pose.position.z = 0;
      odom_pub_->msg_.pose.covariance = boost::assign::list_of
          (static_cast<double>(pose_cov_list[0])) (0)   (0)  (0)  (0)  (0)
          (0) (static_cast<double>(pose_cov_list[1]))  (0)  (0)  (0)  (0)
          (0)   (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
          (0)   (0)   (0) (static_cast<double>(pose_cov_list[3])) (0)  (0)
          (0)   (0)   (0)  (0) (static_cast<double>(pose_cov_list[4])) (0)
          (0)   (0)   (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
      odom_pub_->msg_.twist.twist.linear.y  = 0;
      odom_pub_->msg_.twist.twist.linear.z  = 0;
      odom_pub_->msg_.twist.twist.angular.x = 0;
      odom_pub_->msg_.twist.twist.angular.y = 0;
      odom_pub_->msg_.twist.covariance = boost::assign::list_of
          (static_cast<double>(twist_cov_list[0])) (0)   (0)  (0)  (0)  (0)
          (0) (static_cast<double>(twist_cov_list[1]))  (0)  (0)  (0)  (0)
          (0)   (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
          (0)   (0)   (0) (static_cast<double>(twist_cov_list[3])) (0)  (0)
          (0)   (0)   (0)  (0) (static_cast<double>(twist_cov_list[4])) (0)
          (0)   (0)   (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));
      tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(controller_nh, "/tf", 100));
      odom_frame.transform.translation.z = 0.0;
      odom_frame.child_frame_id = "base_footprint";
      odom_frame.header.frame_id = "odom";

      // get the joint object to use in the realtime loop
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Adding left wheel with joint name: " << left_wheel_name
                             << " and right wheel with joint name: " << right_wheel_name);
      left_wheel_joint_ = hw->getHandle(left_wheel_name);  // throws on failure
      right_wheel_joint_ = hw->getHandle(right_wheel_name);  // throws on failure

      sub_command_ = root_nh.subscribe("/cmd_vel", 1, &DiffDriveController::cmdVelCallback, this);

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

      // COMPUTE AND PUBLISH ODOMETRY
      // estimate linear and angular velocity using joint information
      //----------------------------
      if(!odometry_.update(left_wheel_joint_.getPosition(), right_wheel_joint_.getPosition(), time))
      {
        ROS_WARN_NAMED(name_,
                       "Dropped odom: period too small to integrate or no change.");
        return;
      }

      // publish odometry message
      if(last_state_publish_time_ + publish_period_ < time)
      {
        last_state_publish_time_ += publish_period_;
        // compute and store orientation info
        const geometry_msgs::Quaternion orientation(
              tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

        // populate odom message and publish
        if(odom_pub_->trylock())
        {
          odom_pub_->msg_.header.stamp = time;
          odom_pub_->msg_.pose.pose.position.x = odometry_.getPos().x;
          odom_pub_->msg_.pose.pose.position.y = odometry_.getPos().y;
          odom_pub_->msg_.pose.pose.orientation = orientation;
          odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinearEstimated();
          odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngularEstimated();
          odom_pub_->unlockAndPublish();
        }

        // publish tf /odom frame
        if(tf_odom_pub_->trylock())
        {
          odom_frame.header.stamp = time;
          odom_frame.transform.translation.x = odometry_.getPos().x;
          odom_frame.transform.translation.y = odometry_.getPos().y;
          odom_frame.transform.rotation = orientation;
          tf_odom_pub_->msg_.transforms.clear();
          tf_odom_pub_->msg_.transforms.push_back(odom_frame);
          tf_odom_pub_->unlockAndPublish();
        }
      }
    }

    void starting(const ros::Time& time)
    {
      // set velocity to 0
      const double vel = 0.0;
      left_wheel_joint_.setCommand(vel);
      right_wheel_joint_.setCommand(vel);

      // register starting time used to keep fixed rate
      last_state_publish_time_ = time;
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

    // publish rate related
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;

    // hardware handles
    hardware_interface::JointHandle left_wheel_joint_;
    hardware_interface::JointHandle right_wheel_joint_;

    // cmd_vel related
    struct Commands
    {
      double lin;
      double ang;
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    // odometry related
    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
    Odometry odometry_;
    double wheel_separation_;
    double wheel_radius_;
    geometry_msgs::TransformStamped odom_frame;

  private:
    void cmdVelCallback(const geometry_msgs::Twist& command)
    {
      if(isRunning())
      {
        command_struct_.ang = command.angular.z;
        command_struct_.lin = command.linear.x;
        command_.writeFromNonRT (command_struct_);
        ROS_DEBUG_STREAM_NAMED(name_,
                               "Added values to command. Ang: " << command_struct_.ang
                               << ", Lin: " << command_struct_.lin);
      }
      else
      {
        ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
      }
    }

  };

  PLUGINLIB_DECLARE_CLASS(diff_drive_controller, DiffDriveController, diff_drive_controller::DiffDriveController, controller_interface::ControllerBase);
}//namespace
