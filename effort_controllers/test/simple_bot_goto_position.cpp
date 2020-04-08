#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>


class SimpleBotFixture : public ::testing::Test {
protected:
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::ServiceClient load, unload, start;
  ros::NodeHandle nh;
  double position;

  int val;

  void positionCB(const sensor_msgs::JointState& msg) {
    position = msg.position[0];
  }

  bool waitToReachTarget(double target, std::string& err, double precision=5e-2, double timeout=20) {
    position = 1e3*precision + target; // make position != target
    ros::Rate rate(20);
    std_msgs::Float64 cmd;
    cmd.data = target;
    ros::Time start = ros::Time::now();

    while(ros::ok()) {
      pub.publish(cmd);
      if(std::fabs(target-position) < precision)
        return true;
      if( (ros::Time::now()-start).toSec() > timeout ) {
        err = "Timed out, target = " + std::to_string(target) + ", current = " + std::to_string(position);
        return false;
      }
      rate.sleep();
    }

    err = "ROS shutdwon, target = " + std::to_string(target) + ", current = " + std::to_string(position);
    return false;
  }


  void SetUp() override {
    // setup
    position = 0;
    pub = nh.advertise<std_msgs::Float64>("position_controller/command", 1);
    sub = nh.subscribe("joint_states", 1, &SimpleBotFixture::positionCB, this);
    load = nh.serviceClient<controller_manager_msgs::LoadController>("controller_manager/load_controller");
    unload = nh.serviceClient<controller_manager_msgs::UnloadController>("controller_manager/unload_controller");
    start = nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
  }
};


TEST_F(SimpleBotFixture, TestPosition) {
  ros::Duration(1.0).sleep();

  ASSERT_TRUE(load.waitForExistence(ros::Duration(20)));
  ASSERT_TRUE(unload.waitForExistence(ros::Duration(20)));
  ASSERT_TRUE(start.waitForExistence(ros::Duration(20)));

  controller_manager_msgs::SwitchController switch_srv;
  switch_srv.request.start_controllers.push_back("position_controller");
  switch_srv.request.start_controllers.push_back("joint_state_controller");
  switch_srv.request.strictness = switch_srv.request.STRICT;

  for(const auto& controller : switch_srv.request.start_controllers) {
    controller_manager_msgs::LoadController load_srv;
    load_srv.request.name = controller;
    ASSERT_TRUE(load.call(load_srv));
    ASSERT_TRUE(load_srv.response.ok);
  }

  ASSERT_TRUE(start.call(switch_srv));
  ASSERT_TRUE(switch_srv.response.ok);

  std::string err;
  ASSERT_TRUE(waitToReachTarget(1.0, err)) << err;
  ASSERT_TRUE(waitToReachTarget(-1.0, err)) << err;
  ASSERT_TRUE(waitToReachTarget(0.0, err)) << err;

  switch_srv.request.stop_controllers = switch_srv.request.start_controllers;
  switch_srv.request.start_controllers.clear();

  ASSERT_TRUE(start.call(switch_srv));
  ASSERT_TRUE(switch_srv.response.ok);

  for(const auto& controller : switch_srv.request.stop_controllers) {
    controller_manager_msgs::UnloadController unload_srv;
    unload_srv.request.name = controller;
    ASSERT_TRUE(unload.call(unload_srv));
    ASSERT_TRUE(unload_srv.response.ok);
  }
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dummy");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
