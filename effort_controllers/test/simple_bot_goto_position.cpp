#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>


class SimpleBotFixture : public ::testing::Test {
protected:
  ros::Publisher pub;
  ros::Subscriber sub;
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
  }
};


TEST_F(SimpleBotFixture, TestPosition) {
  ros::Duration(1.0).sleep();
  std::string err;
  ASSERT_TRUE(waitToReachTarget(1.0, err)) << err;
  ASSERT_TRUE(waitToReachTarget(-1.0, err)) << err;
  ASSERT_TRUE(waitToReachTarget(0.0, err)) << err;
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
