#!/usr/bin/env python
import rospy
from math import pi
from four_wheel_steering_msgs.msg import FourWheelSteering
from sensor_msgs.msg import Joy

class TeleopFourWheelSteeringJoy():
    def __init__(self):
        self.axis_dead_zone = 0.05
        self.axis_linear_forward = 5  # Right Trigger
        self.axis_linear_reverse = 2  # Left Trigger
        self.scale_linear = 1
        self.axis_front_steering = 0  # Right left-right stick
        self.axis_rear_steering = 3   # Left left-right stick
        self.scale_steering = pi/10.0

        self.is_trigger_forward_init = False
        self.is_trigger_reverse_init = False
        self.last_forward_speed = 0.0
        self.last_reverse_speed = 0.0

        rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher('cmd_four_wheel_steering', FourWheelSteering, queue_size=10)

        rospy.spin()

    def callback(self, data):
        four_wheel_steering_msg = FourWheelSteering()

        #rospy.loginfo(rospy.get_caller_id() + " axes" + str(data.axes))
        linear_forward = 1.0
        linear_reverse = 1.0
        if self.is_trigger_forward_init:
            linear_forward = data.axes[self.axis_linear_forward]
        elif abs(self.last_forward_speed - data.axes[self.axis_linear_forward]) > 0.01:
            self.is_trigger_forward_init =  True
        else:
            self.last_forward_speed = data.axes[self.axis_linear_forward]

        if self.is_trigger_reverse_init:
            linear_reverse = data.axes[self.axis_linear_reverse]
        elif abs(self.last_reverse_speed - data.axes[self.axis_linear_reverse]) > 0.01:
            self.is_trigger_reverse_init =  True
        else:
            self.last_reverse_speed = data.axes[self.axis_linear_reverse]

        speed = (-linear_forward + linear_reverse)/2.0
        #rospy.loginfo(rospy.get_caller_id() + " speed %s", speed)

        if abs(speed) > self.axis_dead_zone:
            four_wheel_steering_msg.speed = speed*self.scale_linear

        if abs(data.axes[self.axis_front_steering]) > self.axis_dead_zone:
            four_wheel_steering_msg.front_steering_angle = data.axes[self.axis_front_steering]*self.scale_steering
        else:
            four_wheel_steering_msg.front_steering_angle = 0.0

        if abs(data.axes[self.axis_rear_steering]) > self.axis_dead_zone:
            four_wheel_steering_msg.rear_steering_angle = data.axes[self.axis_rear_steering]*self.scale_steering
        else:
            four_wheel_steering_msg.rear_steering_angle = 0.0

        self.pub.publish(four_wheel_steering_msg)


if __name__ == '__main__':
    rospy.init_node('teleop_four_wheel_steering_joy', anonymous=False)
    try:
        teleop_four_wheel_steering_joy = TeleopFourWheelSteeringJoy()
    except rospy.ROSInterruptException: pass
