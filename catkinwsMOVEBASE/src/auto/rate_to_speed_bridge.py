#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, Float32MultiArray

class RateToSpeedBridge:
    def __init__(self):
        rospy.init_node('rate_to_speed_bridge')
        self.max_ticks_per_sec = rospy.get_param('~max_ticks_per_sec', 1000.0)  # Calibrate
        self.left_speed = 0.0
        self.right_speed = 0.0
        rospy.Subscriber('/left_wheel_desired_rate', Int32, self.left_callback)
        rospy.Subscriber('/right_wheel_desired_rate', Int32, self.right_callback)
        self.pub = rospy.Publisher('/motor_speeds', Float32MultiArray, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_speeds)  # 20 Hz
        rospy.spin()

    def left_callback(self, msg):
        self.left_speed = float(msg.data) / self.max_ticks_per_sec

    def right_callback(self, msg):
        self.right_speed = float(msg.data) / self.max_ticks_per_sec

if __name__ == '__main__':
    RateToSpeedBridge()
