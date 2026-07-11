#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class DiffDriveController:
    """
    Differential drive kinematic controller for 2-wheel ground rover.
    Translates Twist (cmd_vel) messages into normalized left/right motor speed commands.
    """
    def __init__(self):
        rospy.init_node('diff_drive_controller')

        # Load physical parameters from ROS parameter server with sensible defaults
        self.wheel_base = rospy.get_param('~wheel_separation', rospy.get_param('~wheel_base', 0.365))
        self.max_speed = rospy.get_param('~max_speed', 0.6)
        self.right_motor_trim = rospy.get_param('~right_motor_trim', 1.34)
        self.left_motor_trim = rospy.get_param('~left_motor_trim', 1.0)
        self.timeout = rospy.get_param('~timeout', 0.5)
        self.rate_hz = rospy.get_param('~rate', 50)

        self.last_cmd_time = rospy.Time.now()

        # ROS Subscriber and Publisher
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)
        self.motor_pub = rospy.Publisher('/motor_speeds', Float32MultiArray, queue_size=1)

        rospy.loginfo("Differential Drive Controller Initialized.")
        rospy.loginfo("Wheel Base: %.3f m | Max Speed: %.2f m/s | Right Trim: %.2f",
                      self.wheel_base, self.max_speed, self.right_motor_trim)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = rospy.Time.now()
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Differential drive inverse kinematics:
        # v_right = v + (omega * L / 2)
        # v_left  = v - (omega * L / 2)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2.0)
        v_left = linear_vel - (angular_vel * self.wheel_base / 2.0)

        # Apply motor calibration trims
        v_left_trimmed = v_left * self.left_motor_trim
        v_right_trimmed = v_right * self.right_motor_trim

        # Scale wheel speeds to [-1.0, 1.0]
        left_speed = self.scale_speed(v_left_trimmed)
        right_speed = self.scale_speed(v_right_trimmed)

        self.publish_speeds(left_speed, right_speed)

    def scale_speed(self, speed):
        """Scales raw speed to [-1.0, 1.0] range based on max_speed."""
        if self.max_speed <= 0.0:
            return 0.0
        scaled = speed / self.max_speed
        return max(min(scaled, 1.0), -1.0)

    def publish_speeds(self, left, right):
        motor_speeds_msg = Float32MultiArray()
        motor_speeds_msg.data = [float(left), float(right)]
        self.motor_pub.publish(motor_speeds_msg)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            # Watchdog timeout to halt motors if cmd_vel stream drops
            if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.timeout:
                self.publish_speeds(0.0, 0.0)
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = DiffDriveController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
