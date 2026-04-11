#!/usr/bin/env python
from __future__ import print_function
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Int32

# Using Python 2 print function

class EncoderNode:
    def __init__(self):
        rospy.init_node('wheel_encoder_node')

        # --- Parameters ---
        self.left_encoder_pin = rospy.get_param('~left_encoder_pin', 11)
        self.right_encoder_pin = rospy.get_param('~right_encoder_pin', 16)
        self.rate = rospy.get_param('~rate', 20) # Publish rate in Hz

        # --- Publishers ---
        self.left_pub = rospy.Publisher('/left_wheel_ticks', Int32, queue_size=10)
        self.right_pub = rospy.Publisher('/right_wheel_ticks', Int32, queue_size=10)

        # --- State Variables ---
        self.left_ticks = 0
        self.right_ticks = 0

        # --- GPIO Setup ---
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.left_encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Setup edge detection with debouncing
        GPIO.add_event_detect(self.left_encoder_pin, GPIO.RISING, callback=self.left_encoder_callback, bouncetime=2)
        GPIO.add_event_detect(self.right_encoder_pin, GPIO.RISING, callback=self.right_encoder_callback, bouncetime=2)

        rospy.loginfo("Wheel encoder node started. Pins L:%d, R:%d", self.left_encoder_pin, self.right_encoder_pin)
        rospy.on_shutdown(self.cleanup)

    def left_encoder_callback(self, channel):
        self.left_ticks += 1

    def right_encoder_callback(self, channel):
        self.right_ticks += 1

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.left_pub.publish(self.left_ticks)
            self.right_pub.publish(self.right_ticks)
            rate.sleep()

    def cleanup(self):
        print("Stopping wheel encoder node and cleaning up GPIO...")
        GPIO.cleanup()

if __name__ == '__main__':
    try:
        node = EncoderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
