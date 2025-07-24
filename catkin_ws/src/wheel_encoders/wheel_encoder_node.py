#!/usr/bin/env python
from __future__ import print_function

import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Int32, Float32

class EncoderNode:
    def __init__(self):
        rospy.init_node('wheel_encoder_node', anonymous=False)

        # --- Parameters ---
        self.left_encoder_pin = rospy.get_param('~left_encoder_pin', 11)
        self.right_encoder_pin = rospy.get_param('~right_encoder_pin', 16)
        self.rate = rospy.get_param('~rate', 20.0)  # Publish rate in Hz
        self.ticks_per_meter = rospy.get_param('~ticks_per_meter', 1000)  # Encoder ticks per meter
        self.timeout = rospy.get_param('~timeout', 0.2)  # Timeout for rate calculation (s)

        # --- Publishers ---
        self.left_ticks_pub = rospy.Publisher('~lwheel_ticks', Int32, queue_size=10)
        self.right_ticks_pub = rospy.Publisher('~rwheel_ticks', Int32, queue_size=10)
        self.left_rate_pub = rospy.Publisher('~lwheel_rate', Float32, queue_size=10)
        self.right_rate_pub = rospy.Publisher('~rwheel_rate', Float32, queue_size=10)

        # --- State Variables ---
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_time = rospy.get_rostime()

        # --- GPIO Setup ---
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.left_encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Setup edge detection with debouncing
        GPIO.add_event_detect(self.left_encoder_pin, GPIO.RISING, 
                            callback=self.left_encoder_callback, bouncetime=2)
        GPIO.add_event_detect(self.right_encoder_pin, GPIO.RISING, 
                            callback=self.right_encoder_callback, bouncetime=2)

        rospy.loginfo("Wheel encoder node started. Pins L:%d, R:%d", 
                     self.left_encoder_pin, self.right_encoder_pin)
        rospy.loginfo("Ticks per meter: %d", self.ticks_per_meter)
        rospy.loginfo("Publish rate: %.2f Hz", self.rate)
        rospy.on_shutdown(self.cleanup)

    def left_encoder_callback(self, channel):
        """Callback for left wheel encoder ticks."""
        self.left_ticks += 1

    def right_encoder_callback(self, channel):
        """Callback for right wheel encoder ticks."""
        self.right_ticks += 1

    def calculate_rates(self):
        """Calculate wheel rates in ticks/second."""
        current_time = rospy.get_rostime()
        dt = (current_time - self.last_time).to_sec()

        if dt < self.timeout:
            # Calculate rates in ticks/second
            left_rate = (self.left_ticks - self.last_left_ticks) / dt
            right_rate = (self.right_ticks - self.last_right_ticks) / dt
        else:
            # Timeout: assume zero velocity
            left_rate = 0.0
            right_rate = 0.0

        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks
        self.last_time = current_time

        return left_rate, right_rate

    def run(self):
        """Main loop to publish ticks and rates."""
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # Publish cumulative ticks
            self.left_ticks_pub.publish(Int32(self.left_ticks))
            self.right_ticks_pub.publish(Int32(self.right_ticks))

            # Calculate and publish rates
            left_rate, right_rate = self.calculate_rates()
            self.left_rate_pub.publish(Float32(left_rate))
            self.right_rate_pub.publish(Float32(right_rate))

            rate.sleep()

    def cleanup(self):
        """Cleanup GPIO on shutdown."""
        rospy.loginfo("Stopping wheel encoder node and cleaning up GPIO...")
        GPIO.cleanup()

if __name__ == '__main__':
    try:
        node = EncoderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
