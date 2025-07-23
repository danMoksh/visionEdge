#!/usr/bin/env python3

import rospy
import Jetson.GPIO as GPIO
import time  # Don't forget to import time for sleep

# Use RPi.GPIO if on Raspberry Pi
from std_srvs.srv import Empty, EmptyResponse

class MotorDriver:
    def __init__(self):
        # Motor 1 pins
        self.ENA = 33
        self.IN1 = 35
        self.IN2 = 37
        
        # Motor 2 pins
        self.ENB = 32
        self.IN3 = 36
        self.IN4 = 38
        
        # GPIO setup
        GPIO.setmode(GPIO.BOARD)
        
        for pin in [self.ENA, self.IN1, self.IN2, self.ENB, self.IN3, self.IN4]:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        
        # Enable both motors
        GPIO.output(self.ENA, GPIO.HIGH)
        GPIO.output(self.ENB, GPIO.HIGH)
        
        # ROS services
        rospy.Service('motor_forward', Empty, self.handle_forward)
        rospy.Service('motor_reverse', Empty, self.handle_reverse)
        rospy.Service('motor_stop', Empty, self.handle_stop)
        
        rospy.loginfo("Motor driver ready. Services: /motor_forward /motor_reverse /motor_stop")

    def handle_forward(self, req):
        self.forward()
        return EmptyResponse()

    def handle_reverse(self, req):
        self.reverse()
        return EmptyResponse()

    def handle_stop(self, req):
        self.stop()
        return EmptyResponse()

    def forward(self):
        rospy.loginfo("Motors moving forward")
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        # Stop
GPIO.output(IN1, GPIO.LOW)
GPIO.output(IN2, GPIO.LOW)
time.sleep(1)


    def reverse(self):
        rospy.loginfo("Motors moving backward")
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        # Stop
GPIO.output(IN1, GPIO.LOW)
GPIO.output(IN2, GPIO.LOW)
time.sleep(1)


    def stop(self):
        rospy.loginfo("Motors stopped")
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)

    def cleanup(self):
        rospy.loginfo("Cleaning up GPIO")
        self.stop()
        GPIO.output(self.ENA, GPIO.LOW)
        GPIO.output(self.ENB, GPIO.LOW)
        GPIO.cleanup()

if __name__ == '__main__':
    rospy.init_node('motor_driver_node')
    driver = MotorDriver()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        driver.cleanup()

