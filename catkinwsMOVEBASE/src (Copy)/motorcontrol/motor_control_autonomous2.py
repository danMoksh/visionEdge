#!/usr/bin/env python

import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Float32MultiArray

class DirectMotorDriver:
    def __init__(self):
        rospy.init_node('direct_motor_driver_node')

        # --- Pin Definitions ---
        self.ENA = 33 # PWM for Left Motor
        self.IN1 = 35
        self.IN2 = 37
        self.ENB = 32 # PWM for Right Motor
        self.IN3 = 36
        self.IN4 = 38

        # --- GPIO Setup ---
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup([self.ENA, self.ENB], GPIO.OUT)

        # --- PWM Initialization ---
        self.pwm_left = GPIO.PWM(self.ENA, 100)  # 100 Hz
        self.pwm_right = GPIO.PWM(self.ENB, 100)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        # --- ROS Subscriber ---
        # Subscribes to direct speed commands for left and right motors
        rospy.Subscriber('/motor_speeds', Float32MultiArray, self.speeds_callback)
        rospy.loginfo("Direct Motor Driver ready. Listening to /motor_speeds.")
        rospy.on_shutdown(self.cleanup)

    def speeds_callback(self, msg):
        # Expects a message with data: [left_speed, right_speed]
        # Speeds should be between -1.0 (full reverse) and 1.0 (full forward)
        if len(msg.data) == 2:
            left_speed = msg.data[0]
            right_speed = msg.data[1]

            self.set_motor_speed(self.pwm_left, self.IN1, self.IN2, left_speed)
            self.set_motor_speed(self.pwm_right, self.IN3, self.IN4, right_speed)

    def set_motor_speed(self, pwm, in1, in2, speed):
        # Clamp speed to be between -1.0 and 1.0
        speed = max(min(speed, 1.0), -1.0)
        
        if speed > 0: # Forward
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif speed < 0: # Reverse
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        else: # Stop
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
        
        # Set the PWM duty cycle
        duty_cycle = abs(speed * 100)
        pwm.ChangeDutyCycle(duty_cycle)

    def cleanup(self):
        rospy.loginfo("Cleaning up GPIO...")
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    driver = DirectMotorDriver()
    rospy.spin()
