#!/usr/bin/env python

import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Float32

# This script controls BOTH motors at the same time with one command.
# It's for testing PWM functionality.

# --- Pin Definitions ---
ENA = 33; IN1 = 35; IN2 = 37  # Left Motor
ENB = 32; IN3 = 36; IN4 = 38  # Right Motor

# --- Global PWM Objects ---
pwm_left = None
pwm_right = None

def setup_pwm():
    """Initializes all GPIO pins for PWM control."""
    global pwm_left, pwm_right
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup([ENA, ENB], GPIO.OUT)
    
    pwm_left = GPIO.PWM(ENA, 100)  # 100 Hz frequency
    pwm_right = GPIO.PWM(ENB, 100)
    pwm_left.start(0) # Start at 0% duty cycle
    pwm_right.start(0)
    rospy.loginfo("GPIO and PWM initialized.")

def speed_callback(msg):
    """Receives a single speed value and applies it to both motors."""
    # Speed is a value from -1.0 (full reverse) to 1.0 (full forward)
    speed = msg.data
    speed = max(min(speed, 1.0), -1.0) # Clamp the value
    
    rospy.loginfo("Setting speed to %.2f", speed)
    
    if speed > 0: # Forward
        GPIO.output([IN1, IN3], GPIO.HIGH)
        GPIO.output([IN2, IN4], GPIO.LOW)
    elif speed < 0: # Reverse
        GPIO.output([IN1, IN3], GPIO.LOW)
        GPIO.output([IN2, IN4], GPIO.HIGH)
    else: # Stop
        GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
        
    # Set the PWM duty cycle for both motors
    duty_cycle = abs(speed * 100)
    pwm_left.ChangeDutyCycle(duty_cycle)
    pwm_right.ChangeDutyCycle(duty_cycle)

def pwm_tester_node():
    rospy.init_node('motor_pwm_test_node')
    setup_pwm()
    rospy.Subscriber('/set_speed', Float32, speed_callback)
    rospy.loginfo("Motor PWM test node is ready.")
    rospy.loginfo("Publish a number from -1.0 to 1.0 to /set_speed")
    
    def cleanup():
        rospy.loginfo("Stopping motors and cleaning up GPIO.")
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
    
    rospy.on_shutdown(cleanup)
    rospy.spin()

if __name__ == '__main__':
    pwm_tester_node()
