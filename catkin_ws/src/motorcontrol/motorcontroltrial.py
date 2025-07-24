#!/usr/bin/env python

import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import String
import time

# --- Pin Definitions ---
IN1 = 35  # Left Motor
IN2 = 37
IN3 = 36  # Right Motor
IN4 = 38
ENA = 33  # Enable pins, not used for PWM here
ENB = 32

def setup_gpio():
    """Initializes all GPIO pins for simple ON/OFF control."""
    GPIO.setmode(GPIO.BOARD)
    # Set all pins as outputs
    all_pins = [ENA, IN1, IN2, ENB, IN3, IN4]
    GPIO.setup(all_pins, GPIO.OUT, initial=GPIO.LOW)
    
    # IMPORTANT: Enable both motors at full power by setting ENA/ENB to HIGH
    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(ENB, GPIO.HIGH)
    rospy.loginfo("GPIO pins initialized. Motors enabled at full power.")

def command_callback(msg):
    """Listens for simple text commands and executes them."""
    command = msg.data.lower()
    rospy.loginfo("Received command: %s", command)

    # Set all IN pins to LOW (stop) before every command
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    time.sleep(0.05) # Small delay to prevent motor driver issues

    if command == "forward":
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
    elif command == "backward":
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN4, GPIO.HIGH)
    elif command == "left":
        GPIO.output(IN2, GPIO.HIGH) # Left backward
        GPIO.output(IN3, GPIO.HIGH) # Right forward
    elif command == "right":
        GPIO.output(IN1, GPIO.HIGH) # Left forward
        GPIO.output(IN4, GPIO.HIGH) # Right backward
    # Any other command (like "stop") will result in the motors being off

def motor_tester_node():
    rospy.init_node('motor_wiring_test_node')
    setup_gpio()
    rospy.Subscriber('/motor_command', String, command_callback)
    rospy.loginfo("Motor wiring test node is ready.")
    rospy.loginfo("Publish 'forward', 'backward', 'left', 'right', or 'stop' to /motor_command")
    
    # Ensure GPIO is cleaned up on exit
    rospy.on_shutdown(GPIO.cleanup)
    
    rospy.spin()

if __name__ == '__main__':
    motor_tester_node()
