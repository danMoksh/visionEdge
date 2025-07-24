#!/usr/bin/env python

import rospy
import Jetson.GPIO as GPIO
import threading
import time
from std_msgs.msg import Float32MultiArray

class SoftwarePwmDriver:
    def __init__(self):
        rospy.init_node('software_pwm_driver_node')

        # --- Pin Definitions ---
        self.ENA, self.IN1, self.IN2 = 33, 35, 37  # Left Motor
        self.ENB, self.IN3, self.IN4 = 32, 36, 38  # Right Motor

        # --- State Variables ---
        self.motor1_speed = 0
        self.motor2_speed = 0
        self.pwm_running = False

        # --- GPIO Setup ---
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False) # FIX 2: Disable channel usage warnings
        GPIO.setup([self.ENA, self.IN1, self.IN2, self.ENB, self.IN3, self.IN4], GPIO.OUT, initial=GPIO.LOW)

        # --- Start the Software PWM Thread ---
        self.start_pwm_thread()

        # --- ROS Subscriber ---
        rospy.Subscriber('/motor_speeds', Float32MultiArray, self.speeds_callback)
        rospy.loginfo("Software PWM Motor Driver ready. Listening to /motor_speeds.")
        rospy.on_shutdown(self.cleanup)

    def speeds_callback(self, msg):
        if len(msg.data) == 2:
            self.set_motor_speed('left', msg.data[0])
            self.set_motor_speed('right', msg.data[1])

    def set_motor_speed(self, motor_name, speed):
        speed = max(min(speed, 1.0), -1.0)

        if motor_name == 'left':
            if speed > 0:
                GPIO.output(self.IN1, GPIO.HIGH); GPIO.output(self.IN2, GPIO.LOW)
            elif speed < 0:
                GPIO.output(self.IN1, GPIO.LOW); GPIO.output(self.IN2, GPIO.HIGH)
            else:
                GPIO.output(self.IN1, GPIO.LOW); GPIO.output(self.IN2, GPIO.LOW)
            self.motor1_speed = abs(speed * 100)
        
        elif motor_name == 'right':
            if speed > 0:
                GPIO.output(self.IN3, GPIO.HIGH); GPIO.output(self.IN4, GPIO.LOW)
            elif speed < 0:
                GPIO.output(self.IN3, GPIO.LOW); GPIO.output(self.IN4, GPIO.HIGH)
            else:
                GPIO.output(self.IN3, GPIO.LOW); GPIO.output(self.IN4, GPIO.LOW)
            self.motor2_speed = abs(speed * 100)

    def software_pwm_thread(self):
        rospy.loginfo("Software PWM thread started.")
        while self.pwm_running:
            cycle_time = 0.02
            on_time1 = cycle_time * (self.motor1_speed / 100.0)
            off_time1 = cycle_time - on_time1
            on_time2 = cycle_time * (self.motor2_speed / 100.0)
            off_time2 = cycle_time - on_time2

            if self.motor1_speed > 0: GPIO.output(self.ENA, GPIO.HIGH)
            if self.motor2_speed > 0: GPIO.output(self.ENB, GPIO.HIGH)
            time.sleep(min(on_time1, on_time2))

            if on_time1 < on_time2: GPIO.output(self.ENA, GPIO.LOW)
            if on_time2 < on_time1: GPIO.output(self.ENB, GPIO.LOW)
            time.sleep(abs(on_time1 - on_time2))

            GPIO.output(self.ENA, GPIO.LOW)
            GPIO.output(self.ENB, GPIO.LOW)
            time.sleep(min(off_time1, off_time2))

    def start_pwm_thread(self):
        if not self.pwm_running:
            self.pwm_running = True
            # FIX 1: Set daemon property in a separate line for Python 2.7
            self.pwm_thread = threading.Thread(target=self.software_pwm_thread)
            self.pwm_thread.daemon = True
            self.pwm_thread.start()

    def cleanup(self):
        rospy.loginfo("Stopping PWM thread and cleaning up GPIO...")
        self.pwm_running = False
        time.sleep(0.1)
        GPIO.cleanup()

if __name__ == '__main__':
    driver = SoftwarePwmDriver()
    rospy.spin()
