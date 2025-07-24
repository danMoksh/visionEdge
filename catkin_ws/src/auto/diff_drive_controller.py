#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32, Float32MultiArray
import numpy as np

class DiffDriveController:
    def __init__(self):
        rospy.init_node('diff_drive_controller', anonymous=False)

        # Robot physical parameters
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.365)  # Distance between wheels (m)
        self.ticks_per_meter = rospy.get_param('~ticks_per_meter', 1000)    # Encoder ticks per meter
        self.max_speed = rospy.get_param('~max_speed', 0.6)                 # Max linear speed (m/s)
        self.rate = rospy.get_param('~rate', 50)                            # Publish rate (Hz)
        self.timeout = rospy.get_param('~timeout', 0.2)                     # Timeout for cmd_vel (s)

        # PID parameters
        self.Kp = rospy.get_param('~Kp', 10.0)
        self.Ki = rospy.get_param('~Ki', 10.0)
        self.Kd = rospy.get_param('~Kd', 0.001)
        self.out_min = rospy.get_param('~out_min', -1000)
        self.out_max = rospy.get_param('~out_max', 1000)

        # Motor trim
        self.right_motor_trim = rospy.get_param('~right_motor_trim', 1.34)

        # Publishers and Subscribers
        # self.lwheel_pub = rospy.Publisher('~lwheel_desired_rate', Int32, queue_size=1)
        # self.rwheel_pub = rospy.Publisher('~rwheel_desired_rate', Int32, queue_size=1)
	self.motor_pub = rospy.Publisher('/motor_speeds', Float32MultiArray, queue_size=1)

        rospy.Subscriber('~cmd_vel', Twist, self.cmd_vel_callback)
        self.lwheel_ticks_sub = rospy.Subscriber('~lwheel_ticks', Int32, self.lwheel_ticks_callback)
        self.rwheel_ticks_sub = rospy.Subscriber('~rwheel_ticks', Int32, self.rwheel_ticks_callback)
        self.lwheel_rate_sub = rospy.Subscriber('~lwheel_rate', Float32, self.lwheel_rate_callback)
        self.rwheel_rate_sub = rospy.Subscriber('~rwheel_rate', Float32, self.rwheel_rate_callback)

        # PID state
        self.last_error = {'left': 0.0, 'right': 0.0}
        self.integral = {'left': 0.0, 'right': 0.0}
        self.current_ticks = {'left': 0, 'right': 0}
        self.current_rates = {'left': 0.0, 'right': 0.0}
        self.target_rates = {'left': 0.0, 'right': 0.0}  # Initialize target_rates
        self.last_time = rospy.get_rostime()
        self.last_cmd_time = rospy.get_rostime()

        # Initialize message objects
        self.lwheel_msg = Int32()
        self.rwheel_msg = Int32()

        rospy.loginfo("Differential Drive Controller Initialized.")
        rospy.loginfo("Wheel Separation: %.2f m", self.wheel_separation)
        rospy.loginfo("Ticks per Meter: %d", self.ticks_per_meter)
        rospy.loginfo("Max Speed: %.2f m/s", self.max_speed)
        rospy.loginfo("Publish Rate: %d Hz", self.rate)
        rospy.loginfo("Timeout: %.2f s", self.timeout)

        # Start control loop
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.control_loop)

    def cmd_vel_callback(self, msg):
        """Handle incoming cmd_vel messages."""
        self.last_cmd_time = rospy.get_rostime()
        
        # Convert linear and angular velocities to wheel velocities (m/s)
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        v_right = linear_vel + (angular_vel * self.wheel_separation / 2.0)
        v_left = linear_vel - (angular_vel * self.wheel_separation / 2.0)

        # Convert to ticks/second
        self.target_rates = {
            'left': v_left * self.ticks_per_meter,
            'right': v_right * self.ticks_per_meter 
        }

    def lwheel_ticks_callback(self, msg):
        """Store left wheel ticks."""
        self.current_ticks['left'] = msg.data

    def rwheel_ticks_callback(self, msg):
        """Store right wheel ticks."""
        self.current_ticks['right'] = msg.data

    def lwheel_rate_callback(self, msg):
        """Store left wheel rate."""
        self.current_rates['left'] = msg.data

    def rwheel_rate_callback(self, msg):
        """Store right wheel rate."""
        self.current_rates['right'] = msg.data

    def pid_control(self, target_rate, side):
        """Apply PID control to achieve target wheel rate."""
        current_time = rospy.get_rostime()
        dt = (current_time - self.last_time).to_sec()

        # Use actual rate from encoder feedback
        current_rate = self.current_rates[side]

        error = target_rate - current_rate
        self.integral[side] += error * dt
        # --- Add this line to clamp the integral term ---
        self.integral[side] = max(min(self.integral[side], self.out_max), self.out_min)
        # --------------------------------------------------
        derivative = (error - self.last_error[side]) / dt if dt > 0 else 0.0

        # PID output
        output = (self.Kp * error + 
                 self.Ki * self.integral[side] + 
                 self.Kd * derivative)

        # Clamp output
        output = max(min(output, self.out_max), self.out_min)

        self.last_error[side] = error
        return int(output)

    def control_loop(self, event):
        """Main control loop running at specified rate."""
        # Check for timeout and set zero velocities if timed out
        if (rospy.get_rostime() - self.last_cmd_time).to_sec() > self.timeout:
            self.target_rates = {'left': 0.0, 'right': 0.0}

	# 1. Get the raw integer output from the PID controller and store it
	lwheel_pid_out = self.pid_control(self.target_rates['left'], 'left')
	rwheel_pid_out = self.pid_control(self.target_rates['right'], 'right')
	
	rwheel_pid_out_trimmed = rwheel_pid_out * self.right_motor_trim
	        
	# Normalize the PID output to the range [-1.0, 1.0]
        # We divide by out_max which was set in the launch file (1000)
        lwheel_speed_normalized = float(lwheel_pid_out) / self.out_max
        rwheel_speed_normalized = float(rwheel_pid_out_trimmed) / self.out_max

        # Create the message
        motor_speeds_msg = Float32MultiArray()
        motor_speeds_msg.data = [lwheel_speed_normalized, rwheel_speed_normalized]

        # Publish the single message
        self.motor_pub.publish(motor_speeds_msg)

        self.last_time = rospy.get_rostime()

    def shutdown(self):
        """Cleanup on node shutdown."""
        self.lwheel_msg.data = 0
        self.rwheel_msg.data = 0
        self.lwheel_pub.publish(self.lwheel_msg)
        self.rwheel_pub.publish(self.rwheel_msg)
        self.timer.shutdown()

if __name__ == '__main__':
    try:
        controller = DiffDriveController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
