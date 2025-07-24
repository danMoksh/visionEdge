#!/usr/bin/env python

# Use this line to ensure that 5 / 2 = 2.5, not 2
from __future__ import division
import rospy
import math
import time
from std_msgs.msg import Float32MultiArray

# --- CONFIGURATION PARAMETERS ---

# A list of waypoints to visit. Each waypoint is a tuple (x, y) in meters.
WAYPOINTS = [
    (0.5, 0.0),   # Go 2 meters forward
   # (2.0, 2.0),   # Go 2 meters to the "left"
   # (0.0, 2.0),   # Go 2 meters "back"
    (0.0, 0.0),   # Return to start
]

# Control gains and thresholds
LINEAR_SPEED = 0.4          # Speed in m/s when moving forward (0.0 to 1.0)
TURNING_SPEED = 0.5         # Speed when turning in place (0.0 to 1.0)
GOAL_THRESHOLD = 0.2        # How close (in meters) to get to a waypoint
ANGLE_THRESHOLD = 0.1       # How close (in radians) to the target angle (approx. 6 degrees)
LOOP_HERTZ = 10             # Frequency of the main control loop

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower_node')
        rospy.loginfo("Waypoint Follower node started.")

        # Publisher for the motor speeds
        self.motor_pub = rospy.Publisher('/motor_speeds', Float32MultiArray, queue_size=10)

        # --- State Variables ---
        self.waypoints = WAYPOINTS
        self.waypoint_index = 0
        self.goal_reached = False

        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!! CRITICAL: SIMULATED ROVER POSE !!!
        # This is a FAKE position for demonstration. In a real robot,
        # you MUST subscribe to an odometry topic (from wheel encoders)
        # or a GPS topic to get the robot's true position and orientation.
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0 # Angle in radians
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        rospy.on_shutdown(self.cleanup)

    def run_loop(self):
        """The main control loop."""
        rate = rospy.Rate(LOOP_HERTZ)

        while not rospy.is_shutdown() and not self.goal_reached:
            # Check if we have visited all waypoints
            if self.waypoint_index >= len(self.waypoints):
                rospy.loginfo("All waypoints reached. Mission complete.")
                self.goal_reached = True
                continue # Skip the rest of the loop

            # Get the current target waypoint
            target_x, target_y = self.waypoints[self.waypoint_index]

            # Calculate distance and angle to the target
            delta_x = target_x - self.current_x
            delta_y = target_y - self.current_y
            distance_to_goal = math.sqrt(delta_x**2 + delta_y**2)
            angle_to_goal = math.atan2(delta_y, delta_x)

            # Calculate the angle error we need to correct
            angle_error = angle_to_goal - self.current_theta

            # Normalize the angle error to be between -PI and PI
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi

            # --- Control Logic ---
            left_speed = 0.0
            right_speed = 0.0

            if distance_to_goal < GOAL_THRESHOLD:
                # We are at the waypoint, move to the next one
                rospy.loginfo("Waypoint {0} reached.".format(self.waypoint_index + 1))
                self.waypoint_index += 1
            elif abs(angle_error) > ANGLE_THRESHOLD:
                # We need to turn to face the waypoint
                rospy.loginfo("Turning to face waypoint...")
                if angle_error > 0: # Turn left
                    left_speed = -TURNING_SPEED
                    right_speed = TURNING_SPEED
                else: # Turn right
                    left_speed = TURNING_SPEED
                    right_speed = -TURNING_SPEED
            else:
                # We are facing the waypoint, move forward
                rospy.loginfo("Moving forward to waypoint...")
                left_speed = LINEAR_SPEED
                right_speed = LINEAR_SPEED

            # Publish the motor commands
            self.publish_speeds(left_speed, right_speed)

            # !!! UPDATE SIMULATED POSITION !!!
            # This is where you would get real data. We are faking it.
            self.update_simulated_pose(left_speed, right_speed)

            rate.sleep()

        # Stop the robot after the loop finishes
        self.stop_robot()

    def update_simulated_pose(self, left_speed, right_speed):
        """
        THIS IS A FAKE UPDATE FUNCTION.
        It pretends to update the robot's position based on motor commands.
        REPLACE THIS with a subscriber to a real odometry/GPS source.
        """
        # This is a highly simplified kinematic model
        dt = 1.0 / LOOP_HERTZ
        v = (left_speed + right_speed) / 2.0 * LINEAR_SPEED # Approximate linear velocity
        omega = (right_speed - left_speed) / 2.0 * TURNING_SPEED # Approximate angular velocity

        self.current_theta += omega * dt
        self.current_x += v * math.cos(self.current_theta) * dt
        self.current_y += v * math.sin(self.current_theta) * dt
        # rospy.loginfo("Simulated Pose: x={0:.2f}, y={1:.2f}, theta={2:.2f}".format(self.current_x, self.current_y, self.current_theta))

    def publish_speeds(self, left, right):
        """Publishes speeds to the /motor_speeds topic."""
        speeds_msg = Float32MultiArray()
        speeds_msg.data = [left, right]
        self.motor_pub.publish(speeds_msg)

    def stop_robot(self):
        rospy.loginfo("Sending stop command.")
        self.publish_speeds(0.0, 0.0)

    def cleanup(self):
        """Called on node shutdown."""
        rospy.loginfo("Shutting down. Stopping robot.")
        self.stop_robot()

if __name__ == '__main__':
    try:
        follower = WaypointFollower()
        follower.run_loop()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint follower node terminated.")
    except Exception as e:
        rospy.logerr("An unhandled exception occurred: {0}".format(e))
