#!/usr/bin/env python

import rospy
import math
import threading
from collections import deque
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower')

        # --- State Variables ---
        self.waypoints = deque() # Use a deque as a thread-safe queue
        self.lock = threading.Lock()
        self.current_pose = None
        self.current_yaw = None

        # --- Get Datum (Origin) from Parameter Server ---
        # This MUST match the datum in your launch file for navsat_transform_node
        self.origin_lat = rospy.get_param('/navsat_transform_node/datum_latitude', 0.0)
        self.origin_lon = rospy.get_param('/navsat_transform_node/datum_longitude', 0.0)
        rospy.loginfo("Waypoint follower initialized with origin: Lat=%f, Lon=%f", self.origin_lat, self.origin_lon)

        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)
        rospy.Subscriber('/add_waypoint', NavSatFix, self.waypoint_callback) # New subscriber

        # --- Navigation Parameters ---
        self.distance_threshold = 1.5  # meters
        self.angular_kp = 1.5
        self.linear_kp = 0.4
        self.max_linear_speed = 0.6  # m/s
        self.max_angular_speed = 1.0   # rad/s

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw

    def waypoint_callback(self, msg):
        """Receives a NavSatFix message and adds it to the waypoint queue."""
        rospy.loginfo("New waypoint received: Lat=%f, Lon=%f", msg.latitude, msg.longitude)
        with self.lock:
            # Convert the incoming Lat/Lon to local XY coordinates
            x, y = self.latlon_to_xy(msg.latitude, msg.longitude)
            self.waypoints.append((x, y))
            rospy.loginfo("Converted to XY=[%.2f, %.2f] and added to queue.", x, y)

    def latlon_to_xy(self, lat2, lon2):
        """Converts lat/lon to meters relative to the origin datum."""
        R = 6378137.0  # Earth's radius in meters
        lat1_rad = math.radians(self.origin_lat)
        lon1_rad = math.radians(self.origin_lon)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        d_lon = lon2_rad - lon1_rad
        d_lat = lat2_rad - lat1_rad

        # This is a simplified equirectangular projection, good for local distances
        x = R * d_lon * math.cos((lat1_rad + lat2_rad) / 2)
        y = R * d_lat
        return x, y

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            with self.lock:
                if not self.waypoints: # If the queue is empty
                    if self.current_pose is not None: # And we are not moving
                        self.cmd_vel_pub.publish(Twist()) # Publish a stop command
                    rate.sleep()
                    continue

            if self.current_pose is None:
                rate.sleep()
                continue

            # Get the next waypoint from the front of the queue
            target_x, target_y = self.waypoints[0]

            dx = target_x - self.current_pose.x
            dy = target_y - self.current_pose.y
            distance_to_target = math.sqrt(dx**2 + dy**2)
            angle_to_target = math.atan2(dy, dx)

            if distance_to_target < self.distance_threshold:
                rospy.loginfo("Waypoint reached. Removing from queue.")
                with self.lock:
                    self.waypoints.popleft() # Remove the reached waypoint
                continue

            angle_error = angle_to_target - self.current_yaw
            if angle_error > math.pi: angle_error -= 2 * math.pi
            if angle_error < -math.pi: angle_error += 2 * math.pi

            twist_cmd = Twist()
            angular_vel = self.angular_kp * angle_error
            twist_cmd.angular.z = max(min(angular_vel, self.max_angular_speed), -self.max_angular_speed)

            if abs(angle_error) < math.pi / 4.0:
                linear_vel = self.linear_kp * distance_to_target
                twist_cmd.linear.x = min(linear_vel, self.max_linear_speed)
            
            self.cmd_vel_pub.publish(twist_cmd)
            rate.sleep()

if __name__ == '__main__':
    follower = WaypointFollower()
    follower.run()
