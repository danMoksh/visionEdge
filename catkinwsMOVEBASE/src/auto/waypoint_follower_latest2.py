#!/usr/bin/env python

import rospy
import math
import threading
from collections import deque
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, PoseStamped
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import tf2_ros


class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower')

        # --- Publishers ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/current_waypoint', PoseStamped, queue_size=1)

        # --- State Variables ---
        self.waypoints = deque()  # Store as (x, y) in map frame
        self.lock = threading.Lock()
        self.current_pose = None
        self.current_yaw = None

        # --- TF2 Buffer for coordinate transformations ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- Subscribers ---
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)
        rospy.Subscriber('/add_waypoint', NavSatFix, self.waypoint_callback)
        rospy.Subscriber('/add_waypoint_xy', Point, self.waypoint_xy_callback)
        rospy.Subscriber('/waypoint_cmd', String, self.waypoint_command_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        # Add this line with your other subscribers
        rospy.Subscriber('/add_relative_waypoint', Point, self.relative_waypoint_callback)

        # --- Parameters ---
        self.datum_lat = rospy.get_param('/navsat_transform_node/datum_latitude', 26.2307901667)
        self.datum_lon = rospy.get_param('/navsat_transform_node/datum_longitude', 78.2061421667)

        # --- Navigation Parameters ---
        self.distance_threshold = 0  # meters
        self.angular_kp = 1.0
        self.linear_kp = 0.4
        self.max_linear_speed = 0.6  # m/s
        self.max_angular_speed = 1.0  # rad/s

        rospy.loginfo("Waypoint follower initialized. Waiting for transforms...")

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.current_yaw = yaw

    def waypoint_callback(self, msg):
        """Receives a NavSatFix message and converts it using GPS to local coordinates."""
        rospy.loginfo("New GPS waypoint received: Lat=%f, Lon=%f", msg.latitude, msg.longitude)

        try:
            x, y = self.gps_to_map_coordinates(msg.latitude, msg.longitude)
            with self.lock:
                self.waypoints.append((x, y))
                rospy.loginfo("GPS waypoint converted to map coordinates: [%.2f, %.2f]", x, y)
        except Exception as e:
            rospy.logerr("Failed to convert GPS waypoint: %s", e)

    def gps_to_map_coordinates(self, lat, lon):
        """Convert GPS lat/lon to map coordinates using simple equirectangular projection."""
        R = 6378137.0  # Earth's radius in meters

        lat1_rad = math.radians(self.datum_lat)
        lon1_rad = math.radians(self.datum_lon)
        lat2_rad = math.radians(lat)
        lon2_rad = math.radians(lon)

        d_lon = lon2_rad - lon1_rad
        d_lat = lat2_rad - lat1_rad

        map_x = R * d_lon * math.cos((lat1_rad + lat2_rad) / 2.0)
        map_y = R * d_lat

        return map_x, map_y

    def rviz_goal_callback(self, msg):
        """Handle goals sent from RViz 2D Nav Goal tool."""
        rospy.loginfo("RViz goal received at [%.2f, %.2f]", msg.pose.position.x, msg.pose.position.y)
        self.add_waypoint_xy(msg.pose.position.x, msg.pose.position.y)

    def waypoint_xy_callback(self, msg):
        """Add waypoint directly in map coordinates."""
        self.add_waypoint_xy(msg.x, msg.y)

    def waypoint_command_callback(self, msg):
        """Handle waypoint commands like 'clear', 'status', etc."""
        command = msg.data.lower().strip()

        if command == "clear":
            with self.lock:
                self.waypoints.clear()
            rospy.loginfo("Waypoint queue cleared.")

        elif command == "status":
            with self.lock:
                count = len(self.waypoints)
            rospy.loginfo("Waypoint queue has %d waypoints.", count)
            if count > 0:
                rospy.loginfo("Next waypoint: [%.2f, %.2f]", self.waypoints[0][0], self.waypoints[0][1])

        elif command == "stop":
            with self.lock:
                self.waypoints.clear()
            self.cmd_vel_pub.publish(Twist())  # Stop the robot
            rospy.loginfo("Emergency stop - waypoints cleared and robot stopped.")

    def add_waypoint_xy(self, x, y):
        """Add waypoint directly in map coordinates (for testing/manual input)."""
        with self.lock:
            self.waypoints.append((x, y))
            rospy.loginfo("Added waypoint at XY=[%.2f, %.2f] in map frame", x, y)

    def relative_waypoint_callback(self, msg):
        """Add waypoint relative to current robot position and orientation."""
        if self.current_pose is None or self.current_yaw is None:
            rospy.logwarn("No current pose available for relative waypoint")
            return

	rospy.loginfo("Current pose: [%.2f, %.2f], yaw: %.2f rad (%.1f deg)", 
                  self.current_pose.x, self.current_pose.y, 
                  self.current_yaw, math.degrees(self.current_yaw))

        # Transform relative coordinates to absolute map coordinates  
        rel_x, rel_y = msg.x, msg.y
        abs_x = self.current_pose.x + rel_x * math.cos(self.current_yaw) - rel_y * math.sin(self.current_yaw)
        abs_y = self.current_pose.y + rel_x * math.sin(self.current_yaw) + rel_y * math.cos(self.current_yaw)

        self.add_waypoint_xy(abs_x, abs_y)
        rospy.loginfo("Added relative waypoint [%.2f, %.2f] -> absolute [%.2f, %.2f]", rel_x, rel_y, abs_x, abs_y)
	

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            with self.lock:
                if not self.waypoints:
                    if self.current_pose is not None:
                        self.cmd_vel_pub.publish(Twist())  # Stop if idle
                    rate.sleep()
                    continue

            if self.current_pose is None:
                rate.sleep()
                continue

            target_x, target_y = self.waypoints[0]

            # --- Publish current target waypoint as PoseStamped ---
            waypoint_pose = PoseStamped()
            waypoint_pose.header.stamp = rospy.Time.now()
            waypoint_pose.header.frame_id = "map"
            waypoint_pose.pose.position.x = target_x
            waypoint_pose.pose.position.y = target_y
            waypoint_pose.pose.orientation.w = 1.0  # No rotation
            self.waypoint_pub.publish(waypoint_pose)
            # --- End waypoint publish block ---

            dx = target_x - self.current_pose.x
            dy = target_y - self.current_pose.y
            distance_to_target = math.sqrt(dx ** 2 + dy ** 2)
            angle_to_target = math.atan2(dy, dx)

            if distance_to_target < self.distance_threshold:
                rospy.loginfo("Waypoint reached. Removing from queue.")
                with self.lock:
                    self.waypoints.popleft()
                continue

            angle_error = angle_to_target - self.current_yaw
            if angle_error > math.pi:
                angle_error -= 2 * math.pi
            elif angle_error < -math.pi:
                angle_error += 2 * math.pi

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
#!/usr/bin/env python

import rospy
import math
import threading
from collections import deque
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, PoseStamped
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import tf2_ros


class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower')

        # --- Publishers ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/current_waypoint', PoseStamped, queue_size=1)

        # --- State Variables ---
        self.waypoints = deque()  # Store as (x, y) in map frame
        self.lock = threading.Lock()
        self.current_pose = None
        self.current_yaw = None

        # --- TF2 Buffer for coordinate transformations ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- Subscribers ---
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)
        rospy.Subscriber('/add_waypoint', NavSatFix, self.waypoint_callback)
        rospy.Subscriber('/add_waypoint_xy', Point, self.waypoint_xy_callback)
        rospy.Subscriber('/waypoint_cmd', String, self.waypoint_command_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        # Add this line with your other subscribers
        rospy.Subscriber('/add_relative_waypoint', Point, self.relative_waypoint_callback)

        # --- Parameters ---
        self.datum_lat = rospy.get_param('/navsat_transform_node/datum_latitude', 26.2307901667)
        self.datum_lon = rospy.get_param('/navsat_transform_node/datum_longitude', 78.2061421667)

        # --- Navigation Parameters ---
        self.distance_threshold = 1.5  # meters
        self.angular_kp = 1.5
        self.linear_kp = 0.4
        self.max_linear_speed = 0.6  # m/s
        self.max_angular_speed = 1.0  # rad/s

        rospy.loginfo("Waypoint follower initialized. Waiting for transforms...")

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.current_yaw = yaw

    def waypoint_callback(self, msg):
        """Receives a NavSatFix message and converts it using GPS to local coordinates."""
        rospy.loginfo("New GPS waypoint received: Lat=%f, Lon=%f", msg.latitude, msg.longitude)

        try:
            x, y = self.gps_to_map_coordinates(msg.latitude, msg.longitude)
            with self.lock:
                self.waypoints.append((x, y))
                rospy.loginfo("GPS waypoint converted to map coordinates: [%.2f, %.2f]", x, y)
        except Exception as e:
            rospy.logerr("Failed to convert GPS waypoint: %s", e)

    def gps_to_map_coordinates(self, lat, lon):
        """Convert GPS lat/lon to map coordinates using simple equirectangular projection."""
        R = 6378137.0  # Earth's radius in meters

        lat1_rad = math.radians(self.datum_lat)
        lon1_rad = math.radians(self.datum_lon)
        lat2_rad = math.radians(lat)
        lon2_rad = math.radians(lon)

        d_lon = lon2_rad - lon1_rad
        d_lat = lat2_rad - lat1_rad

        map_x = R * d_lon * math.cos((lat1_rad + lat2_rad) / 2.0)
        map_y = R * d_lat

        return map_x, map_y

    def rviz_goal_callback(self, msg):
        """Handle goals sent from RViz 2D Nav Goal tool."""
        rospy.loginfo("RViz goal received at [%.2f, %.2f]", msg.pose.position.x, msg.pose.position.y)
        self.add_waypoint_xy(msg.pose.position.x, msg.pose.position.y)

    def waypoint_xy_callback(self, msg):
        """Add waypoint directly in map coordinates."""
        self.add_waypoint_xy(msg.x, msg.y)

    def waypoint_command_callback(self, msg):
        """Handle waypoint commands like 'clear', 'status', etc."""
        command = msg.data.lower().strip()

        if command == "clear":
            with self.lock:
                self.waypoints.clear()
            rospy.loginfo("Waypoint queue cleared.")

        elif command == "status":
            with self.lock:
                count = len(self.waypoints)
            rospy.loginfo("Waypoint queue has %d waypoints.", count)
            if count > 0:
                rospy.loginfo("Next waypoint: [%.2f, %.2f]", self.waypoints[0][0], self.waypoints[0][1])

        elif command == "stop":
            with self.lock:
                self.waypoints.clear()
            self.cmd_vel_pub.publish(Twist())  # Stop the robot
            rospy.loginfo("Emergency stop - waypoints cleared and robot stopped.")

    def add_waypoint_xy(self, x, y):
        """Add waypoint directly in map coordinates (for testing/manual input)."""
        with self.lock:
            self.waypoints.append((x, y))
            rospy.loginfo("Added waypoint at XY=[%.2f, %.2f] in map frame", x, y)

    def relative_waypoint_callback(self, msg):
        """Add waypoint relative to current robot position and orientation."""
        if self.current_pose is None or self.current_yaw is None:
            rospy.logwarn("No current pose available for relative waypoint")
            return

        # Transform relative coordinates to absolute map coordinates  
        rel_x, rel_y = msg.x, msg.y
        abs_x = self.current_pose.x + rel_x * math.cos(self.current_yaw) - rel_y * math.sin(self.current_yaw)
        abs_y = self.current_pose.y + rel_x * math.sin(self.current_yaw) + rel_y * math.cos(self.current_yaw)

        self.add_waypoint_xy(abs_x, abs_y)
        rospy.loginfo("Added relative waypoint [%.2f, %.2f] -> absolute [%.2f, %.2f]", rel_x, rel_y, abs_x, abs_y)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            with self.lock:
                if not self.waypoints:
                    if self.current_pose is not None:
                        self.cmd_vel_pub.publish(Twist())  # Stop if idle
                    rate.sleep()
                    continue

            if self.current_pose is None:
                rate.sleep()
                continue

            target_x, target_y = self.waypoints[0]

            # --- Publish current target waypoint as PoseStamped ---
            waypoint_pose = PoseStamped()
            waypoint_pose.header.stamp = rospy.Time.now()
            waypoint_pose.header.frame_id = "map"
            waypoint_pose.pose.position.x = target_x
            waypoint_pose.pose.position.y = target_y
            waypoint_pose.pose.orientation.w = 1.0  # No rotation
            self.waypoint_pub.publish(waypoint_pose)
            # --- End waypoint publish block ---

            dx = target_x - self.current_pose.x
            dy = target_y - self.current_pose.y
            distance_to_target = math.sqrt(dx ** 2 + dy ** 2)
            angle_to_target = math.atan2(dy, dx)

            if distance_to_target < self.distance_threshold:
                rospy.loginfo("Waypoint reached. Removing from queue.")
                with self.lock:
                    self.waypoints.popleft()
                continue

            angle_error = angle_to_target - self.current_yaw
            if angle_error > math.pi:
                angle_error -= 2 * math.pi
            elif angle_error < -math.pi:
                angle_error += 2 * math.pi

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

