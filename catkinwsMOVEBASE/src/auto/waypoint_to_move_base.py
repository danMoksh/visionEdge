#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import actionlib
from collections import deque
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class WaypointToMoveBase:
    def __init__(self):
        rospy.init_node('waypoint_to_move_base')
        # Move Base Action Client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Publishers
        self.current_waypoint_pub = rospy.Publisher('/current_waypoint', PoseStamped, queue_size=1)
        # State Variables
        self.waypoints = deque()  # Store as (x, y) in map frame
        self.current_pose = None
        self.current_yaw = None
        self.lock = threading.Lock()
        # Subscribers
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)
        rospy.Subscriber('/add_waypoint', NavSatFix, self.waypoint_callback)
        rospy.Subscriber('/add_waypoint_xy', Point, self.waypoint_xy_callback)
        rospy.Subscriber('/waypoint_cmd', String, self.waypoint_command_callback)
        rospy.Subscriber('/add_relative_waypoint', Point, self.relative_waypoint_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        # Parameters
        self.datum_lat = rospy.get_param('/navsat_transform_node/datum_latitude', 26.2307901667)
        self.datum_lon = rospy.get_param('/navsat_transform_node/datum_longitude', 78.2061421667)
        self.distance_threshold = 0.1  # meters
        rospy.loginfo("Waypoint to Move Base node initialized.")

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.current_yaw = yaw

    def waypoint_callback(self, msg):
        """Convert GPS waypoint to map coordinates and add to queue."""
        try:
            x, y = self.gps_to_map_coordinates(msg.latitude, msg.longitude)
            with self.lock:
                self.waypoints.append((x, y))
                rospy.loginfo("GPS waypoint converted to map coordinates: [%.2f, %.2f]", x, y)
                self.send_next_goal()
        except Exception as e:
            rospy.logerr("Failed to convert GPS waypoint: %s", e)

    def gps_to_map_coordinates(self, lat, lon):
        """Convert GPS lat/lon to map coordinates using equirectangular projection."""
        R = 6378137.0  # Earth's radius in meters
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError("Invalid latitude or longitude values")
        lat1_rad = math.radians(self.datum_lat)
        lon1_rad = math.radians(self.datum_lon)
        lat2_rad = math.radians(lat)
        lon2_rad = math.radians(lon)
        d_lon = lon2_rad - lon1_rad
        d_lat = lat2_rad - lat1_rad
        cos_term = math.cos((lat1_rad + lat2_rad) / 2.0)
        if abs(cos_term) < 1e-10:
            rospy.logwarn("GPS conversion near poles, results may be inaccurate")
        map_x = R * d_lon * cos_term
        map_y = R * d_lat
        return map_x, map_y

    def waypoint_xy_callback(self, msg):
        """Add waypoint directly in map coordinates."""
        with self.lock:
            self.waypoints.append((msg.x, msg.y))
            rospy.loginfo("Added waypoint at XY=[%.2f, %.2f] in map frame", msg.x, msg.y)
            self.send_next_goal()

    def rviz_goal_callback(self, msg):
        """Handle RViz 2D Nav Goal."""
        rospy.loginfo("RViz goal received at [%.2f, %.2f]", msg.pose.position.x, msg.pose.position.y)
        with self.lock:
            self.waypoints.append((msg.pose.position.x, msg.pose.position.y))
            self.send_next_goal()

    def relative_waypoint_callback(self, msg):
        """Add waypoint relative to current position and orientation."""
        if self.current_pose is None or self.current_yaw is None:
            rospy.logwarn("No current pose available for relative waypoint")
            return
        rel_x, rel_y = msg.x, msg.y
        abs_x = self.current_pose.x + rel_x * math.cos(self.current_yaw) - rel_y * math.sin(self.current_yaw)
        abs_y = self.current_pose.y + rel_x * math.sin(self.current_yaw) + rel_y * math.cos(self.current_yaw)
        with self.lock:
            self.waypoints.append((abs_x, abs_y))
            rospy.loginfo("Added relative waypoint [%.2f, %.2f] -> absolute [%.2f, %.2f]", rel_x, rel_y, abs_x, abs_y)
            self.send_next_goal()

    def waypoint_command_callback(self, msg):
        """Handle waypoint commands."""
        command = msg.data.lower().strip()
        if command == "clear":
            with self.lock:
                self.waypoints.clear()
            self.client.cancel_all_goals()
            rospy.loginfo("Waypoint queue cleared and Move Base goals canceled.")
        elif command == "status":
            with self.lock:
                count = len(self.waypoints)
            rospy.loginfo("Waypoint queue has %d waypoints.", count)
            if count > 0:
                rospy.loginfo("Next waypoint: [%.2f, %.2f]", self.waypoints[0][0], self.waypoints[0][1])
        elif command == "stop":
            with self.lock:
                self.waypoints.clear()
            self.client.cancel_all_goals()
            rospy.loginfo("Emergency stop - waypoints cleared and Move Base stopped.")

    def send_next_goal(self):
        """Send the next waypoint as a Move Base goal."""
        with self.lock:
            if not self.waypoints:
                return
            target_x, target_y = self.waypoints[0]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        goal.target_pose.pose.orientation.w = 1.0  # No specific orientation
        self.current_waypoint_pub.publish(goal.target_pose)
        self.client.send_goal(goal, done_cb=self.goal_done_callback)
        rospy.loginfo("Sent Move Base goal: [%.2f, %.2f]", target_x, target_y)

    def goal_done_callback(self, state, result):
        """Callback when a Move Base goal is completed."""
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Waypoint reached.")
            with self.lock:
                if self.waypoints:
                    self.waypoints.popleft()
            self.send_next_goal()
        else:
            rospy.logwarn("Move Base goal failed with state: %d", state)
            # Optionally retry or clear waypoints
            self.send_next_goal()

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        node = WaypointToMoveBase()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint to Move Base node terminated.")#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import actionlib
from collections import deque
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class WaypointToMoveBase:
    def __init__(self):
        rospy.init_node('waypoint_to_move_base')
        # Move Base Action Client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Publishers
        self.current_waypoint_pub = rospy.Publisher('/current_waypoint', PoseStamped, queue_size=1)
        # State Variables
        self.waypoints = deque()  # Store as (x, y) in map frame
        self.current_pose = None
        self.current_yaw = None
        self.lock = threading.Lock()
        # Subscribers
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)
        rospy.Subscriber('/add_waypoint', NavSatFix, self.waypoint_callback)
        rospy.Subscriber('/add_waypoint_xy', Point, self.waypoint_xy_callback)
        rospy.Subscriber('/waypoint_cmd', String, self.waypoint_command_callback)
        rospy.Subscriber('/add_relative_waypoint', Point, self.relative_waypoint_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        # Parameters
        self.datum_lat = rospy.get_param('/navsat_transform_node/datum_latitude', 26.2307901667)
        self.datum_lon = rospy.get_param('/navsat_transform_node/datum_longitude', 78.2061421667)
        self.distance_threshold = 0.1  # meters
        rospy.loginfo("Waypoint to Move Base node initialized.")

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.current_yaw = yaw

    def waypoint_callback(self, msg):
        """Convert GPS waypoint to map coordinates and add to queue."""
        try:
            x, y = self.gps_to_map_coordinates(msg.latitude, msg.longitude)
            with self.lock:
                self.waypoints.append((x, y))
                rospy.loginfo("GPS waypoint converted to map coordinates: [%.2f, %.2f]", x, y)
                self.send_next_goal()
        except Exception as e:
            rospy.logerr("Failed to convert GPS waypoint: %s", e)

    def gps_to_map_coordinates(self, lat, lon):
        """Convert GPS lat/lon to map coordinates using equirectangular projection."""
        R = 6378137.0  # Earth's radius in meters
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError("Invalid latitude or longitude values")
        lat1_rad = math.radians(self.datum_lat)
        lon1_rad = math.radians(self.datum_lon)
        lat2_rad = math.radians(lat)
        lon2_rad = math.radians(lon)
        d_lon = lon2_rad - lon1_rad
        d_lat = lat2_rad - lat1_rad
        cos_term = math.cos((lat1_rad + lat2_rad) / 2.0)
        if abs(cos_term) < 1e-10:
            rospy.logwarn("GPS conversion near poles, results may be inaccurate")
        map_x = R * d_lon * cos_term
        map_y = R * d_lat
        return map_x, map_y

    def waypoint_xy_callback(self, msg):
        """Add waypoint directly in map coordinates."""
        with self.lock:
            self.waypoints.append((msg.x, msg.y))
            rospy.loginfo("Added waypoint at XY=[%.2f, %.2f] in map frame", msg.x, msg.y)
            self.send_next_goal()

    def rviz_goal_callback(self, msg):
        """Handle RViz 2D Nav Goal."""
        rospy.loginfo("RViz goal received at [%.2f, %.2f]", msg.pose.position.x, msg.pose.position.y)
        with self.lock:
            self.waypoints.append((msg.pose.position.x, msg.pose.position.y))
            self.send_next_goal()

    def relative_waypoint_callback(self, msg):
        """Add waypoint relative to current position and orientation."""
        if self.current_pose is None or self.current_yaw is None:
            rospy.logwarn("No current pose available for relative waypoint")
            return
        rel_x, rel_y = msg.x, msg.y
        abs_x = self.current_pose.x + rel_x * math.cos(self.current_yaw) - rel_y * math.sin(self.current_yaw)
        abs_y = self.current_pose.y + rel_x * math.sin(self.current_yaw) + rel_y * math.cos(self.current_yaw)
        with self.lock:
            self.waypoints.append((abs_x, abs_y))
            rospy.loginfo("Added relative waypoint [%.2f, %.2f] -> absolute [%.2f, %.2f]", rel_x, rel_y, abs_x, abs_y)
            self.send_next_goal()

    def waypoint_command_callback(self, msg):
        """Handle waypoint commands."""
        command = msg.data.lower().strip()
        if command == "clear":
            with self.lock:
                self.waypoints.clear()
            self.client.cancel_all_goals()
            rospy.loginfo("Waypoint queue cleared and Move Base goals canceled.")
        elif command == "status":
            with self.lock:
                count = len(self.waypoints)
            rospy.loginfo("Waypoint queue has %d waypoints.", count)
            if count > 0:
                rospy.loginfo("Next waypoint: [%.2f, %.2f]", self.waypoints[0][0], self.waypoints[0][1])
        elif command == "stop":
            with self.lock:
                self.waypoints.clear()
            self.client.cancel_all_goals()
            rospy.loginfo("Emergency stop - waypoints cleared and Move Base stopped.")

    def send_next_goal(self):
        """Send the next waypoint as a Move Base goal."""
        with self.lock:
            if not self.waypoints:
                return
            target_x, target_y = self.waypoints[0]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        goal.target_pose.pose.orientation.w = 1.0  # No specific orientation
        self.current_waypoint_pub.publish(goal.target_pose)
        self.client.send_goal(goal, done_cb=self.goal_done_callback)
        rospy.loginfo("Sent Move Base goal: [%.2f, %.2f]", target_x, target_y)

    def goal_done_callback(self, state, result):
        """Callback when a Move Base goal is completed."""
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Waypoint reached.")
            with self.lock:
                if self.waypoints:
                    self.waypoints.popleft()
            self.send_next_goal()
        else:
            rospy.logwarn("Move Base goal failed with state: %d", state)
            # Optionally retry or clear waypoints
            self.send_next_goal()

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        node = WaypointToMoveBase()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint to Move Base node terminated.")
