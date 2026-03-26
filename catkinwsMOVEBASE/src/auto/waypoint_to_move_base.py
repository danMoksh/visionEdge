#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS Move Base Actionlib Bridge for Autonomous FOD Removal Rover.
Receives GPS/XY/Relative waypoints, manages an execution queue, and
dispatches navigation goals to the ROS move_base action server.
"""

import rospy
import math
import actionlib
import threading
from collections import deque

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback


class WaypointToMoveBase:
    def __init__(self):
        rospy.init_node('waypoint_to_move_base')

        # Action Client for ROS Navigation Stack
        self.action_server_name = rospy.get_param('~action_server_name', 'move_base')
        rospy.loginfo("Connecting to [%s] action server...", self.action_server_name)
        self.client = actionlib.SimpleActionClient(self.action_server_name, MoveBaseAction)
        
        # Wait up to 10 seconds for move_base action server
        server_connected = self.client.wait_for_server(rospy.Duration(10.0))
        if not server_connected:
            rospy.logwarn("Action server [%s] not immediately available. Will retry on dispatch.",
                          self.action_server_name)

        # Publishers
        self.current_waypoint_pub = rospy.Publisher('/current_waypoint', PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher('/waypoint_status', String, queue_size=10)

        # State Variables & Thread Safety
        self.waypoints = deque()  # Store as (x, y, orientation_rad or None) in map frame
        self.current_pose = None
        self.current_yaw = None
        self.is_navigating = False
        self.paused = False
        self.lock = threading.Lock()

        # Subscribers
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback, queue_size=1)
        rospy.Subscriber('/add_waypoint', NavSatFix, self.waypoint_callback, queue_size=10)
        rospy.Subscriber('/add_waypoint_xy', Point, self.waypoint_xy_callback, queue_size=10)
        rospy.Subscriber('/waypoint_cmd', String, self.waypoint_command_callback, queue_size=10)
        rospy.Subscriber('/add_relative_waypoint', Point, self.relative_waypoint_callback, queue_size=10)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback, queue_size=10)

        # Parameters
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.datum_lat = rospy.get_param('/navsat_transform_node/datum_latitude', 26.2307901667)
        self.datum_lon = rospy.get_param('/navsat_transform_node/datum_longitude', 78.2061421667)
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.15)  # meters
        self.align_to_path = rospy.get_param('~align_to_path', True)

        rospy.loginfo("Waypoint to Move Base node initialized successfully.")

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
                self.waypoints.append((x, y, None))
                rospy.loginfo("Enqueued GPS waypoint lat/lon [%.7f, %.7f] -> map [%.2f, %.2f]",
                              msg.latitude, msg.longitude, x, y)
            self.send_next_goal()
        except Exception as e:
            rospy.logerr("Failed to convert GPS waypoint: %s", e)

    def gps_to_map_coordinates(self, lat, lon):
        """Convert GPS lat/lon to map coordinates using equirectangular projection."""
        R = 6378137.0  # Earth's equatorial radius in meters
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError("Invalid latitude [%f] or longitude [%f] values" % (lat, lon))
        lat1_rad = math.radians(self.datum_lat)
        lon1_rad = math.radians(self.datum_lon)
        lat2_rad = math.radians(lat)
        lon2_rad = math.radians(lon)
        d_lon = lon2_rad - lon1_rad
        d_lat = lat2_rad - lat1_rad
        cos_mean_lat = math.cos((lat1_rad + lat2_rad) / 2.0)
        
        map_x = R * d_lon * cos_mean_lat
        map_y = R * d_lat
        return map_x, map_y

    def waypoint_xy_callback(self, msg):
        """Add waypoint directly in map coordinates."""
        with self.lock:
            self.waypoints.append((msg.x, msg.y, None))
            rospy.loginfo("Enqueued map waypoint at XY=[%.2f, %.2f]", msg.x, msg.y)
        self.send_next_goal()

    def rviz_goal_callback(self, msg):
        """Handle RViz 2D Nav Goal."""
        q = msg.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        with self.lock:
            self.waypoints.append((msg.pose.position.x, msg.pose.position.y, yaw))
            rospy.loginfo("Enqueued RViz 2D Nav Goal at [%.2f, %.2f] yaw=%.2f rad",
                          msg.pose.position.x, msg.pose.position.y, yaw)
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
            self.waypoints.append((abs_x, abs_y, None))
            rospy.loginfo("Added relative waypoint [%.2f, %.2f] -> absolute [%.2f, %.2f]",
                          rel_x, rel_y, abs_x, abs_y)
        self.send_next_goal()

    def waypoint_command_callback(self, msg):
        """Handle waypoint queue and navigation commands."""
        command = msg.data.lower().strip()
        rospy.loginfo("Received waypoint command: %s", command)
        
        if command == "clear":
            with self.lock:
                self.waypoints.clear()
                self.is_navigating = False
            self.client.cancel_all_goals()
            rospy.loginfo("Waypoint queue cleared and active Move Base goals canceled.")
            self.publish_status("CLEARED")
            
        elif command == "pause":
            with self.lock:
                self.paused = True
            self.client.cancel_all_goals()
            rospy.loginfo("Navigation paused.")
            self.publish_status("PAUSED")

        elif command == "resume":
            with self.lock:
                self.paused = False
            rospy.loginfo("Navigation resumed.")
            self.send_next_goal()

        elif command == "stop":
            with self.lock:
                self.waypoints.clear()
                self.is_navigating = False
                self.paused = False
            self.client.cancel_all_goals()
            rospy.loginfo("Emergency stop triggered - goals canceled and queue cleared.")
            self.publish_status("STOPPED")

        elif command == "status":
            with self.lock:
                count = len(self.waypoints)
                nav_state = "NAVIGATING" if self.is_navigating else ("PAUSED" if self.paused else "IDLE")
            rospy.loginfo("Waypoint queue: %d pending | State: %s", count, nav_state)
            self.publish_status(nav_state)

    def send_next_goal(self):
        """Send the next waypoint as a Move Base goal if idle."""
        with self.lock:
            if self.paused:
                rospy.loginfo("Waypoint dispatch skipped: currently paused.")
                return
            if self.is_navigating:
                rospy.logdebug("Waypoint dispatch skipped: already navigating to a goal.")
                return
            if not self.waypoints:
                rospy.loginfo("All waypoints reached. Navigation queue is empty.")
                self.publish_status("IDLE")
                return

            target_x, target_y, target_yaw = self.waypoints[0]
            self.is_navigating = True

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        goal.target_pose.pose.position.z = 0.0

        # Calculate approach orientation
        if target_yaw is not None:
            q = quaternion_from_euler(0, 0, target_yaw)
        elif self.align_to_path and self.current_pose is not None:
            heading = math.atan2(target_y - self.current_pose.y, target_x - self.current_pose.x)
            q = quaternion_from_euler(0, 0, heading)
        else:
            q = [0.0, 0.0, 0.0, 1.0]

        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.current_waypoint_pub.publish(goal.target_pose)
        rospy.loginfo("Dispatching Move Base goal: [%.2f, %.2f] (remaining queue: %d)",
                      target_x, target_y, len(self.waypoints))

        self.client.send_goal(
            goal,
            done_cb=self.goal_done_callback,
            feedback_cb=self.goal_feedback_callback
        )
        self.publish_status("NAVIGATING")

    def goal_feedback_callback(self, feedback):
        """Feedback callback tracking distance to active goal."""
        if self.current_pose is not None:
            with self.lock:
                if self.waypoints:
                    target_x, target_y, _ = self.waypoints[0]
                    dx = target_x - self.current_pose.x
                    dy = target_y - self.current_pose.y
                    dist = math.hypot(dx, dy)
                    rospy.logdebug_throttle(2.0, "Distance to goal: %.2f m", dist)

    def goal_done_callback(self, state, result):
        """Callback when a Move Base goal completes."""
        with self.lock:
            self.is_navigating = False
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Waypoint successfully reached!")
                if self.waypoints:
                    self.waypoints.popleft()
            elif state == GoalStatus.PREEMPTED:
                rospy.loginfo("Move Base goal was preempted/cancelled.")
            elif state == GoalStatus.ABORTED:
                rospy.logwarn("Move Base goal aborted by navigation planner. Retrying next goal.")
                if self.waypoints:
                    self.waypoints.popleft()
            else:
                rospy.logwarn("Move Base goal finished with terminal state: %d", state)
                if self.waypoints:
                    self.waypoints.popleft()

        # Dispatch next waypoint in queue
        self.send_next_goal()

    def publish_status(self, state_str):
        with self.lock:
            pending = len(self.waypoints)
        msg = String()
        msg.data = f"STATE:{state_str}|PENDING:{pending}"
        self.status_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = WaypointToMoveBase()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint to Move Base node terminated.")
