#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import threading
from collections import deque
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


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
        self.gps_data = None
        self.imu_data = None
        self.last_cmd_vel = None

        # --- Subscribers ---
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)
        rospy.Subscriber('/add_waypoint', NavSatFix, self.waypoint_callback)
        rospy.Subscriber('/add_waypoint_xy', Point, self.waypoint_xy_callback)
        rospy.Subscriber('/waypoint_cmd', String, self.waypoint_command_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        rospy.Subscriber('/add_relative_waypoint', Point, self.relative_waypoint_callback)
        rospy.Subscriber('/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # --- Parameters ---
        self.datum_lat = rospy.get_param('/navsat_transform_node/datum_latitude', 26.2307901667)
        self.datum_lon = rospy.get_param('/navsat_transform_node/datum_longitude', 78.2061421667)

        # --- Navigation Parameters ---
        self.distance_threshold = 0.2  # meters
        self.angular_kp = 0.5
        self.linear_kp = 0.4
        self.max_linear_speed = 0.6  # m/s
        self.max_angular_speed = 1.0  # rad/s

        rospy.loginfo("Waypoint follower initialized.")

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

        # Validate input coordinates
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError("Invalid latitude or longitude values")

        lat1_rad = math.radians(self.datum_lat)
        lon1_rad = math.radians(self.datum_lon)
        lat2_rad = math.radians(lat)
        lon2_rad = math.radians(lon)

        d_lon = lon2_rad - lon1_rad
        d_lat = lat2_rad - lat1_rad

        # Avoid potential numerical issues near poles
        cos_term = math.cos((lat1_rad + lat2_rad) / 2.0)
        if abs(cos_term) < 1e-10:
            rospy.logwarn("GPS conversion near poles, results may be inaccurate")

        map_x = R * d_lon * cos_term
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

    def gps_callback(self, msg):
        """Store latest GPS data."""
        self.gps_data = msg

    def imu_callback(self, msg):
        """Store latest IMU data."""
        self.imu_data = msg

    def math_to_compass_angle(self, math_yaw_rad):
        """Convert mathematical yaw (rad) to compass angle (degrees)."""
        math_degrees = math.degrees(math_yaw_rad)
        compass_degrees = 90.0 - math_degrees

        # Normalize to 0-360 range
        if compass_degrees < 0:
            compass_degrees += 360
        elif compass_degrees >= 360:
            compass_degrees -= 360

        return compass_degrees

    def compass_to_direction(self, compass_degrees):
        """Convert compass degrees to human-readable direction."""
        directions = [
            (0, "North"), (22.5, "NNE"), (45, "Northeast"), (67.5, "ENE"),
            (90, "East"), (112.5, "ESE"), (135, "Southeast"), (157.5, "SSE"),
            (180, "South"), (202.5, "SSW"), (225, "Southwest"), (247.5, "WSW"),
            (270, "West"), (292.5, "WNW"), (315, "Northwest"), (337.5, "NNW"),
            (360, "North")
        ]

        for i, (angle, direction) in enumerate(directions[:-1]):
            next_angle = directions[i + 1][0]
            if angle <= compass_degrees < next_angle:
                return direction
        return "North"

    def log_debug_info(self):
        """Log comprehensive debug information."""
        rospy.loginfo("\n" + "=" * 80)
        rospy.loginfo("ROVER DEBUG STATUS")
        rospy.loginfo("=" * 80)

        # POSE & ORIENTATION
        if self.current_pose and self.current_yaw is not None:
            compass_angle = self.math_to_compass_angle(self.current_yaw)
            direction = self.compass_to_direction(compass_angle)

            rospy.loginfo("POSITION & HEADING:")
            rospy.loginfo("  Map Position: X=%.2fm, Y=%.2fm", self.current_pose.x, self.current_pose.y)
            rospy.loginfo("  Math Yaw: %.3frad (%.1f°)", self.current_yaw, math.degrees(self.current_yaw))
            rospy.loginfo("  Compass: %.1f° (%s)", compass_angle, direction)
        else:
            rospy.loginfo("POSITION: NO POSE DATA")

        # GPS
        if self.gps_data:
            rospy.loginfo("GPS:")
            rospy.loginfo("  Lat: %.8f°", self.gps_data.latitude)
            rospy.loginfo("  Lon: %.8f°", self.gps_data.longitude)
            rospy.loginfo("  Alt: %.2fm", self.gps_data.altitude)
        else:
            rospy.loginfo("GPS: NO GPS DATA")

        # IMU
        if self.imu_data:
            accel = self.imu_data.linear_acceleration
            angular = self.imu_data.angular_velocity
            rospy.loginfo("IMU:")
            rospy.loginfo("  Linear Accel: X=%.3f, Y=%.3f, Z=%.3f m/s²", accel.x, accel.y, accel.z)
            rospy.loginfo("  Angular Vel: X=%.3f, Y=%.3f, Z=%.3f rad/s", angular.x, angular.y, angular.z)
        else:
            rospy.loginfo("IMU: NO IMU DATA")

        # MOTOR COMMANDS
        if self.last_cmd_vel:
            linear_speed = self.last_cmd_vel.linear.x
            angular_speed = self.last_cmd_vel.angular.z
            linear_kmh = linear_speed * 3.6
            angular_dps = math.degrees(angular_speed)

            rospy.loginfo("MOTOR COMMANDS:")
            rospy.loginfo("  Linear Speed: %.3fm/s (%.1fkm/h)", linear_speed, linear_kmh)
            rospy.loginfo("  Angular Speed: %.3frad/s (%.1f°/s)", angular_speed, angular_dps)
            rospy.loginfo("  Direction: %s",
                          'FORWARD' if linear_speed > 0.01 else 'REVERSE' if linear_speed < -0.01 else 'STOPPED')
            rospy.loginfo("  Turning: %s",
                          'RIGHT' if angular_speed < -0.01 else 'LEFT' if angular_speed > 0.01 else 'STRAIGHT')
        else:
            rospy.loginfo("MOTOR COMMANDS: NO COMMAND DATA")

        # WAYPOINTS
        with self.lock:
            waypoint_count = len(self.waypoints)
            rospy.loginfo("WAYPOINTS:")
            rospy.loginfo("  Queue Length: %d", waypoint_count)
            if waypoint_count > 0:
                next_wp = self.waypoints[0]
                rospy.loginfo("  Next Target: [%.2f, %.2f]", next_wp[0], next_wp[1])

                if self.current_pose:
                    dx = next_wp[0] - self.current_pose.x
                    dy = next_wp[1] - self.current_pose.y
                    distance = math.sqrt(dx ** 2 + dy ** 2)
                    target_angle_rad = math.atan2(dy, dx)
                    target_compass = self.math_to_compass_angle(target_angle_rad)
                    target_direction = self.compass_to_direction(target_compass)

                    rospy.loginfo("  Distance to Target: %.2fm", distance)
                    rospy.loginfo("  Target Bearing: %.1f° (%s)", target_compass, target_direction)
            else:
                rospy.loginfo("  Status: IDLE (No waypoints)")

        rospy.loginfo("=" * 80)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        debug_counter = 0

        while not rospy.is_shutdown():
            # Log debug info every 20 cycles (2 Hz)
            debug_counter += 1
            if debug_counter >= 20:
                self.log_debug_info()
                debug_counter = 0

            # Check waypoints with lock
            with self.lock:
                if not self.waypoints:
                    stop_cmd = Twist()  # Create stop command
                    self.last_cmd_vel = stop_cmd
                    self.cmd_vel_pub.publish(stop_cmd)
                    rate.sleep()
                    continue

            if self.current_pose is None or self.current_yaw is None:
                rospy.logwarn("No current pose or yaw available, skipping navigation")
                rate.sleep()
                continue

            # Get target waypoint
            with self.lock:
                target_x, target_y = self.waypoints[0]

            # Publish current target waypoint as PoseStamped
            waypoint_pose = PoseStamped()
            waypoint_pose.header.stamp = rospy.Time.now()
            waypoint_pose.header.frame_id = "map"
            waypoint_pose.pose.position.x = target_x
            waypoint_pose.pose.position.y = target_y
            waypoint_pose.pose.orientation.w = 1.0  # No rotation
            self.waypoint_pub.publish(waypoint_pose)

            # Calculate navigation commands
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

            self.last_cmd_vel = twist_cmd
            self.cmd_vel_pub.publish(twist_cmd)
            rate.sleep()


if __name__ == '__main__':
    try:
        follower = WaypointFollower()
        follower.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint follower node terminated.")
