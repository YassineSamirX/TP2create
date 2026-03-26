"""
NavigateToWaypoint / NavigateToDockVicinity — Action Nodes
===========================================================
Custom proportional controller for go-to-pose navigation.
Uses /odom for position tracking and /scan for reactive obstacle avoidance.

This avoids depending on Nav2 for simplicity while demonstrating
the navigation concepts. The same logic can be replaced with
Nav2's NavigateToPose action client for production use.

Reuses Assignment 2's _navigate_to_point() concept but adapted
for Create3 odometry (Odometry msg instead of turtlesim Pose).
"""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from create3_bt.bt_framework import ActionNode, NodeStatus


class NavigateToWaypoint(ActionNode):
    """
    Navigate to target_pose from blackboard using proportional control.
    Includes reactive obstacle detection from /scan.
    """

    def on_init(self):
        self._cmd_vel_pub = self.ros_node.create_publisher(
            Twist, '/cmd_vel_nav', 10)

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0
        self._got_odom = False

        self._front_range = float('inf')

        self._odom_sub = self.ros_node.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self._scan_sub = self.ros_node.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)

        self._start_time = None
        self._timeout = 60.0  # seconds before giving up
        self.ros_node.get_logger().info('[BT] NavigateToWaypoint: ready')

    def _odom_cb(self, msg):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self._got_odom = True

    def _scan_cb(self, msg):
        """Get minimum range in front sector (~±15°)."""
        if len(msg.ranges) == 0:
            return
        n = len(msg.ranges)
        front_ranges = []
        for deg in list(range(0, 15)) + list(range(n - 15, n)):
            r = msg.ranges[deg % n]
            if not math.isinf(r) and not math.isnan(r) and r > 0.05:
                front_ranges.append(r)
        self._front_range = min(front_ranges) if front_ranges else float('inf')

    def on_start(self):
        target = self.blackboard.get('target_pose')
        if target is None:
            self.ros_node.get_logger().error('[BT] NavigateToWaypoint: no target_pose!')
            return NodeStatus.FAILURE
        self._start_time = time.time()
        self.ros_node.get_logger().info(
            f'[BT] NavigateToWaypoint: heading to ({target["x"]:.2f}, {target["y"]:.2f})')
        return NodeStatus.RUNNING

    def on_running(self):
        if not self._got_odom:
            return NodeStatus.RUNNING

        target = self.blackboard.get('target_pose')
        if target is None:
            return NodeStatus.FAILURE

        # Check timeout
        if time.time() - self._start_time > self._timeout:
            self.ros_node.get_logger().warn('[BT] NavigateToWaypoint: timeout!')
            self._stop()
            return NodeStatus.FAILURE

        # Check obstacle ahead — if blocked, return FAILURE to trigger recovery
        if self._front_range < 0.35:
            self.ros_node.get_logger().warn(
                f'[BT] NavigateToWaypoint: obstacle at {self._front_range:.2f}m!')
            self._stop()
            return NodeStatus.FAILURE

        # ── Proportional Controller (adapted from Assignment 2) ──
        dx = target['x'] - self._current_x
        dy = target['y'] - self._current_y
        distance = math.sqrt(dx * dx + dy * dy)

        # Arrival check
        if distance < 0.15:
            self._stop()
            self.ros_node.get_logger().info('[BT] NavigateToWaypoint: arrived!')
            return NodeStatus.SUCCESS

        desired_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(desired_yaw - self._current_yaw)

        twist = Twist()
        Kp_linear = 1.5
        Kp_angular = 4.0

        if abs(yaw_error) > 0.3:
            # Large heading error — rotate in place
            twist.angular.z = self._clamp(Kp_angular * yaw_error, 0.8)
            twist.linear.x = 0.0
        else:
            twist.linear.x = min(Kp_linear * distance, 0.2)
            twist.angular.z = self._clamp(Kp_angular * yaw_error, 0.8)

        self._cmd_vel_pub.publish(twist)
        return NodeStatus.RUNNING

    def on_halt(self):
        self._stop()
        self._start_time = None

    def _stop(self):
        self._cmd_vel_pub.publish(Twist())

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(value, limit))


class NavigateToDockVicinity(NavigateToWaypoint):
    """
    Navigate back to the dock position (from blackboard).
    Same controller as NavigateToWaypoint but reads dock_pose instead.
    """

    def on_start(self):
        dock_pose = self.blackboard.get('dock_pose')
        if dock_pose is None:
            self.ros_node.get_logger().error(
                '[BT] NavigateToDockVicinity: no dock_pose!')
            return NodeStatus.FAILURE

        # Set dock_pose as target_pose for the parent's navigation logic
        self.blackboard.set('target_pose', dock_pose)
        self._start_time = time.time()
        self.ros_node.get_logger().info(
            f'[BT] NavigateToDockVicinity: returning to dock at '
            f'({dock_pose["x"]:.2f}, {dock_pose["y"]:.2f})')
        return NodeStatus.RUNNING
