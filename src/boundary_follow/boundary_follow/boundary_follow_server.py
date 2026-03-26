"""
Boundary Follow Action Server — Assignment 2 Reuse
====================================================
Wall-following behaviour adapted from Assignment 2's turtlesim
boundary-following logic, now compatible with Create3.

Key adaptations from Assignment 2:
  - Uses /scan (LaserScan) instead of turtlesim pose for obstacle detection
  - Publishes to /cmd_vel_nav (for twist_mux) instead of /turtle1/cmd_vel
  - Wrapped as a ROS2 Action Server instead of a standalone FSM node
  - Runs for a configurable duration then returns

The wall-following algorithm:
  1. Find the nearest obstacle using front/left/right scan sectors
  2. If obstacle ahead: turn away
  3. If wall on right: follow it (keep distance ~0.35m)
  4. If no wall nearby: turn right to find one
  This is a classic right-hand wall-following rule.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import math
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from custom_interfaces.action import BoundaryFollow


class BoundaryFollowServer(Node):

    def __init__(self):
        super().__init__('boundary_follow_server')

        self.cb_group = ReentrantCallbackGroup()

        # Parameters (tuned for Create3 in Gazebo)
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('wall_distance', 0.35)       # desired wall distance
        self.declare_parameter('front_threshold', 0.40)      # obstacle ahead threshold
        self.declare_parameter('side_threshold', 0.60)       # wall detection on side

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.wall_distance = self.get_parameter('wall_distance').value
        self.front_threshold = self.get_parameter('front_threshold').value
        self.side_threshold = self.get_parameter('side_threshold').value

        # Scan data
        self.scan_ranges = None

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10,
            callback_group=self.cb_group)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        # Action server
        self._action_server = ActionServer(
            self,
            BoundaryFollow,
            '/boundary_follow',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('BoundaryFollow action server ready on /boundary_follow')

    # ──────────────────────────────────────────────
    # Scan callback
    # ──────────────────────────────────────────────

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges

    # ──────────────────────────────────────────────
    # Scan sector helpers — adapted from Assignment 2
    # ──────────────────────────────────────────────

    def _get_sector_min(self, start_deg, end_deg):
        """Get minimum range in a sector (degrees, 0=front, CCW positive)."""
        if self.scan_ranges is None:
            return float('inf')

        n = len(self.scan_ranges)
        if n == 0:
            return float('inf')

        min_range = float('inf')
        for deg in range(int(start_deg), int(end_deg)):
            idx = deg % n
            r = self.scan_ranges[idx]
            if not math.isinf(r) and not math.isnan(r) and r > 0.05:
                min_range = min(min_range, r)
        return min_range

    def _get_front_range(self):
        """Range in front of robot (~±15°)."""
        return min(
            self._get_sector_min(345, 360),
            self._get_sector_min(0, 15)
        )

    def _get_right_range(self):
        """Range on right side (~260°-280° in Create3 scan frame)."""
        return self._get_sector_min(260, 290)

    def _get_left_range(self):
        """Range on left side (~80°-100°)."""
        return self._get_sector_min(70, 110)

    # ──────────────────────────────────────────────
    # Action callbacks
    # ──────────────────────────────────────────────

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'BoundaryFollow goal received: duration={goal_request.duration:.1f}s')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('BoundaryFollow cancel requested')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Main wall-following loop — runs for the requested duration.
        This is the core Assignment 2 logic adapted for Create3 sensors.
        """
        self.get_logger().info('Executing BoundaryFollow...')
        duration = goal_handle.request.duration
        if duration <= 0:
            duration = 15.0

        feedback_msg = BoundaryFollow.Feedback()
        result = BoundaryFollow.Result()

        start_time = time.time()
        rate = self.create_rate(20)

        total_distance = 0.0
        prev_time = start_time

        while True:
            elapsed = time.time() - start_time

            # Check time budget
            if elapsed >= duration:
                break

            # Check cancel
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._stop()
                result.distance_traveled = total_distance
                result.success = False
                return result

            # Wait for scan data
            if self.scan_ranges is None:
                rate.sleep()
                continue

            # ── Core wall-following logic (from Assignment 2) ──
            front = self._get_front_range()
            right = self._get_right_range()
            left = self._get_left_range()

            twist = Twist()

            if front < self.front_threshold:
                # Obstacle ahead — turn left (away from wall)
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
            elif right < self.wall_distance - 0.05:
                # Too close to wall — veer left slightly
                twist.linear.x = self.linear_speed * 0.5
                twist.angular.z = self.angular_speed * 0.5
            elif right < self.side_threshold:
                # Wall on right at good distance — follow it
                twist.linear.x = self.linear_speed
                # P-controller to maintain wall distance
                error = right - self.wall_distance
                twist.angular.z = -1.5 * error  # negative = turn right toward wall
            else:
                # No wall on right — turn right to find one
                twist.linear.x = self.linear_speed * 0.5
                twist.angular.z = -self.angular_speed * 0.4

            self.cmd_vel_pub.publish(twist)

            # Track approximate distance
            dt = time.time() - prev_time
            prev_time = time.time()
            total_distance += abs(twist.linear.x) * dt

            # Publish feedback
            feedback_msg.elapsed_time = elapsed
            feedback_msg.min_range = min(front, right, left)
            goal_handle.publish_feedback(feedback_msg)

            rate.sleep()

        # Done
        self._stop()
        goal_handle.succeed()
        result.distance_traveled = total_distance
        result.success = True
        self.get_logger().info(
            f'BoundaryFollow completed. Distance: {total_distance:.2f}m')
        return result

    def _stop(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryFollowServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
