"""
Draw Square Action Server — Assignment 1 Reuse
================================================
Velocity-based square-drawing action server for Create3.
Drives each side using odometry feedback, turns 90° between sides.
Works in base_link frame — no tf2 needed.

Reused from Assignment 1, adapted for Create3 topic names.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from custom_interfaces.action import DrawSquare


class DrawSquareServer(Node):

    def __init__(self):
        super().__init__('draw_square_server')

        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('linear_speed', 0.15)   # m/s — safe for Create3
        self.declare_parameter('angular_speed', 0.5)   # rad/s

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Odom state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/Robot5/odom', self.odom_callback, 10,
            callback_group=self.cb_group)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/Robot5/cmd_vel', 10)

        # Action server
        self._action_server = ActionServer(
            self,
            DrawSquare,
            '/Robot5/draw_square',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('DrawSquare action server ready on /draw_square')


    # Odometry

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # Quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # Action callbacks

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'DrawSquare goal received: side_length={goal_request.side_length:.2f}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('DrawSquare goal cancel requested')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing DrawSquare...')
        side_length = goal_handle.request.side_length
        if side_length <= 0:
            side_length = 0.5

        feedback_msg = DrawSquare.Feedback()
        result = DrawSquare.Result()
        total_distance = 0.0

        rate = self.create_rate(20)  # 20 Hz control loop

        for side in range(1, 5):
            # ── Drive straight for side_length ──
            feedback_msg.current_side = side
            feedback_msg.percent_complete = (side - 1) / 4.0 * 100.0
            goal_handle.publish_feedback(feedback_msg)

            start_x = self.current_x
            start_y = self.current_y

            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._stop()
                    result.total_distance = total_distance
                    result.success = False
                    return result

                dx = self.current_x - start_x
                dy = self.current_y - start_y
                driven = math.sqrt(dx * dx + dy * dy)

                if driven >= side_length:
                    break

                twist = Twist()
                twist.linear.x = self.linear_speed
                self.cmd_vel_pub.publish(twist)
                rate.sleep()

            self._stop()
            total_distance += side_length
            time.sleep(0.3)  # brief pause at corner

            # Turn 90° CCW
            target_yaw = self._normalize_angle(self.current_yaw + math.pi / 2)

            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._stop()
                    result.total_distance = total_distance
                    result.success = False
                    return result

                yaw_error = self._normalize_angle(target_yaw - self.current_yaw)
                if abs(yaw_error) < 0.05:
                    break

                twist = Twist()
                twist.angular.z = self.angular_speed if yaw_error > 0 else -self.angular_speed
                self.cmd_vel_pub.publish(twist)
                rate.sleep()

            self._stop()
            time.sleep(0.2)

            feedback_msg.percent_complete = side / 4.0 * 100.0
            goal_handle.publish_feedback(feedback_msg)

        # Done
        goal_handle.succeed()
        result.total_distance = total_distance
        result.success = True
        self.get_logger().info(f'DrawSquare completed. Total distance: {total_distance:.2f}m')
        return result

    # Helpers

    def _stop(self):
        self.cmd_vel_pub.publish(Twist())

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = DrawSquareServer()
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
