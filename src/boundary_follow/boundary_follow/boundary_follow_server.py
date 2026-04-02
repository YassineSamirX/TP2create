"""
Boundary Follow Action Server — Create3 Native IR Sensors
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import time

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector
from custom_interfaces.action import BoundaryFollow

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


FRONT_OBSTACLE_THRESHOLD = 100
WALL_TOO_CLOSE_THRESHOLD = 300
WALL_FOLLOW_THRESHOLD = 100
IR_MIN = 15


class BoundaryFollowServer(Node):

    def __init__(self):
        super().__init__('boundary_follow_server')

        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        #  AJOUT QoS BEST_EFFORT
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.ir_values = {
            'front_center_left': IR_MIN,
            'front_center_right': IR_MIN,
            'front_left': IR_MIN,
            'front_right': IR_MIN,
            'left': IR_MIN,
            'side_left': IR_MIN,
            'right': IR_MIN,
        }

        self.bump_detected = False

        # ✅ MODIFIÉ (10 → qos_profile)
        self.ir_sub = self.create_subscription(
            IrIntensityVector,
            '/Robot5/ir_intensity',
            self.ir_callback,
            qos_profile,
            callback_group=self.cb_group)

        # ✅ MODIFIÉ (10 → qos_profile)
        self.hazard_sub = self.create_subscription(
            HazardDetectionVector,
            '/Robot5/hazard_detection',
            self.hazard_callback,
            qos_profile,
            callback_group=self.cb_group)

        self.cmd_vel_pub = self.create_publisher(Twist, '/Robot5/cmd_vel', 10)

        self._action_server = ActionServer(
            self,
            BoundaryFollow,
            '/Robot5/boundary_follow',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            'BoundaryFollow action server ready (BEST_EFFORT)')

    def ir_callback(self, msg):
        for reading in msg.readings:
            frame = reading.header.frame_id
            key = frame.replace('ir_intensity_', '')
            if key in self.ir_values:
                self.ir_values[key] = reading.value

    def hazard_callback(self, msg):
        self.bump_detected = False
        for detection in msg.detections:
            if detection.type == detection.BUMP:
                self.bump_detected = True
                self.get_logger().warn('BUMP détecté !')

    def get_front_ir(self):
        return max(
            self.ir_values['front_center_left'],
            self.ir_values['front_center_right'],
            self.ir_values['front_left'],
            self.ir_values['front_right'],
        )

    def get_right_ir(self):
        return self.ir_values['right']

    def get_left_ir(self):
        return max(
            self.ir_values['left'],
            self.ir_values['side_left'],
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'BoundaryFollow goal reçu: duration={goal_request.duration:.1f}s')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('BoundaryFollow annulé')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):

        self.get_logger().info('Démarrage BoundaryFollow...')

        duration = goal_handle.request.duration
        if duration <= 0:
            duration = 15.0

        feedback_msg = BoundaryFollow.Feedback()
        result = BoundaryFollow.Result()

        start_time = time.time()
        rate = self.create_rate(20)

        total_distance = 0.0
        prev_time = start_time

        self.get_logger().info('Phase 1: recherche d\'un mur...')
        search_start = time.time()

        while time.time() - search_start < 5.0:
            if self.get_right_ir() > WALL_FOLLOW_THRESHOLD:
                break
            if self.get_front_ir() > FRONT_OBSTACLE_THRESHOLD:
                break

            twist = Twist()
            twist.linear.x = self.linear_speed * 0.3
            twist.angular.z = -self.angular_speed * 0.5
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.get_logger().info('Phase 2: wall-following démarré')

        while True:

            elapsed = time.time() - start_time
            if elapsed >= duration:
                break

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._stop()
                result.distance_traveled = total_distance
                result.success = False
                return result

            front_ir = self.get_front_ir()
            right_ir = self.get_right_ir()

            twist = Twist()

            if self.bump_detected:
                self._stop()
                time.sleep(0.1)

                twist.linear.x = -self.linear_speed
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.5)

                twist = Twist()
                twist.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.8)

                self.bump_detected = False

            elif front_ir > FRONT_OBSTACLE_THRESHOLD:
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed

            elif right_ir > WALL_TOO_CLOSE_THRESHOLD:
                twist.linear.x = self.linear_speed * 0.5
                twist.angular.z = self.angular_speed * 0.4

            elif right_ir > WALL_FOLLOW_THRESHOLD:
                error = right_ir - WALL_FOLLOW_THRESHOLD
                correction = error * 0.002

                twist.linear.x = self.linear_speed
                twist.angular.z = min(correction, self.angular_speed * 0.3)

            else:
                twist.linear.x = self.linear_speed * 0.5
                twist.angular.z = -self.angular_speed * 0.4

            self.cmd_vel_pub.publish(twist)

            dt = time.time() - prev_time
            prev_time = time.time()
            total_distance += abs(twist.linear.x) * dt

            feedback_msg.elapsed_time = elapsed
            feedback_msg.min_range = float(right_ir)
            goal_handle.publish_feedback(feedback_msg)

            rate.sleep()

        self._stop()
        goal_handle.succeed()

        result.distance_traveled = total_distance
        result.success = True

        self.get_logger().info(
            f'BoundaryFollow terminé. Distance: {total_distance:.2f}m')

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