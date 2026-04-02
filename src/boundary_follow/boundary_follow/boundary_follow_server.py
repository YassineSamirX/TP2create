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
WALL_LOST_THRESHOLD = 70
IR_MIN = 15
WALL_MEMORY_SECONDS = 1.0
SEARCH_LINEAR_FACTOR = 0.9
SEARCH_ANGULAR_FACTOR = 0.18
TOO_CLOSE_LINEAR_FACTOR = 0.75
FOLLOW_TURN_GAIN = 0.0008          # Reduced from 0.0012
COMMAND_SMOOTHING = 0.15           # Reduced from 0.5 (smoother transitions)
IR_FILTER_ALPHA = 0.3              # Low-pass filter for IR sensors
HYSTERESIS = 20                    # Prevents rapid state switching
DERIVATIVE_GAIN = 0.002            # Dampens oscillations


class BoundaryFollowServer(Node):

    def __init__(self):
        super().__init__('boundary_follow_server')

        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

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
        self.last_wall_seen_time = time.time()
        self.last_linear_command = 0.0
        self.last_angular_command = 0.0
        self.last_right_ir = IR_MIN
        self.current_state = 'SEARCH'

        self.ir_sub = self.create_subscription(
            IrIntensityVector,
            '/Robot5/ir_intensity',
            self.ir_callback,
            qos_profile,
            callback_group=self.cb_group)

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
            'BoundaryFollow action server ready (anti-jitter enabled)')

    def ir_callback(self, msg):
        for reading in msg.readings:
            frame = reading.header.frame_id
            key = frame.replace('ir_intensity_', '')
            if key in self.ir_values:
                # Low-pass filter to reduce sensor noise
                self.ir_values[key] = (
                    IR_FILTER_ALPHA * reading.value +
                    (1 - IR_FILTER_ALPHA) * self.ir_values[key]
                )

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

    def _get_state_with_hysteresis(self, front_ir, right_ir):
        """Determine state with hysteresis to prevent rapid switching."""
        if self.bump_detected:
            return 'BUMP'
        
        if front_ir > FRONT_OBSTACLE_THRESHOLD + HYSTERESIS:
            return 'FRONT_OBSTACLE'
        elif self.current_state == 'FRONT_OBSTACLE' and front_ir > FRONT_OBSTACLE_THRESHOLD - HYSTERESIS:
            return 'FRONT_OBSTACLE'
        
        if right_ir > WALL_TOO_CLOSE_THRESHOLD + HYSTERESIS:
            return 'TOO_CLOSE'
        elif self.current_state == 'TOO_CLOSE' and right_ir > WALL_TOO_CLOSE_THRESHOLD - HYSTERESIS:
            return 'TOO_CLOSE'
        
        if right_ir > WALL_FOLLOW_THRESHOLD + HYSTERESIS:
            return 'FOLLOWING'
        elif self.current_state == 'FOLLOWING' and right_ir > WALL_FOLLOW_THRESHOLD - HYSTERESIS:
            return 'FOLLOWING'
        
        return 'SEARCH'

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

            twist = self._make_smoothed_twist(
                self.linear_speed * SEARCH_LINEAR_FACTOR,
                -self.angular_speed * SEARCH_ANGULAR_FACTOR)
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

            # Compute derivative for damping
            ir_derivative = right_ir - self.last_right_ir
            self.last_right_ir = right_ir

            if right_ir > WALL_LOST_THRESHOLD:
                self.last_wall_seen_time = time.time()

            target_linear = 0.0
            target_angular = 0.0

            # Use hysteresis-based state machine
            self.current_state = self._get_state_with_hysteresis(front_ir, right_ir)

            if self.current_state == 'BUMP':
                self._stop()
                time.sleep(0.1)

                twist = Twist()
                twist.linear.x = -self.linear_speed
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.5)

                twist = Twist()
                twist.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.8)

                self.bump_detected = False

            elif self.current_state == 'FRONT_OBSTACLE':
                target_linear = 0.0
                target_angular = self.angular_speed

            elif self.current_state == 'TOO_CLOSE':
                target_linear = self.linear_speed * TOO_CLOSE_LINEAR_FACTOR
                target_angular = self.angular_speed * 0.35

            elif self.current_state == 'FOLLOWING':
                error = right_ir - WALL_FOLLOW_THRESHOLD
                # PD control: proportional + derivative
                correction = error * FOLLOW_TURN_GAIN - ir_derivative * DERIVATIVE_GAIN

                target_linear = self.linear_speed * 1.1
                target_angular = max(-self.angular_speed * 0.3, 
                                    min(correction, self.angular_speed * 0.18))

            else:  # SEARCH
                wall_recently_seen = (
                    time.time() - self.last_wall_seen_time < WALL_MEMORY_SECONDS
                )
                if wall_recently_seen:
                    target_linear = self.linear_speed * 1.05
                    target_angular = -self.angular_speed * 0.12
                else:
                    target_linear = self.linear_speed * SEARCH_LINEAR_FACTOR
                    target_angular = -self.angular_speed * SEARCH_ANGULAR_FACTOR

            twist = self._make_smoothed_twist(target_linear, target_angular)

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
        self.last_linear_command = 0.0
        self.last_angular_command = 0.0
        self.cmd_vel_pub.publish(Twist())

    def _make_smoothed_twist(self, target_linear, target_angular):
        twist = Twist()
        self.last_linear_command += (
            target_linear - self.last_linear_command
        ) * COMMAND_SMOOTHING
        self.last_angular_command += (
            target_angular - self.last_angular_command
        ) * COMMAND_SMOOTHING
        twist.linear.x = self.last_linear_command
        twist.angular.z = self.last_angular_command
        return twist


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