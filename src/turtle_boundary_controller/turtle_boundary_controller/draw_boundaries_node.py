import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import math
import threading
from enum import Enum

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.srv import SetPen


class State(Enum):
    MOVE_TO_BOUNDARY = 1
    FOLLOW_BOUNDARY = 2
    RETURN_HOME = 3
    IDLE = 4
    MANUAL = 5


# Turtlesim window is ~11.09 x 11.09
# We define the boundary rectangle with a margin from the edges
BOUNDARY_MARGIN = 0.5
WALL_MIN = BOUNDARY_MARGIN
WALL_MAX = 11.09 - BOUNDARY_MARGIN

# Corner positions of the boundary rectangle (CCW order)
# CCW: bottom-left -> bottom-right -> top-right -> top-left
CORNERS_CCW = [
    (WALL_MIN, WALL_MIN),   # 0: bottom-left
    (WALL_MAX, WALL_MIN),   # 1: bottom-right
    (WALL_MAX, WALL_MAX),   # 2: top-right
    (WALL_MIN, WALL_MAX),   # 3: top-left
]


class DrawBoundariesNode(Node):

    def __init__(self):
        super().__init__('draw_boundaries_node')

        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('boundary_margin', BOUNDARY_MARGIN)

        self.speed = self.get_parameter('speed').value
        self.margin = self.get_parameter('boundary_margin').value

        # State machine
        self.state = State.MOVE_TO_BOUNDARY
        self.previous_state = None

        self.current_pose = None
        self.initial_pose = None

        # Boundary following state
        self.current_corner_index = None
        self.boundary_start_point = None
        self.corners_visited = 0

        # Lock to prevent concurrent control loop re-entry
        self._lock = threading.Lock()

        # Subscribers
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10,
            callback_group=self.cb_group)

        self.keyboard_sub = self.create_subscription(
            String, '/keyboard_input', self.keyboard_callback, 10,
            callback_group=self.cb_group)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Service client for pen control
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        # Main control loop at 50 Hz
        self.control_timer = self.create_timer(
            0.02, self.control_loop, callback_group=self.cb_group)

        self.get_logger().info('Draw boundaries node started.')

    # ──────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.initial_pose is None:
            self.initial_pose = msg
            self.get_logger().info(
                f'Initial pose: ({msg.x:.2f}, {msg.y:.2f}, theta={msg.theta:.2f})')

    def keyboard_callback(self, msg):
        if msg.data == 'toggle_manual':
            if self.state == State.MANUAL:
                # Resume from manual mode — re-evaluate state from current position
                self.state = self._determine_resume_state()
                self.get_logger().info(f'Resuming -> {self.state.name}')
            elif self.state != State.IDLE:
                # Enter manual mode — stop turtle, let teleop take over
                self.previous_state = self.state
                self.state = State.MANUAL
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info('MANUAL mode. Use teleop to control. Press SPACE to resume.')

    # ──────────────────────────────────────────────
    # Main control loop
    # ──────────────────────────────────────────────

    def control_loop(self):
        if self.current_pose is None:
            return

        # Prevent re-entrant execution from multiple threads
        if not self._lock.acquire(blocking=False):
            return
        try:
            if self.state == State.MOVE_TO_BOUNDARY:
                self._do_move_to_boundary()
            elif self.state == State.FOLLOW_BOUNDARY:
                self._do_follow_boundary()
            elif self.state == State.RETURN_HOME:
                self._do_return_home()
            elif self.state == State.MANUAL:
                pass  # teleop_key node handles movement
            elif self.state == State.IDLE:
                pass
        finally:
            self._lock.release()

    # ──────────────────────────────────────────────
    # State: MOVE_TO_BOUNDARY
    # ──────────────────────────────────────────────

    def _do_move_to_boundary(self):
        self._set_pen(off=True)

        pose = self.current_pose

        # Find the nearest wall
        distances = {
            'left':   pose.x - WALL_MIN,
            'right':  WALL_MAX - pose.x,
            'bottom': pose.y - WALL_MIN,
            'top':    WALL_MAX - pose.y,
        }
        nearest_wall = min(distances, key=distances.get)
        dist_to_wall = distances[nearest_wall]

        # Target heading for each wall
        target_angles = {
            'left':   math.pi,
            'right':  0.0,
            'bottom': -math.pi / 2,
            'top':    math.pi / 2,
        }
        target_angle = target_angles[nearest_wall]

        # Rotate toward the wall first
        angle_error = self._normalize_angle(target_angle - pose.theta)

        twist = Twist()
        if abs(angle_error) > 0.02:
            Kp_rot = 4.0
            twist.angular.z = self._clamp(Kp_rot * angle_error, self.speed)
            self.cmd_vel_pub.publish(twist)
            return

        # Drive straight toward wall
        if dist_to_wall > 0.05:
            Kp = 2.0
            twist.linear.x = min(Kp * dist_to_wall, self.speed)
            self.cmd_vel_pub.publish(twist)
            return

        # Reached boundary wall — stop and transition
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info(f'Reached boundary wall: {nearest_wall}')

        self._init_boundary_following(nearest_wall)
        self.boundary_start_point = (pose.x, pose.y)
        self.corners_visited = 0
        self._set_pen(off=False)
        self.state = State.FOLLOW_BOUNDARY
        self.get_logger().info(
            f'-> FOLLOW_BOUNDARY, first target corner: {self.current_corner_index}')

    def _init_boundary_following(self, nearest_wall):
        """Pick the first target corner for CCW boundary following."""
        # CCW: 0=bottom-left, 1=bottom-right, 2=top-right, 3=top-left
        if nearest_wall == 'bottom':
            self.current_corner_index = 1  # go right along bottom
        elif nearest_wall == 'right':
            self.current_corner_index = 2  # go up along right
        elif nearest_wall == 'top':
            self.current_corner_index = 3  # go left along top
        elif nearest_wall == 'left':
            self.current_corner_index = 0  # go down along left

    # ──────────────────────────────────────────────
    # State: FOLLOW_BOUNDARY
    # ──────────────────────────────────────────────

    def _do_follow_boundary(self):
        # Check if boundary loop is closed
        if self.corners_visited >= 4 and self.boundary_start_point is not None:
            dx = self.current_pose.x - self.boundary_start_point[0]
            dy = self.current_pose.y - self.boundary_start_point[1]
            if math.sqrt(dx * dx + dy * dy) < 0.5:
                self.cmd_vel_pub.publish(Twist())
                self._set_pen(off=True)
                self.state = State.RETURN_HOME
                self.get_logger().info('Boundary closed! -> RETURN_HOME')
                return

        target = CORNERS_CCW[self.current_corner_index]
        self._navigate_to_point(
            target[0], target[1],
            arrival_threshold=0.25,
            on_arrival=self._on_corner_reached)

    def _on_corner_reached(self):
        self.get_logger().info(
            f'Reached corner {self.current_corner_index}: '
            f'{CORNERS_CCW[self.current_corner_index]}')
        self.corners_visited += 1
        self.current_corner_index = (self.current_corner_index + 1) % 4

    # ──────────────────────────────────────────────
    # State: RETURN_HOME
    # ──────────────────────────────────────────────

    def _do_return_home(self):
        if self.initial_pose is None:
            return
        self._navigate_to_point(
            self.initial_pose.x, self.initial_pose.y,
            arrival_threshold=0.15,
            on_arrival=self._on_home_reached)

    def _on_home_reached(self):
        self.cmd_vel_pub.publish(Twist())
        self.state = State.IDLE
        self.get_logger().info('Returned home. IDLE.')

    # ──────────────────────────────────────────────
    # Resume from MANUAL
    # ──────────────────────────────────────────────

    def _determine_resume_state(self):
        pose = self.current_pose

        # If we already completed the boundary, keep returning home
        if self.corners_visited >= 4:
            self._set_pen(off=True)
            return State.RETURN_HOME

        # If near a wall and we've started boundary following, resume it
        near_wall = (pose.x <= WALL_MIN + 0.3 or pose.x >= WALL_MAX - 0.3 or
                     pose.y <= WALL_MIN + 0.3 or pose.y >= WALL_MAX - 0.3)

        if near_wall and self.boundary_start_point is not None:
            self._retarget_nearest_corner()
            self._set_pen(off=False)
            return State.FOLLOW_BOUNDARY

        # Otherwise start from scratch — move to boundary
        self._set_pen(off=True)
        return State.MOVE_TO_BOUNDARY

    def _retarget_nearest_corner(self):
        pose = self.current_pose
        best_idx = self.current_corner_index
        best_dist = float('inf')

        for i in range(4):
            cx, cy = CORNERS_CCW[i]
            d = math.sqrt((pose.x - cx) ** 2 + (pose.y - cy) ** 2)
            if d < best_dist:
                best_dist = d
                best_idx = i

        # If very close to the nearest corner, target the next one
        if best_dist < 0.4:
            best_idx = (best_idx + 1) % 4

        self.current_corner_index = best_idx

    # ──────────────────────────────────────────────
    # Navigation helper
    # ──────────────────────────────────────────────

    def _navigate_to_point(self, target_x, target_y, arrival_threshold=0.15,
                           on_arrival=None):
        pose = self.current_pose
        dx = target_x - pose.x
        dy = target_y - pose.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < arrival_threshold:
            self.cmd_vel_pub.publish(Twist())
            if on_arrival:
                on_arrival()
            return

        desired_theta = math.atan2(dy, dx)
        angle_error = self._normalize_angle(desired_theta - pose.theta)

        twist = Twist()
        Kp_linear = 2.0
        Kp_angular = 6.0

        if abs(angle_error) > 0.3:
            # Large heading error — rotate in place first
            twist.angular.z = self._clamp(Kp_angular * angle_error, self.speed)
            twist.linear.x = 0.0
        else:
            twist.linear.x = min(Kp_linear * distance, self.speed)
            twist.angular.z = self._clamp(Kp_angular * angle_error, self.speed)

        self.cmd_vel_pub.publish(twist)

    # ──────────────────────────────────────────────
    # Utilities
    # ──────────────────────────────────────────────

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

    def _set_pen(self, r=255, g=255, b=255, width=3, off=False):
        if not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('set_pen service not available')
            return
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = int(off)
        self.pen_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = DrawBoundariesNode()
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
