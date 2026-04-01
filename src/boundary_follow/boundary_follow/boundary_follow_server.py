"""
Boundary Follow Action Server — Create3 Native IR Sensors
===========================================================
Wall-following behaviour using the Create3's native IR intensity sensors
and hazard detection. No LiDAR required.

IR Sensors available (frame_id):
  - ir_intensity_front_center_left   → obstacle devant gauche
  - ir_intensity_front_center_right  → obstacle devant droite
  - ir_intensity_front_left          → avant gauche
  - ir_intensity_front_right         → avant droite
  - ir_intensity_left                → gauche
  - ir_intensity_side_left           → côté gauche
  - ir_intensity_right               → droite (suivi du mur)

IR Values:
  - ~15   = pas d'obstacle (valeur minimale)
  - ~200+ = obstacle détecté
  - ~1000+ = obstacle très proche

Algorithme (right-hand wall following):
  1. Si obstacle devant (IR front élevé) → tourner à gauche
  2. Si mur à droite à bonne distance → avancer tout droit
  3. Si mur à droite trop proche → virer légèrement à gauche
  4. Si pas de mur à droite → tourner à droite pour en trouver un
  5. Si BUMP détecté → reculer et tourner à gauche
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


# ── IR Thresholds (calibrés pour le sim Create3) ──
FRONT_OBSTACLE_THRESHOLD = 100   # IR front → obstacle devant, tourner
WALL_TOO_CLOSE_THRESHOLD = 300   # IR right → trop proche du mur
WALL_FOLLOW_THRESHOLD = 100      # IR right → mur détecté à bonne distance
IR_MIN = 15                      # Valeur minimale (pas d'obstacle)


class BoundaryFollowServer(Node):

    def __init__(self):
        super().__init__('boundary_follow_server')

        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # IR sensor values (dictionnaire par frame_id)
        self.ir_values = {
            'front_center_left': IR_MIN,
            'front_center_right': IR_MIN,
            'front_left': IR_MIN,
            'front_right': IR_MIN,
            'left': IR_MIN,
            'side_left': IR_MIN,
            'right': IR_MIN,
        }

        # Hazard state
        self.bump_detected = False

        # Subscribers
        self.ir_sub = self.create_subscription(
            IrIntensityVector,
            '/ir_intensity',
            self.ir_callback,
            10,
            callback_group=self.cb_group)

        self.hazard_sub = self.create_subscription(
            HazardDetectionVector,
            '/hazard_detection',
            self.hazard_callback,
            10,
            callback_group=self.cb_group)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

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

        self.get_logger().info(
            'BoundaryFollow action server ready (IR sensors) on /boundary_follow')

    # ──────────────────────────────────────────────
    # Sensor callbacks
    # ──────────────────────────────────────────────

    def ir_callback(self, msg):
        """Met à jour les valeurs IR depuis le message."""
        for reading in msg.readings:
            # frame_id format: "ir_intensity_front_center_left"
            # On extrait la partie après "ir_intensity_"
            frame = reading.header.frame_id
            key = frame.replace('ir_intensity_', '')
            if key in self.ir_values:
                self.ir_values[key] = reading.value

    def hazard_callback(self, msg):
        """Détecte les bumps et autres hazards."""
        self.bump_detected = False
        for detection in msg.detections:
            if detection.type == detection.BUMP:
                self.bump_detected = True
                self.get_logger().warn('BUMP détecté !')

    # ──────────────────────────────────────────────
    # IR helpers
    # ──────────────────────────────────────────────

    def get_front_ir(self):
        """Valeur maximale des capteurs frontaux."""
        return max(
            self.ir_values['front_center_left'],
            self.ir_values['front_center_right'],
            self.ir_values['front_left'],
            self.ir_values['front_right'],
        )

    def get_right_ir(self):
        """Valeur du capteur droit."""
        return self.ir_values['right']

    def get_left_ir(self):
        """Valeur maximale des capteurs gauches."""
        return max(
            self.ir_values['left'],
            self.ir_values['side_left'],
        )

    # ──────────────────────────────────────────────
    # Action callbacks
    # ──────────────────────────────────────────────

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'BoundaryFollow goal reçu: duration={goal_request.duration:.1f}s')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('BoundaryFollow annulé')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Boucle principale de wall-following avec les capteurs IR natifs.
        """
        self.get_logger().info('Démarrage BoundaryFollow...')
        duration = goal_handle.request.duration
        if duration <= 0:
            duration = 15.0

        feedback_msg = BoundaryFollow.Feedback()
        result = BoundaryFollow.Result()

        start_time = time.time()
        rate = self.create_rate(20)  # 20 Hz

        total_distance = 0.0
        prev_time = start_time

        # Phase 1 : trouver un mur en tournant à droite
        self.get_logger().info('Phase 1: recherche d\'un mur...')
        search_start = time.time()

        while time.time() - search_start < 5.0:
            if self.get_right_ir() > WALL_FOLLOW_THRESHOLD:
                break
            if self.get_front_ir() > FRONT_OBSTACLE_THRESHOLD:
                break
            twist = Twist()
            twist.linear.x = self.linear_speed * 0.3
            twist.angular.z = -self.angular_speed * 0.5  # tourner à droite
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.get_logger().info('Phase 2: wall-following démarré')

        # Phase 2 : wall-following principal
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

            # ── Logique wall-following ──

            if self.bump_detected:
                # Bump → reculer et tourner à gauche
                self.get_logger().warn('Bump! → recul + rotation gauche')
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
                # Obstacle devant → tourner à gauche (CCW)
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
                self.get_logger().debug(
                    f'Obstacle devant (IR={front_ir}) → rotation gauche')

            elif right_ir > WALL_TOO_CLOSE_THRESHOLD:
                # Trop proche du mur à droite → virer légèrement à gauche
                twist.linear.x = self.linear_speed * 0.5
                twist.angular.z = self.angular_speed * 0.4
                self.get_logger().debug(
                    f'Trop proche mur droite (IR={right_ir}) → virer gauche')

            elif right_ir > WALL_FOLLOW_THRESHOLD:
                # Mur à droite à bonne distance → suivre tout droit
                # P-controller pour maintenir la distance
                error = right_ir - WALL_FOLLOW_THRESHOLD
                correction = error * 0.002  # gain proportionnel
                twist.linear.x = self.linear_speed
                twist.angular.z = min(correction, self.angular_speed * 0.3)
                self.get_logger().debug(
                    f'Suivi mur droite (IR={right_ir}) → tout droit')

            else:
                # Pas de mur à droite → tourner à droite pour en trouver un
                twist.linear.x = self.linear_speed * 0.5
                twist.angular.z = -self.angular_speed * 0.4
                self.get_logger().debug(
                    f'Pas de mur droite (IR={right_ir}) → chercher mur')

            self.cmd_vel_pub.publish(twist)

            # Distance approximative
            dt = time.time() - prev_time
            prev_time = time.time()
            total_distance += abs(twist.linear.x) * dt

            # Feedback
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
