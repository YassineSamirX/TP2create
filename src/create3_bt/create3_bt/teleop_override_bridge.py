"""
Teleop Override Bridge
========================
Standalone node that listens for keyboard input (spacebar)
and publishes Bool to /manual_override to toggle between
autonomous and manual control modes.

Reuses the keyboard detection pattern from Assignment 2's
keyboard_listener.py, but publishes a Bool toggle instead
of a String message.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import sys
import select


class TeleopOverrideBridge(Node):

    def __init__(self):
        super().__init__('teleop_override_bridge')
        self.publisher_ = self.create_publisher(Bool, '/manual_override', 10)
        self._manual_mode = False

        # Try raw mode (Linux) or fallback to polling
        try:
            import termios
            import tty
            self._old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            self._raw_mode = True
        except (ImportError, termios.error):
            self._raw_mode = False

        # Poll keyboard at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(
            'Override bridge ready. Press SPACE to toggle manual override.')

    def timer_callback(self):
        if not self._raw_mode:
            return

        if select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1)
            if key == ' ':
                self._manual_mode = not self._manual_mode
                msg = Bool()
                msg.data = self._manual_mode
                self.publisher_.publish(msg)
                status = 'MANUAL' if self._manual_mode else 'AUTONOMOUS'
                self.get_logger().info(f'SPACE pressed — mode: {status}')
            elif key == '\x03':  # Ctrl+C
                raise KeyboardInterrupt

    def destroy_node(self):
        if self._raw_mode:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopOverrideBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
