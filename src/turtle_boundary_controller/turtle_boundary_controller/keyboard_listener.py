import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sys
import termios
import tty
import select


class KeyboardListener(Node):
    """Minimal node: listens for spacebar presses, publishes toggle messages."""

    def __init__(self):
        super().__init__('keyboard_listener')
        self.publisher_ = self.create_publisher(String, '/keyboard_input', 10)

        # Save and set terminal to raw mode for direct key reading
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        # Poll keyboard at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Keyboard listener ready. Press SPACE to toggle manual mode.')

    def timer_callback(self):
        if select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1)
            if key == ' ':
                msg = String()
                msg.data = 'toggle_manual'
                self.publisher_.publish(msg)
                self.get_logger().info('SPACE pressed — toggling manual mode')
            elif key == '\x03':  # Ctrl+C
                raise KeyboardInterrupt

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
