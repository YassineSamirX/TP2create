"""
DockAction — Action Node
=========================
Return to dock by driving forward slowly until contact.
In a real Create3, would call /dock action (irobot_create_msgs).
For simulation, drives forward slowly for a fixed duration.
"""

import time
from geometry_msgs.msg import Twist
from create3_bt.bt_framework import ActionNode, NodeStatus


class DockAction(ActionNode):
    """Dock with charging station."""

    def on_init(self):
        self._cmd_vel_pub = self.ros_node.create_publisher(
            Twist, '/cmd_vel_nav', 10)
        self._start_time = None
        self._dock_duration = 4.0  # seconds to drive forward toward dock
        self.ros_node.get_logger().info('[BT] DockAction: ready')

    def on_start(self):
        self.ros_node.get_logger().info('[BT] DockAction: starting dock sequence')
        self._start_time = time.time()
        return NodeStatus.RUNNING

    def on_running(self):
        elapsed = time.time() - self._start_time

        if elapsed < self._dock_duration:
            # Drive forward slowly toward dock
            twist = Twist()
            twist.linear.x = 0.08
            self._cmd_vel_pub.publish(twist)
            return NodeStatus.RUNNING
        else:
            # Stop — assume docked
            self._cmd_vel_pub.publish(Twist())
            self.ros_node.get_logger().info('[BT] DockAction: docking complete')
            return NodeStatus.SUCCESS

    def on_halt(self):
        self._cmd_vel_pub.publish(Twist())
        self._start_time = None
