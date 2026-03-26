"""
TeleopPassthrough — Stateful Action Node
==========================================
When manual override is active, this node:
  on_start: publishes zero velocity (stops the robot)
  on_running: returns RUNNING (blocks autonomous mission)
  on_halt: does nothing (autonomous resumes when override ends)

The robot is controlled via a separate teleop node publishing
to /cmd_vel_teleop, which twist_mux forwards to /cmd_vel.
"""

from geometry_msgs.msg import Twist
from create3_bt.bt_framework import ActionNode, NodeStatus


class TeleopPassthrough(ActionNode):

    def on_init(self):
        self._cmd_vel_pub = self.ros_node.create_publisher(
            Twist, '/cmd_vel_nav', 10)
        self.ros_node.get_logger().info('[BT] TeleopPassthrough: ready')

    def on_start(self):
        # Stop the robot immediately
        self._cmd_vel_pub.publish(Twist())
        self.ros_node.get_logger().info(
            '[BT] TeleopPassthrough: robot stopped, teleop active')
        return NodeStatus.RUNNING

    def on_running(self):
        # Stay in RUNNING — block the autonomous mission
        # The manual override check will halt us when override ends
        return NodeStatus.RUNNING

    def on_halt(self):
        self.ros_node.get_logger().info(
            '[BT] TeleopPassthrough: halted, autonomous resuming')
