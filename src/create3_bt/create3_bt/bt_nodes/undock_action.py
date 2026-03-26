"""
UndockAction — Action Node (Action Client)
============================================
Calls the Create3 /undock action.
For simulation without irobot_create_msgs, uses a simple
service call to /undock_sim (std_srvs/Trigger) or just
drives backward briefly.
"""

import time
from geometry_msgs.msg import Twist
from create3_bt.bt_framework import ActionNode, NodeStatus


class UndockAction(ActionNode):
    """Undock from charging station by driving backward."""

    def on_init(self):
        self._cmd_vel_pub = self.ros_node.create_publisher(
            Twist, '/cmd_vel_nav', 10)
        self._start_time = None
        self._undock_duration = 3.0  # seconds to drive backward
        self.ros_node.get_logger().info('[BT] UndockAction: ready')

    def on_start(self):
        self.ros_node.get_logger().info('[BT] UndockAction: starting undock sequence')
        self._start_time = time.time()
        return NodeStatus.RUNNING

    def on_running(self):
        elapsed = time.time() - self._start_time

        if elapsed < self._undock_duration:
            # Drive backward slowly
            twist = Twist()
            twist.linear.x = -0.1
            self._cmd_vel_pub.publish(twist)
            return NodeStatus.RUNNING
        else:
            # Stop and mark as undocked
            self._cmd_vel_pub.publish(Twist())
            self.ros_node.get_logger().info('[BT] UndockAction: undock complete')
            return NodeStatus.SUCCESS

    def on_halt(self):
        self._cmd_vel_pub.publish(Twist())
        self._start_time = None
