"""
CheckNotManualOverride — Condition Node
========================================
Subscribes to /manual_override (std_msgs/Bool).
Returns SUCCESS when autonomous mode (override == false).
Returns FAILURE when manual override is active (override == true).

Used as the first child of the root ReactiveSequence so that
manual override is checked on EVERY tick.
"""

from std_msgs.msg import Bool
from create3_bt.bt_framework import ConditionNode, NodeStatus


class CheckNotManualOverride(ConditionNode):

    def on_init(self):
        self._manual_override = False
        self._sub = self.ros_node.create_subscription(
            Bool, '/manual_override', self._override_cb, 10)
        self.ros_node.get_logger().info(
            '[BT] CheckNotManualOverride: subscribed to /manual_override')

    def _override_cb(self, msg):
        self._manual_override = msg.data
        if msg.data:
            self.ros_node.get_logger().info('[BT] Manual override ACTIVATED')
        else:
            self.ros_node.get_logger().info('[BT] Manual override RELEASED')

    def _tick_impl(self):
        if not self._manual_override:
            return NodeStatus.SUCCESS   # autonomous mode
        return NodeStatus.FAILURE       # manual override active
