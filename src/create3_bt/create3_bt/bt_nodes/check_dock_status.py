"""
CheckIsDocked / CheckAlreadyUndocked — Condition Nodes
========================================================
Subscribe to Create3's /dock_status topic to check docking state.
Create3 publishes irobot_create_msgs/msg/DockStatus on this topic.

For simulation without irobot_create_msgs, we fall back to a
simple Bool on /is_docked.
"""

from std_msgs.msg import Bool
from create3_bt.bt_framework import ConditionNode, NodeStatus


class CheckIsDocked(ConditionNode):
    """Returns SUCCESS if the robot IS docked."""

    def on_init(self):
        self._is_docked = True  # assume docked at start
        self._sub = self.ros_node.create_subscription(
            Bool, '/is_docked', self._dock_cb, 10)
        self.ros_node.get_logger().info('[BT] CheckIsDocked: subscribed to /is_docked')

    def _dock_cb(self, msg):
        self._is_docked = msg.data

    def _tick_impl(self):
        if self._is_docked:
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class CheckAlreadyUndocked(ConditionNode):
    """Returns SUCCESS if the robot is NOT docked (already undocked)."""

    def on_init(self):
        self._is_docked = True  # assume docked at start
        self._sub = self.ros_node.create_subscription(
            Bool, '/is_docked', self._dock_cb, 10)
        self.ros_node.get_logger().info('[BT] CheckAlreadyUndocked: subscribed to /is_docked')

    def _dock_cb(self, msg):
        self._is_docked = msg.data

    def _tick_impl(self):
        if not self._is_docked:
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE
