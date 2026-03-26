"""
SetMissionComplete — Sync Action Node
=======================================
Sets mission_complete = true on the blackboard.
Called as the last step of the mission sequence.
"""

from create3_bt.bt_framework import LeafNode, NodeStatus


class SetMissionComplete(LeafNode):

    def _tick_impl(self):
        self.blackboard.set('mission_complete', True)
        self.ros_node.get_logger().info(
            '[BT] SetMissionComplete: ✅ Mission complete!')
        return NodeStatus.SUCCESS
