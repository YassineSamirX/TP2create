"""
Patrol Timer — Start + Check Condition
========================================
StartPatrolTimer: records the start time on the blackboard.
CheckPatrolTimeNotExpired: returns SUCCESS if patrol time remaining.
"""

import time
from create3_bt.bt_framework import LeafNode, ConditionNode, NodeStatus


class StartPatrolTimer(LeafNode):
    """Record patrol start time on blackboard."""

    def _tick_impl(self):
        self.blackboard.set('patrol_start_time', time.time())
        duration = self.blackboard.get('patrol_duration', 120.0)
        self.ros_node.get_logger().info(
            f'[BT] StartPatrolTimer: patrol started, duration={duration}s')
        return NodeStatus.SUCCESS


class CheckPatrolTimeNotExpired(ConditionNode):
    """SUCCESS if patrol time has NOT expired, FAILURE otherwise."""

    def _tick_impl(self):
        start = self.blackboard.get('patrol_start_time')
        duration = self.blackboard.get('patrol_duration', 120.0)

        if start is None:
            return NodeStatus.FAILURE

        elapsed = time.time() - start
        if elapsed < duration:
            return NodeStatus.SUCCESS
        else:
            self.ros_node.get_logger().info(
                f'[BT] CheckPatrolTimeNotExpired: patrol time expired ({elapsed:.0f}s)')
            return NodeStatus.FAILURE
