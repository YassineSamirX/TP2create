"""
SelectNextWaypoint / SetRecoveryWaypoint — Sync Action Nodes
==============================================================
SelectNextWaypoint: picks the next waypoint from the list (cycling).
SetRecoveryWaypoint: after a recovery, skips to the next waypoint.

Waypoints are stored on the blackboard as a list of (x, y) tuples
in the odom frame.
"""

from create3_bt.bt_framework import LeafNode, NodeStatus


class SelectNextWaypoint(LeafNode):
    """Cycle through waypoints and write target_pose to blackboard."""

    def _tick_impl(self):
        waypoints = self.blackboard.get('waypoints', [])
        if not waypoints:
            self.ros_node.get_logger().error('[BT] SelectNextWaypoint: no waypoints!')
            return NodeStatus.FAILURE

        idx = self.blackboard.get('waypoint_index', 0)
        wp = waypoints[idx % len(waypoints)]

        target_pose = {'x': wp[0], 'y': wp[1], 'frame': 'odom'}
        self.blackboard.set('target_pose', target_pose)
        self.blackboard.set('waypoint_index', (idx + 1) % len(waypoints))

        self.ros_node.get_logger().info(
            f'[BT] SelectNextWaypoint: target=({wp[0]:.2f}, {wp[1]:.2f}), '
            f'index={idx}')
        return NodeStatus.SUCCESS


class SetRecoveryWaypoint(LeafNode):
    """After recovery, advance to next waypoint (skip the blocked one)."""

    def _tick_impl(self):
        waypoints = self.blackboard.get('waypoints', [])
        if not waypoints:
            return NodeStatus.FAILURE

        idx = self.blackboard.get('waypoint_index', 0)
        # Skip ahead by one (the current one was blocked)
        new_idx = (idx + 1) % len(waypoints)
        self.blackboard.set('waypoint_index', new_idx)

        wp = waypoints[new_idx]
        target_pose = {'x': wp[0], 'y': wp[1], 'frame': 'odom'}
        self.blackboard.set('target_pose', target_pose)

        self.ros_node.get_logger().info(
            f'[BT] SetRecoveryWaypoint: skipping to waypoint {new_idx} '
            f'({wp[0]:.2f}, {wp[1]:.2f})')
        return NodeStatus.SUCCESS
