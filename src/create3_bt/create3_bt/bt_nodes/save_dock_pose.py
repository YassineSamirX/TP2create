"""
SaveDockPose — Sync Action Node
=================================
Saves the robot's current position (from /odom) as the dock location
to the blackboard. Called immediately after undocking.

Uses tf2 concept: reads odom → base_link transform.
"""

from nav_msgs.msg import Odometry
from create3_bt.bt_framework import LeafNode, NodeStatus


class SaveDockPose(LeafNode):
    """Save current odom pose as dock_pose on blackboard."""

    def on_init(self):
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0
        self._got_odom = False

        self._sub = self.ros_node.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.ros_node.get_logger().info('[BT] SaveDockPose: subscribed to /odom')

    def _odom_cb(self, msg):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        self._got_odom = True

    def _tick_impl(self):
        if not self._got_odom:
            self.ros_node.get_logger().warn('[BT] SaveDockPose: waiting for odom...')
            return NodeStatus.RUNNING

        dock_pose = {
            'x': self._current_x,
            'y': self._current_y,
            'frame': 'odom'
        }
        self.blackboard.set('dock_pose', dock_pose)
        self.ros_node.get_logger().info(
            f'[BT] SaveDockPose: saved dock at ({dock_pose["x"]:.2f}, {dock_pose["y"]:.2f})')
        return NodeStatus.SUCCESS
