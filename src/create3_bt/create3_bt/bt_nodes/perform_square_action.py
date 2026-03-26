"""
PerformSquareAtWaypoint — Action Node (Action Client)
=====================================================
Calls the /draw_square action server (Assignment 1 reuse).
This is a BT leaf node that acts as an action client.
"""

import math
from rclpy.action import ActionClient
from create3_bt.bt_framework import ActionNode, NodeStatus
from custom_interfaces.action import DrawSquare


class PerformSquareAtWaypoint(ActionNode):
    """Call /draw_square action — Assignment 1 reuse as BT leaf node."""

    def on_init(self):
        self._action_client = ActionClient(
            self.ros_node, DrawSquare, '/draw_square')
        self._goal_handle = None
        self._result_future = None
        self._done = False
        self._success = False
        self.ros_node.get_logger().info(
            '[BT] PerformSquareAtWaypoint: action client ready')

    def on_start(self):
        self.ros_node.get_logger().info(
            '[BT] PerformSquareAtWaypoint: sending draw_square goal')

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.ros_node.get_logger().warn(
                '[BT] PerformSquareAtWaypoint: /draw_square server not available!')
            return NodeStatus.FAILURE

        goal = DrawSquare.Goal()
        goal.side_length = 0.4  # 40cm sides — reasonable for Create3

        self._done = False
        self._success = False

        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        send_goal_future.add_done_callback(self._goal_response_cb)

        return NodeStatus.RUNNING

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.ros_node.get_logger().warn(
                '[BT] PerformSquareAtWaypoint: goal rejected')
            self._done = True
            self._success = False
            return

        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self._done = True
        self._success = result.success
        self.ros_node.get_logger().info(
            f'[BT] PerformSquareAtWaypoint: done, '
            f'success={result.success}, dist={result.total_distance:.2f}m')

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.ros_node.get_logger().debug(
            f'[BT] DrawSquare: side {fb.current_side}/4, '
            f'{fb.percent_complete:.0f}%')

    def on_running(self):
        if self._done:
            if self._success:
                return NodeStatus.SUCCESS
            return NodeStatus.FAILURE
        return NodeStatus.RUNNING

    def on_halt(self):
        if self._goal_handle is not None:
            self.ros_node.get_logger().info(
                '[BT] PerformSquareAtWaypoint: cancelling draw_square')
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._result_future = None
        self._done = False
