"""
BoundaryFollowBehavior — Action Node (Action Client)
=====================================================
Calls the /boundary_follow action server (Assignment 2 reuse).
Used as a recovery behavior when normal navigation hits an obstacle.
"""

from rclpy.action import ActionClient
from create3_bt.bt_framework import ActionNode, NodeStatus
from custom_interfaces.action import BoundaryFollow


class BoundaryFollowBehavior(ActionNode):
    """Call /boundary_follow action — Assignment 2 reuse as BT recovery node."""

    def on_init(self):
        self._action_client = ActionClient(
            self.ros_node, BoundaryFollow, '/boundary_follow')
        self._goal_handle = None
        self._result_future = None
        self._done = False
        self._success = False
        self.ros_node.get_logger().info(
            '[BT] BoundaryFollowBehavior: action client ready')

    def on_start(self):
        self.ros_node.get_logger().info(
            '[BT] BoundaryFollowBehavior: starting wall-following recovery')

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.ros_node.get_logger().warn(
                '[BT] BoundaryFollowBehavior: /boundary_follow server not available!')
            return NodeStatus.FAILURE

        goal = BoundaryFollow.Goal()
        goal.duration = 15.0  # follow wall for 15 seconds

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
                '[BT] BoundaryFollowBehavior: goal rejected')
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
            f'[BT] BoundaryFollowBehavior: done, '
            f'success={result.success}, dist={result.distance_traveled:.2f}m')

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.ros_node.get_logger().debug(
            f'[BT] BoundaryFollow: {fb.elapsed_time:.0f}s, '
            f'min_range={fb.min_range:.2f}m')

    def on_running(self):
        if self._done:
            if self._success:
                return NodeStatus.SUCCESS
            return NodeStatus.FAILURE
        return NodeStatus.RUNNING

    def on_halt(self):
        if self._goal_handle is not None:
            self.ros_node.get_logger().info(
                '[BT] BoundaryFollowBehavior: cancelling boundary_follow')
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._result_future = None
        self._done = False
