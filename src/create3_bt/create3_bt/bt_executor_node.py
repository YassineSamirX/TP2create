"""
Create3 BT Executor Node — Main Behavior Tree Engine
=====================================================
This is the heart of the system. It implements a Python-based
Behavior Tree that ticks at ~10 Hz and orchestrates the entire
Create3 mission: undock → patrol → return → dock.

The BT is implemented in pure Python (no BehaviorTree.CPP dependency)
using a lightweight BT framework defined here. This keeps the
project simple and avoids C++ compilation while maintaining the
BT patterns (Sequence, Fallback, ReactiveSequence, etc.).

Architecture:
  Root [ReactiveSequence]
  ├── ManualOverrideGuard [ReactiveFallback]
  │   ├── CheckNotManualOverride [Condition]
  │   └── TeleopPassthrough [Action]
  └── MissionSequence [Sequence]
      ├── UndockFallback [Fallback]
      │   ├── CheckAlreadyUndocked [Condition]
      │   └── UndockAction [Action]
      ├── SaveDockPose [Action]
      ├── StartPatrolTimer [Action]
      ├── PatrolSubtree [ReactiveSequence]
      │   ├── CheckPatrolTimeNotExpired [Condition]
      │   └── PatrolLoop [Fallback]
      │       ├── NormalPatrol [Sequence]
      │       │   ├── SelectNextWaypoint [Action]
      │       │   ├── NavigateToWaypoint [Action]
      │       │   └── PerformSquareAtWaypoint [Action]
      │       └── ObstacleRecovery [Sequence]
      │           ├── BoundaryFollowBehavior [Action]
      │           └── SetRecoveryWaypoint [Action]
      └── ReturnAndDockSubtree [Sequence]
          ├── NavigateToDockVicinity [Action]
          ├── DockAction [Action]
          └── SetMissionComplete [Action]
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import time

# BT framework (local module)
from create3_bt.bt_framework import (
    Sequence, ReactiveSequence, Fallback, ReactiveFallback, Blackboard
)

# BT leaf nodes (local modules)
from create3_bt.bt_nodes.check_dock_status import CheckIsDocked, CheckAlreadyUndocked
from create3_bt.bt_nodes.check_manual_override import CheckNotManualOverride
from create3_bt.bt_nodes.teleop_passthrough import TeleopPassthrough
from create3_bt.bt_nodes.undock_action import UndockAction
from create3_bt.bt_nodes.dock_action import DockAction
from create3_bt.bt_nodes.save_dock_pose import SaveDockPose
from create3_bt.bt_nodes.patrol_timer import StartPatrolTimer, CheckPatrolTimeNotExpired
from create3_bt.bt_nodes.select_waypoint import SelectNextWaypoint, SetRecoveryWaypoint
from create3_bt.bt_nodes.navigate_to_waypoint import NavigateToWaypoint, NavigateToDockVicinity
from create3_bt.bt_nodes.perform_square_action import PerformSquareAtWaypoint
from create3_bt.bt_nodes.boundary_follow_action import BoundaryFollowBehavior
from create3_bt.bt_nodes.set_mission_complete import SetMissionComplete


class BTExecutorNode(Node):
    """Main BT executor — builds the tree and ticks it."""

    def __init__(self):
        super().__init__('bt_executor')

        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('tick_rate', 10.0)          # Hz
        self.declare_parameter('patrol_duration', 120.0)   # seconds

        tick_rate = self.get_parameter('tick_rate').value
        patrol_duration = self.get_parameter('patrol_duration').value

        # Shared blackboard
        self.blackboard = Blackboard()
        self.blackboard.set('patrol_duration', patrol_duration)
        self.blackboard.set('mission_complete', False)
        self.blackboard.set('waypoint_index', 0)

        # Default waypoints (in odom frame, relative to dock)
        self.declare_parameter('waypoints', [
            1.0, 0.0,    # (x, y) waypoint 1
            1.0, 1.0,    # waypoint 2
            0.0, 1.0,    # waypoint 3
        ])
        wp_flat = self.get_parameter('waypoints').value
        waypoints = []
        for i in range(0, len(wp_flat), 2):
            waypoints.append((wp_flat[i], wp_flat[i + 1]))
        self.blackboard.set('waypoints', waypoints)

        # Build the behavior tree
        self.tree = self._build_tree()

        # Tick timer
        self.tick_timer = self.create_timer(
            1.0 / tick_rate, self.tick_callback,
            callback_group=self.cb_group)

        self.get_logger().info(
            f'BT Executor started. Tick rate: {tick_rate} Hz, '
            f'Patrol duration: {patrol_duration}s, '
            f'Waypoints: {waypoints}')

    def _build_tree(self):
        """Construct the full behavior tree."""

        # ── Manual Override Guard ──
        manual_override_guard = ReactiveFallback('ManualOverrideGuard', [
            CheckNotManualOverride('CheckNotManualOverride',
                                   self, self.blackboard),
            TeleopPassthrough('TeleopPassthrough',
                              self, self.blackboard),
        ])

        # ── Undock Subtree ──
        undock_subtree = Fallback('UndockFallback', [
            CheckAlreadyUndocked('CheckAlreadyUndocked',
                                 self, self.blackboard),
            UndockAction('UndockAction', self, self.blackboard),
        ])

        # ── Normal Patrol ──
        normal_patrol = Sequence('NormalPatrol', [
            SelectNextWaypoint('SelectNextWaypoint',
                               self, self.blackboard),
            NavigateToWaypoint('NavigateToWaypoint',
                               self, self.blackboard),
            PerformSquareAtWaypoint('PerformSquareAtWaypoint',
                                    self, self.blackboard),
        ])

        # ── Obstacle Recovery ──
        obstacle_recovery = Sequence('ObstacleRecovery', [
            BoundaryFollowBehavior('BoundaryFollowBehavior',
                                    self, self.blackboard),
            SetRecoveryWaypoint('SetRecoveryWaypoint',
                                self, self.blackboard),
        ])

        # ── Patrol Loop ──
        patrol_loop = Fallback('PatrolLoop', [
            normal_patrol,
            obstacle_recovery,
        ])

        # ── Patrol Subtree (time-limited) ──
        patrol_subtree = ReactiveSequence('PatrolSubtree', [
            CheckPatrolTimeNotExpired('CheckPatrolTimeNotExpired',
                                      self, self.blackboard),
            patrol_loop,
        ])

        # ── Return and Dock ──
        return_and_dock = Sequence('ReturnAndDockSubtree', [
            NavigateToDockVicinity('NavigateToDockVicinity',
                                   self, self.blackboard),
            DockAction('DockAction', self, self.blackboard),
            SetMissionComplete('SetMissionComplete',
                               self, self.blackboard),
        ])

        # ── Mission Sequence ──
        mission = Sequence('MissionSequence', [
            undock_subtree,
            SaveDockPose('SaveDockPose', self, self.blackboard),
            StartPatrolTimer('StartPatrolTimer', self, self.blackboard),
            patrol_subtree,
            return_and_dock,
        ])

        # ── Root ──
        root = ReactiveSequence('Root', [
            manual_override_guard,
            mission,
        ])

        return root

    def tick_callback(self):
        """Called at tick_rate Hz — one tick of the behavior tree."""
        if self.blackboard.get('mission_complete', False):
            return

        status = self.tree.tick()
        # Optionally log
        # self.get_logger().debug(f'BT tick -> {status}')


def main(args=None):
    rclpy.init(args=args)
    node = BTExecutorNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
