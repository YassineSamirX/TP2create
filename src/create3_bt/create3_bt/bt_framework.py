"""
Lightweight Behavior Tree Framework for Python/ROS2
====================================================
Pure-Python BT implementation with the core node types needed
for BehaviorTree.CPP-compatible logic:

  - Sequence: ticks children left-to-right, fails on first FAILURE
  - ReactiveSequence: re-evaluates ALL children every tick
  - Fallback: tries children left-to-right, succeeds on first SUCCESS
  - ReactiveFallback: re-evaluates ALL children every tick
  - Condition: leaf that returns SUCCESS or FAILURE
  - ActionNode: leaf that returns RUNNING/SUCCESS/FAILURE
  - Blackboard: key-value store shared across all nodes
"""

from enum import Enum


class NodeStatus(Enum):
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'
    RUNNING = 'RUNNING'
    IDLE = 'IDLE'


class Blackboard:
    """Shared key-value store for all BT nodes."""

    def __init__(self):
        self._data = {}

    def set(self, key, value):
        self._data[key] = value

    def get(self, key, default=None):
        return self._data.get(key, default)

    def has(self, key):
        return key in self._data


# ──────────────────────────────────────────────
# Base classes
# ──────────────────────────────────────────────

class TreeNode:
    """Base class for all BT nodes."""

    def __init__(self, name):
        self.name = name
        self.status = NodeStatus.IDLE

    def tick(self):
        raise NotImplementedError

    def halt(self):
        self.status = NodeStatus.IDLE


class LeafNode(TreeNode):
    """Base for condition and action leaf nodes."""

    def __init__(self, name, ros_node, blackboard):
        super().__init__(name)
        self.ros_node = ros_node
        self.blackboard = blackboard
        self._initialized = False

    def on_init(self):
        """Called once when the node is first ticked (lazy init for subscriptions)."""
        pass

    def tick(self):
        if not self._initialized:
            self.on_init()
            self._initialized = True
        return self._tick_impl()

    def _tick_impl(self):
        raise NotImplementedError


class ConditionNode(LeafNode):
    """Returns SUCCESS or FAILURE — never RUNNING."""
    pass


class ActionNode(LeafNode):
    """Can return RUNNING — has on_start, on_running, on_halt lifecycle."""

    def __init__(self, name, ros_node, blackboard):
        super().__init__(name, ros_node, blackboard)
        self._started = False

    def on_start(self):
        """Called when the action begins."""
        return NodeStatus.RUNNING

    def on_running(self):
        """Called while the action is RUNNING."""
        return NodeStatus.RUNNING

    def on_halt(self):
        """Called when the action is interrupted."""
        pass

    def _tick_impl(self):
        if not self._started:
            self._started = True
            result = self.on_start()
            self.status = result
            return result
        else:
            result = self.on_running()
            self.status = result
            if result != NodeStatus.RUNNING:
                self._started = False
            return result

    def halt(self):
        if self._started:
            self.on_halt()
            self._started = False
        super().halt()


class SyncActionNode(LeafNode):
    """Synchronous action — completes in a single tick (SUCCESS or FAILURE)."""
    pass


# ──────────────────────────────────────────────
# Composite nodes
# ──────────────────────────────────────────────

class Sequence(TreeNode):
    """
    Ticks children left-to-right.
    - If a child returns FAILURE → halt remaining, return FAILURE.
    - If a child returns RUNNING → return RUNNING, resume from here next tick.
    - If all children return SUCCESS → return SUCCESS.
    """

    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
        self._current_child = 0

    def tick(self):
        while self._current_child < len(self.children):
            child = self.children[self._current_child]
            status = child.tick()

            if status == NodeStatus.RUNNING:
                self.status = NodeStatus.RUNNING
                return NodeStatus.RUNNING
            elif status == NodeStatus.FAILURE:
                self._halt_from(self._current_child + 1)
                self._current_child = 0
                self.status = NodeStatus.FAILURE
                return NodeStatus.FAILURE
            else:  # SUCCESS
                self._current_child += 1

        self._current_child = 0
        self.status = NodeStatus.SUCCESS
        return NodeStatus.SUCCESS

    def halt(self):
        self._halt_from(0)
        self._current_child = 0
        super().halt()

    def _halt_from(self, idx):
        for i in range(idx, len(self.children)):
            self.children[i].halt()


class ReactiveSequence(TreeNode):
    """
    Like Sequence, but re-evaluates ALL children from the beginning
    on every tick. If child[0] returns FAILURE, the rest are halted.
    Used for the root node to continuously check manual override.
    """

    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self):
        for i, child in enumerate(self.children):
            status = child.tick()

            if status == NodeStatus.RUNNING:
                # Halt any children after this one that might have been running
                for j in range(i + 1, len(self.children)):
                    if self.children[j].status == NodeStatus.RUNNING:
                        self.children[j].halt()
                self.status = NodeStatus.RUNNING
                return NodeStatus.RUNNING
            elif status == NodeStatus.FAILURE:
                # Halt remaining
                for j in range(i + 1, len(self.children)):
                    self.children[j].halt()
                self.status = NodeStatus.FAILURE
                return NodeStatus.FAILURE
            # SUCCESS → continue to next child

        self.status = NodeStatus.SUCCESS
        return NodeStatus.SUCCESS

    def halt(self):
        for child in self.children:
            child.halt()
        super().halt()


class Fallback(TreeNode):
    """
    Ticks children left-to-right.
    - If a child returns SUCCESS → return SUCCESS.
    - If a child returns RUNNING → return RUNNING.
    - If all children return FAILURE → return FAILURE.
    """

    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
        self._current_child = 0

    def tick(self):
        while self._current_child < len(self.children):
            child = self.children[self._current_child]
            status = child.tick()

            if status == NodeStatus.RUNNING:
                self.status = NodeStatus.RUNNING
                return NodeStatus.RUNNING
            elif status == NodeStatus.SUCCESS:
                self._halt_from(self._current_child + 1)
                self._current_child = 0
                self.status = NodeStatus.SUCCESS
                return NodeStatus.SUCCESS
            else:  # FAILURE
                self._current_child += 1

        self._current_child = 0
        self.status = NodeStatus.FAILURE
        return NodeStatus.FAILURE

    def halt(self):
        self._halt_from(0)
        self._current_child = 0
        super().halt()

    def _halt_from(self, idx):
        for i in range(idx, len(self.children)):
            self.children[i].halt()


class ReactiveFallback(TreeNode):
    """
    Like Fallback, but re-evaluates from the beginning every tick.
    Used for the ManualOverrideGuard.
    """

    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self):
        for i, child in enumerate(self.children):
            status = child.tick()

            if status == NodeStatus.RUNNING:
                for j in range(i + 1, len(self.children)):
                    if self.children[j].status == NodeStatus.RUNNING:
                        self.children[j].halt()
                self.status = NodeStatus.RUNNING
                return NodeStatus.RUNNING
            elif status == NodeStatus.SUCCESS:
                for j in range(i + 1, len(self.children)):
                    self.children[j].halt()
                self.status = NodeStatus.SUCCESS
                return NodeStatus.SUCCESS
            # FAILURE → try next child

        self.status = NodeStatus.FAILURE
        return NodeStatus.FAILURE

    def halt(self):
        for child in self.children:
            child.halt()
        super().halt()
