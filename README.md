# Turtle Boundary Controller — Assignment 2

A ROS2 package that makes a turtlesim turtle autonomously draw the boundaries of its domain. The behavior is interruptible: press **SPACE** at any time to take manual control via teleop.

## Architecture

A **Finite State Machine (FSM)** drives the turtle through these states:

| State | Description | Pen |
|---|---|---|
| `MOVE_TO_BOUNDARY` | Drives straight toward the nearest wall | UP |
| `FOLLOW_BOUNDARY` | Traces the perimeter rectangle counter-clockwise | DOWN |
| `RETURN_HOME` | Navigates back to initial position | UP |
| `IDLE` | Done — turtle stops | — |
| `MANUAL` | Teleop mode (entered/exited via SPACE) | unchanged |

### Nodes

- **draw_boundaries_node** — Main FSM controller. Subscribes to `/turtle1/pose` and `/keyboard_input`, publishes to `/turtle1/cmd_vel`, calls `/turtle1/set_pen`.
- **keyboard_listener** — Captures spacebar presses from terminal, publishes `toggle_manual` on `/keyboard_input`.
- **turtlesim_node** — Standard turtlesim.
- **turtle_teleop_key** — Standard turtlesim teleop for manual control.

## Installation & Build

```bash
cd ~/robotics/ROS-TP2
colcon build
source install/setup.bash
```

## Running

### Launch everything at once

```bash
ros2 launch turtle_boundary_controller draw_boundaries.launch.py
```

This opens:
- The turtlesim window
- An xterm for the keyboard listener (press SPACE here to toggle manual mode)
- An xterm for turtle_teleop_key (use arrow keys when in manual mode)

### Launch options

| Option | Default | Description |
|---|---|---|
| `speed` | `2.0` | Maximum linear/angular speed |
| `boundary_margin` | `0.5` | Distance from wall edge defining the boundary |

Example:
```bash
ros2 launch turtle_boundary_controller draw_boundaries.launch.py speed:=1.5 boundary_margin:=1.0
```

## Usage

1. Launch the system — the turtle automatically starts moving toward the nearest wall
2. When it reaches the boundary, pen goes down and it traces the full perimeter counter-clockwise
3. After completing the rectangle, pen goes up and the turtle returns to its starting position
4. **At any time**, press SPACE in the keyboard listener terminal to enter manual mode
5. In manual mode, use arrow keys in the teleop terminal to move the turtle
6. Press SPACE again to resume autonomous boundary-following from the current position

## Dependencies

- ROS2 Jazzy
- `turtlesim`
- `xterm` (for the launch file to open separate terminal windows)

Install xterm if needed:
```bash
sudo apt install xterm
```
