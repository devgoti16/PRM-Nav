# PRM-Nav (Project Z)

## Overview
**PRM-Nav** implements a **Probabilistic Roadmap (PRM)** navigation framework for a differential-drive robot in a planar environment with circular obstacles. The system receives goals via `/goal_pose` and executes smooth, collision-free trajectories while visualizing paths and obstacles in **RViz**.


## Features
- PRM-based path planning with random sampling and k-nearest neighbor connections
- Dijkstra’s algorithm for shortest path computation
- Path simplification and smoothing to reduce unnecessary waypoints
- Obstacle handling with inflation for safe navigation
- ROS2 node (`prm_navigator`) integrating planning, control, and visualization
- Simulation with `sim1` node for robot motion and RViz visualization
- Supports multiple environments with configurable obstacles


## Usage

1. Launch the Simulator

```bash
ros2 run sim sim1
```

Starts the simulation node
Robot pose published on /pose
Past robot positions visualized via /sim_markers

2. Launch RViz2

```bash
rviz2
```

Add the following displays:

RobotModel: Subscribe to /robot_description
MarkerArray: Subscribe to /sim_markers (robot trajectory)
MarkerArray: Subscribe to /obstacle_markers (obstacles)
Path: Subscribe to /planned_path (planned PRM path)

Set the fixed frame to world.

3. Launch the PRM Navigator Node

```bash
ros2 launch projectz navigator_launch.py obstacles:="0.0,2.0,0.8; -2.0,0.0,0.5; 2.0,-1.5,0.65; -1.5,-2.5,0.7"
```

Configure different environments by changing the obstacle string
Node subscribes to:
/pose → Current robot pose
/goal_pose → Target goal from RViz
Node publishes:
/cmd_vel → Velocity commands for the robot
/planned_path → Planned path for visualization
/obstacle_markers → Obstacle visualization in RViz

4. Set Goals in RViz
Use the “2D Goal Pose” tool
Click a location in the world to set the goal
Robot automatically plans and moves toward the goal

## Demonstration

https://github.com/user-attachments/assets/f1974a44-1c9d-4578-bf59-0eca4632e59e

## File Structure

```graphql
PRM-Nav/
├── projectz/
│   ├── navigator.py        # Main PRM navigator ROS2 node
│   ├── prm.py              # PRM planner implementation
│   ├── geometry.py         # Geometric utilities and collision checks
├── launch/
│   └── navigator_launch.py # Launch file with configurable obstacles
├── sim1.py                 # Robot simulation node
└── README.md               # Project documentation
```

## Results
Successfully navigates environments with 4, 5, and 6 obstacles

Handles straight-line, multi-turn, and unreachable goals

Maintains safe clearance and smooth trajectories

Path simplification and smoothing keeps paths near-optimal while avoiding collisions
