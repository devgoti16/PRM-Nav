# PRM-Nav (Project Z)

## Overview
PRM-Nav implements a **Probabilistic Roadmap (PRM)** navigation framework for a differential-drive robot in a planar environment with circular obstacles. The system receives goals via `/goal_pose` and executes smooth, collision-free trajectories while visualizing paths and obstacles in RViz.

This project was developed for **CSCE 752: Advanced Robotics**.

---

## Features
- PRM-based path planning with random sampling and k-nearest neighbor connections
- Dijkstra’s algorithm for shortest path computation
- Path simplification and smoothing to reduce unnecessary waypoints
- Obstacle handling with inflation for safe navigation
- ROS2 node (`prm_navigator`) integrating planning, control, and visualization
- Simulation with `sim1` node for robot motion and RViz visualization
- Supports multiple environments with configurable obstacles

---

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/devgoti16/PRM-Nav.git
   cd PRM-Nav
