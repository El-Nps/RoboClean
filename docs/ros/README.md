
# ROS

[← Docs](../README.md)

This section documents the ROS 2 architecture of RoboClean. The version used is ROS 2 Jazzy.
Detailed behavior and workflows for the nodes are documented in dedicated pages:

- [Cleaner](cleaner/README.md)
- [Explorer](explorer/README.md)
- [Web bridge](web_bridge/README.md)

## Overview

The robot uses pre-existing ROS and Nav2 nodes to build the map, scan its surroundings using the LiDAR, and create and refine the occupancy grid. All nodes used and their parameters can be found in the [Nav2 parameter file](params/nav2_params.yaml).

## Launch architecture

There are two main launch modes.

### 1) Hardware launch

Launch file: `launch/robot.launch.py`

Starts:

- Robot state publisher (URDF/Xacro)
- Base controller (Arduino serial bridge)
- RPLidar driver
- LiDAR watchdog + wait-for-scan gate
- Nav2 bringup (started only after first scan)
- Web bridge
- System monitor

### 2) Simulation launch

Launch file: `launch/simulation.launch.py`

Starts:

- Gazebo (headless) with project world
- Robot spawn from URDF
- `ros_gz_bridge` for `/cmd_vel`, `/odom`, `/scan`, `/tf`, `/clock`
- Wait-for-scan gate before Nav2
- Nav2 bringup
- Web bridge
- System monitor

## Mapping and localization modes

Both main launches support persistent map operation via launch arguments:

- `map_mode`: `auto` | `mapping` | `lifelong` | `localization`
- `map_dir`: directory for map artifacts (default `/opt/tmp/map`)
- `map_name`: map base name (default `roboclean_map`)

Behavior summary:

- `auto`: chooses `lifelong` if map + posegraph files exist, otherwise `mapping`
- `mapping`: SLAM enabled, builds new map/posegraph
- `lifelong`: SLAM with existing map context and persisted posegraph. Map gets refined over time. (Part of SlamToolbox)
- `localization`: localization with AMCL against existing static map and saved initial pose metadata

### Frames and topics

- Global frame: `map`
- Local/odometry frame: `odom`
- Robot base frame: mostly `base_footprint` (behavior/collision parts use `base_link`)
- LiDAR topic: `/scan`
- Odometry topic: `/odom`

### Navigator and behavior tree

- Navigator plugins:
	- `navigate_to_pose`
	- `navigate_through_poses`
	Note: We tried using navigate_through_poses for the cleaner node to get a more efficient path, but the robot behaved unexpectedly. Therefore, we went back to using a goal queue with navigate_to_pose.

[Cleaner details →](cleaner/README.md)
