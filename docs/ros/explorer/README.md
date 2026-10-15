# Explorer Node

[← ROS](../README.md)

The explorer is an autonomous exploration node.
Its job is simple: pick a promising frontier on the current map, send a `NavigateToPose` goal to Nav2, and repeat until no useful frontier remains.

Implementation: `ros2/src/roboclean/roboclean/explorer_node.py`.

## Inputs and outputs

Main subscriptions/services/actions:

- Subscribes to:
	- `/map` (`nav_msgs/OccupancyGrid`, transient-local QoS)
	- `/odom` (`nav_msgs/Odometry`) for distance metrics
- Uses TF to get robot pose in map frame.
- Sends action goals to `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`).
- Calls SLAM Toolbox services:
	- `/slam_toolbox/save_map`
	- `/slam_toolbox/serialize_map`

## Frontier detection strategy

Frontiers are computed directly from the occupancy grid with a vectorized approach:

- Free cells: value `0`
- Unknown cells: value `-1`
- Walls/occupied: value `100`

A free cell is considered frontier if:

- it has at least 3 unknown neighbors (3x3 neighborhood), and
- it has fewer than 3 wall neighbors.

## Goal selection logic

For each frontier cluster, the node evaluates candidate points by:

1. Sorting clusters by distance to the robot.
2. Using a distance transform to reject points too close to walls.
3. Preferring points that are further from walls.
4. Skipping points that are currently blacklisted.

The first candidate that passes these checks becomes the goal.
Goal orientation (`yaw`) is set so the robot faces the target point.

## Completion condition

Exploration stops when the number of detected frontier clusters is too low (`<= min_number_frontiers`, default 1), or when no safe non-blacklisted frontier candidate remains.

At completion, the map saver is triggered and the map is saved at the MAP_DIR (default in .env.example)
