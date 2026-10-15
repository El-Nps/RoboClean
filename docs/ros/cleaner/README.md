# Cleaner Node

[← ROS](../README.md)

The cleaner node runs coverage cleaning over a selected area.
Its main behavior is:

1. wait for map and cleaning polygon,
2. call Boustrophedon decomposition to split the cleaning area into polygons,
3. generate sweep path for each polygon and add waypoints to the planned path,
4. send the waypoints one by one as a Nav2 goal to the NavigationClient,
5. continuously mark cleaned area,
6. shut down when the queue is finished.

## Overview:

At runtime, cleaner subscribes to:

- `/map` (OccupancyGrid)
- `/cleaning_polygon` (PolygonStamped)

and publishes:

- `/vacuum_map` (OccupancyGrid with visited/cleaned cells)

Navigation is delegated to Nav2 via `navigate_to_pose` action goals.

### Startup behavior

- On first map message, cleaner initializes an internal visited grid.
- Cleaning does not start until a valid polygon arrives on `/cleaning_polygon`.
- If robot pose TF is not available yet (`map -> base_link`), cleaner retries a few times.

Once all requirements are present, cleaner triggers smart clean planning.

## Core execution flow

The active mode is a queued path execution:

1. Build coverage path from map + polygon.
2. Convert waypoints into oriented `PoseStamped` goals (yaw follows path direction).
3. Push all poses into a goal queue.
4. Send first goal.
5. On each Nav2 result callback, send the next goal immediately.
6. When queue is empty, cleaning is considered complete and the node requests shutdown.

So cleaner is effectively a sequential goal dispatcher with local coverage tracking.

## Coverage tracking (vacuum map)

Independent from goal sending, cleaner runs a timer every 0.5s:

- Reads current robot pose from TF.
- Stamps a square footprint around the robot into the visited grid.
- Publishes the updated visited grid as `/vacuum_map`.

This provides a live cleaned-area map for visualization and coverage metrics.

## Path planning pipeline

Cleaner calls `CoveragePathGenerator.generate_path(...)` from:

- `ros2/src/roboclean/roboclean/path_planer.py`

Pipeline overview:

1. **Costmap preprocessing**
	- Convert occupancy data to free/occupied/unknown masks.
	- Apply opening/closing filters to reduce map noise.
	- Remove tiny obstacle blobs.

2. **Boustrophedon decomposition**
	- Convert selected cleaning area into a decomposed set of free-space cells.
	- Uses Ruby implementation in:
	  - `ros2/src/roboclean/roboclean/boustrophedon_decomposition.rb`
	- Output is a set of simpler polygonal regions.

3. **Sweep generation per region**
	- For each region, generate back-and-forth coverage lines using:
	  - `ros2/src/roboclean/roboclean/utils/sweep.py`
	- Sweep width is based on configured robot footprint width.

4. **Region stitching**
	- Connect region paths in an order/direction that minimizes transition distance. 
	- Reverse local path direction when it gives a shorter connection.

The result is a single waypoint list that cleaner executes with Nav2.

## Boustrophedon and sweep in one sentence each

- **Boustrophedon decomposition**: split complex free space into simpler cells that can be covered systematically while avoiding big obstacles (e.g. couches)
- **Sweep**: generate lane-like zig-zag passes inside each cell so the whole area is covered with minimal overlap.

## Notes for readers

- The node contains a fallback `clean_step` strategy for nearest unvisited-cell goals, but the current main behavior is the precomputed smart-clean queue.
- Cleaner is intentionally simple at runtime, as ROS and NAV2 can be demanding for the Raspberry Pi. Therefore, path calculations are handled using a map snapshot before the robot starts cleaning. That also means the path is planned without considering moved objects.

#### ToDo

For a more efficient cleaning path in large rooms, it makes sense to sort the polygons before calculating paths. This might be done with Ant Colony Optimization in order to find a close-to-ideal path to visit all polygons.
Also, as the robot does not follow the cleaning path as intended, it might be wise to:
A) Further optimize the robot behavior with a different planner node or adjustments to `nav2_params`
B) Run a second cleaner node after the first one has finished (like the old clean-step logic from the previous group) to cover remaining uncovered parts of the map and achieve close to 100% coverage
