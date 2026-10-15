# Web Bridge Node

[← ROS](../README.md)

The web bridge is the interface layer between frontend and ROS 2.
It combines:

- a FastAPI HTTP server,
- a WebSocket channel for live updates,
- and a ROS node that publishes/subscribes to robot topics.

Implementation: `ros2/src/roboclean/roboclean/web_bridge.py`.

## Main role in the system

The node acts as an orchestrator for three concerns:

1. **Mode control**
	- manages high-level mode (`idle`, `explore`, `clean`)
	- starts/stops explorer and cleaner launch processes

2. **State streaming**
	- streams pose, map, vacuum map, path, sensors, and system metrics to clients

3. **Command translation**
	- translates frontend requests into ROS actions, topic publishes, and service calls

## Runtime architecture

Two loops run in parallel:

- ROS thread:
  - runs `rclpy.spin()` for ROS subscriptions, timers, and publishers
- API thread:
  - runs FastAPI/Uvicorn for HTTP + WebSocket clients

A thread-safe queue is used for frontend commands (`cmd_vel`, cleaning commands, goal cancel, sensor requests), so API handlers stay responsive and ROS publishes happen safely in the ROS thread.

## Mode behavior

### Start explore

When exploration starts, web bridge:

1. switches mode to `explore`
2. stops cleaner process (if active)
3. stops current navigation and robot motion
4. prepares a fresh exploration state:
	- reset SLAM toolbox
	- clear global and local costmaps
	- delete persisted map artifacts for the configured basename
5. launches explorer

### Start clean

When cleaning starts, web bridge:

1. switches mode to `clean`
2. stops explorer process (if active)
3. launches cleaner

### Abort

Abort always:

- cancels active navigation goal,
- publishes zero velocity,
- terminates explorer and cleaner,
- sets mode back to `idle`.

## Data exposed to frontend

### Pose and maps

- Pose is read from TF and published at high rate.
- Exploration map and vacuum map are compressed (`zlib`) and sent as binary frames.
- Map metadata (width, height, resolution, origin) is sent with payload headers.

### Path and polygons

- Nav2 path updates are forwarded as simplified point lists.
- Cleaning polygon is accepted from frontend, published to ROS, and rebroadcast to clients.
- An auto-outline helper can derive a polygon from the largest wall contour in the current map.

### Sensors and metrics

- Sensor read requests are proxied via ROS topics and returned to API clients.
- Proximity values and system metrics are pushed as live updates.

## Command interface (grouped)

The API surface is intentionally broad, but behavior falls into a few groups:

- **Mode/process control**: start explore, start vacuum, abort
- **Navigation**: send `NavigateToPose` goal, cancel goal
- **Manual actuation**: velocity commands, vacuum/brush commands
- **Map operations**: map retrieval, persistent map+posegraph save
- **Cleaning area**: set polygon, auto-generate polygon, polygon coverage query
- **Status endpoints**: explorer/cleaner process status, current maps and pose
- **WebSocket**: initial snapshot + continuous live updates + action commands

## Lifelong map handling

Map persistence is aligned with the global project settings:

- directory: `MAP_DIR` (default `/opt/tmp/map`)
- basename: `MAP_BASENAME` (default `roboclean_map`)

Saving map from the web bridge triggers:

1. SLAM map save (`.yaml` + `.pgm`)
2. SLAM posegraph serialization (`.posegraph`)

These files are then reused by launch logic in lifelong/localization modes.

## API: 

### HTTP endpoints

| Method | Path | Purpose | Request body | Response (summary) |
|---|---|---|---|---|
| `GET` | `/` | Serve frontend `index.html` | none | HTML file or `{ "ERROR": ... }` |
| `GET` | `/pose` | Get latest robot pose | none | Pose object or `{ "ERROR": "No pose data yet" }` |
| `GET` | `/map` | Get latest exploration map | none | Serialized map or `{ "ERROR": "No map data yet" }` |
| `GET` | `/vacuum_map` | Get latest vacuum/coverage map | none | Serialized map or `{ "ERROR": "No vacuum map data yet" }` |
| `POST` | `/start-explore` | Switch to explore mode and launch explorer | none | `{ "status": "Explorer started", ... }` or `503/500` on reset/prep failure |
| `POST` | `/start-vacuum` | Switch to clean mode and launch cleaner | none | `{ "status": "Vacuum started" }` |
| `POST` | `/abort` | Cancel nav, stop motion, terminate explorer/cleaner, set idle | none | `{ "status": "idle" }` |
| `POST` | `/save-map` | Save persistent SLAM map and posegraph | none | Save report object or `503` when save/serialize fails |
| `POST` | `/move` | Send `NavigateToPose` goal | `{ x, y, qx, qy, qz, qw }` | `{ "status": "moving", "target": { ... } }` or `{ "error": ... }` |
| `POST` | `/manual_control` | Queue velocity command | `{ linear, angular }` | `{ "status": "sent", "linear": ..., "angular": ... }` |
| `POST` | `/api/cleaning` | Set vacuum/brush states | `{ vacuum, brush }` | `{ "status": "sent", "vacuum": ..., "brush": ... }` |
| `GET` | `/explorer_status` | Get explorer process status | none | `{ "status": "idle" \| "running" \| "finished" \| "ERROR: <code>" }` |
| `GET` | `/vacuum_status` | Get cleaner process status | none | `{ "status": "idle" \| "running" \| "finished" \| "ERROR: <code>" }` |
| `POST` | `/coverage_polygon` | Compute coverage metrics for a polygon on current maps | `{ points: [{ x, y }, ...] }` | `{ "status": "ok", ...coverage fields... }` |
| `POST` | `/cleaning_polygon` | Publish and store cleaning polygon | `{ points: [{ x, y }, ...] }` (min 3 points) | `{ "status": "sent", "polygon": { frame_id, points } }` or `400` |
| `POST` | `/auto-outline-polygon` | Auto-generate polygon from largest wall contour and set it as cleaning polygon | none | Same payload as `/cleaning_polygon`, or `400/422` |
| `GET` | `/api/sensor/{type}/{pin}` | Request one sensor reading via ROS | path params: `type` in `a`/`d`, `pin` int | `{ pin, type, key, value, timestamp }` or timeout/error |

### WebSocket endpoint

| Endpoint | Direction | Message shape | Notes |
|---|---|---|---|
| `/ws` | server -> client | `{"type":"initial_state","data":{"mode","pose","vacuum","brush"}}` | Sent immediately after connect |
| `/ws` | server -> client | binary map frame | Sent for latest exploration/vacuum map when available |
| `/ws` | server -> client | `{"type":"cleaning_polygon_update","data":...}` | Sent on connect (if polygon exists) and on polygon updates |
| `/ws` | client -> server | `{"action":"manual_control", ...}` | Internally handled like `POST /manual_control` |
| `/ws` | client -> server | `{"action":"start-explore"}` | Internally handled like `POST /start-explore` |
| `/ws` | client -> server | `{"action":"start-vacuum"}` | Internally handled like `POST /start-vacuum` |
| `/ws` | client -> server | `{"action":"abort"}` | Internally handled like `POST /abort` |
| `/ws` | client -> server | `{"action":"move", ...}` | Internally handled like `POST /move` |
| `/ws` | client -> server | `{"action":"cleaning", ...}` | Internally handled like `POST /api/cleaning` |