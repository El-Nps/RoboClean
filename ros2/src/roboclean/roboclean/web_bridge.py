import asyncio
import base64
import glob
import json
import os
import signal
import struct
import threading
import time
import zlib
import numpy as np
import queue
from contextlib import suppress

from pydantic import BaseModel
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

import rclpy
from action_msgs.srv import CancelGoal as CancelGoalSrv
from geometry_msgs.msg import PolygonStamped, Point32, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import Reset as SlamReset, SaveMap, SerializePoseGraph
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import Buffer, LookupException, TimeoutException, TransformException, TransformListener
from .utils.room_segmentation import extract_largest_wall_outline_polygon

from skimage.draw import disk as sk_disk

latest_pose = None
latest_map = None
latest_vacuum_map = None
latest_keepout_map = None
latest_cleaning_polygon = None

current_mode = "idle"

explorer_process = None
cleaner_process = None

main_event_loop = None
connected_clients = set()

sensor_values = {}
sensor_events = {}
ros_command_queue = queue.Queue()

class MoveCommand(BaseModel):
    x: float
    y: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

class ManualControlCommand(BaseModel):
    linear: float = 0.0
    angular: float = 0.0

class CleaningCommand(BaseModel):
    vacuum: int = 0
    brush: int = 0

MAP_STORAGE_DIR = os.getenv("MAP_DIR", "/opt/tmp/map")
MAP_STORAGE_BASENAME = os.getenv("MAP_BASENAME", "roboclean_map")

LEDGE_THRESHOLD = int(os.getenv("LEDGE_THRESHOLD", "100"))
LEDGE_DEBOUNCE_S = float(os.getenv("LEDGE_DEBOUNCE_S", "2.0"))
LEDGE_OFFSET_M = float(os.getenv("LEDGE_OFFSET_M", "0.15"))
LEDGE_HW_M = float(os.getenv("LEDGE_HW_M", "0.1"))
LEDGE_HD_M = float(os.getenv("LEDGE_HD_M", "0.05"))
KEEPOUT_FILL = 100
KEEPOUT_DTYPE = np.int8
DEFAULT_STROKE_RADIUS_M = float(os.getenv("DEFAULT_STROKE_RADIUS", "0.075"))

async def broadcast(message, is_binary=False):
    if not connected_clients:
        return
        
    async def send(ws):
        try:
            await (ws.send_bytes(message) if is_binary else ws.send_json(message))
            return None
        except Exception:
            return ws
            
    dead_sockets = await asyncio.gather(*(send(ws) for ws in connected_clients))
    connected_clients.difference_update(filter(None, dead_sockets))

def schedule_broadcast(message, is_binary=False):
    if main_event_loop and main_event_loop.is_running():
        asyncio.run_coroutine_threadsafe(broadcast(message, is_binary), main_event_loop)

async def set_mode(new_mode: str):
    global current_mode
    current_mode = new_mode
    await broadcast({"type": "mode_update", "data": current_mode})

def delete_persistent_map_files(logger=None):
    os.makedirs(MAP_STORAGE_DIR, exist_ok=True)
    deleted_files = []
    for path in glob.glob(os.path.join(MAP_STORAGE_DIR, f"{MAP_STORAGE_BASENAME}*")):
        if not os.path.isfile(path):
            continue
        try:
            os.remove(path)
            deleted_files.append(path)
        except OSError as exc:
            if logger is not None:
                logger.warn(f"Failed to delete persistent map file '{path}': {exc}")
            continue
    return deleted_files

def get_map_info(msg):
    return {
        "width": msg.info.width,
        "height": msg.info.height,
        "resolution": msg.info.resolution,
        "origin": {
            "position": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y,
                "z": msg.info.origin.position.z,
            },
            "orientation": {
                "x": msg.info.origin.orientation.x,
                "y": msg.info.origin.orientation.y,
                "z": msg.info.origin.orientation.z,
                "w": msg.info.origin.orientation.w,
            },
        },
    }

def serialize_map(msg: OccupancyGrid):
    if msg is None:
        return None
    raw_bytes = msg.data.tobytes()
    data = zlib.compress(raw_bytes)
    return {
        "info": get_map_info(msg),
        "compression": "zlib",
        "data_b64": base64.b64encode(data).decode("utf-8"),
    }


def _world_to_grid(wx: float, wy: float, map_info):
    resolution = float(map_info.resolution)
    if resolution <= 0.0:
        raise ValueError("invalid map resolution")
    gx = (float(wx) - float(map_info.origin.position.x)) / resolution
    gy = (float(wy) - float(map_info.origin.position.y)) / resolution
    return gx, gy


def _grid_to_world(gx: float, gy: float, map_info):
    resolution = float(map_info.resolution)
    wx = float(map_info.origin.position.x) + float(gx) * resolution
    wy = float(map_info.origin.position.y) + float(gy) * resolution
    return wx, wy


def _point_in_polygon(point_x: float, point_y: float, polygon_points):
    inside = False
    n = len(polygon_points)
    if n < 3:
        return False

    j = n - 1
    for i in range(n):
        xi, yi = polygon_points[i]
        xj, yj = polygon_points[j]
        intersects = ((yi > point_y) != (yj > point_y)) and (
            point_x < ((xj - xi) * (point_y - yi)) / ((yj - yi) if (yj - yi) != 0 else 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def compute_polygon_coverage(points):
    if latest_map is None or latest_vacuum_map is None:
        raise HTTPException(status_code=503, detail="Coverage unavailable: map data not ready")

    if not isinstance(points, list) or len(points) < 3:
        raise HTTPException(status_code=400, detail="Need at least 3 polygon points")

    try:
        polygon_world = [(float(p["x"]), float(p["y"])) for p in points]
    except (TypeError, ValueError, KeyError) as exc:
        raise HTTPException(status_code=400, detail="Invalid point format. Expected {'x': number, 'y': number}") from exc

    map_w = int(latest_map.info.width)
    map_h = int(latest_map.info.height)
    vac_w = int(latest_vacuum_map.info.width)
    vac_h = int(latest_vacuum_map.info.height)

    map_data = np.asarray(latest_map.data, dtype=np.int16)
    vac_data = np.asarray(latest_vacuum_map.data, dtype=np.int16)

    if map_data.size != map_w * map_h:
        raise HTTPException(status_code=500, detail="Coverage unavailable: invalid exploration map size")
    if vac_data.size != vac_w * vac_h:
        raise HTTPException(status_code=500, detail="Coverage unavailable: invalid vacuum map size")

    map_grid = map_data.reshape((map_h, map_w))
    vac_grid = vac_data.reshape((vac_h, vac_w))

    polygon_grid = [
        _world_to_grid(wx, wy, latest_map.info)
        for (wx, wy) in polygon_world
    ]

    min_x = max(0, int(np.floor(min(p[0] for p in polygon_grid))))
    max_x = min(map_w - 1, int(np.ceil(max(p[0] for p in polygon_grid))))
    min_y = max(0, int(np.floor(min(p[1] for p in polygon_grid))))
    max_y = min(map_h - 1, int(np.ceil(max(p[1] for p in polygon_grid))))

    if min_x > max_x or min_y > max_y:
        return {"free_cells": 0, "covered_cells": 0, "coverage_percent": 0.0}

    free_cells = 0
    covered_cells = 0

    for y in range(min_y, max_y + 1):
        for x in range(min_x, max_x + 1):
            if map_grid[y, x] != 0:
                continue

            center_x = x + 0.5
            center_y = y + 0.5
            if not _point_in_polygon(center_x, center_y, polygon_grid):
                continue

            free_cells += 1
            wx, wy = _grid_to_world(center_x, center_y, latest_map.info)
            vac_gx, vac_gy = _world_to_grid(wx, wy, latest_vacuum_map.info)
            vac_x = int(np.floor(vac_gx))
            vac_y = int(np.floor(vac_gy))

            if 0 <= vac_x < vac_w and 0 <= vac_y < vac_h and vac_grid[vac_y, vac_x] == 100:
                covered_cells += 1

    coverage_percent = (covered_cells * 100.0 / free_cells) if free_cells > 0 else 0.0
    return {
        "free_cells": int(free_cells),
        "covered_cells": int(covered_cells),
        "coverage_percent": float(coverage_percent),
    }

async def terminate_process(proc):
    if not proc or proc.returncode is not None:
        return

    with suppress(ProcessLookupError):
        os.killpg(proc.pid, signal.SIGINT) # Graceful shutdown
        try:
            await asyncio.wait_for(proc.wait(), timeout=3.0)
        except asyncio.TimeoutError:
            os.killpg(proc.pid, signal.SIGKILL) # Force kill

def serialize_cleaning_polygon(msg: PolygonStamped):
    if msg is None:
        return None
    return {
        "frame_id": msg.header.frame_id,
        "points": [
            {"x": p.x, "y": p.y}
            for p in msg.polygon.points
        ]
    }

async def run_node(name):
    global explorer_process, cleaner_process
    launch_files = {
        "explorer": "explorer_launch.py",
        "cleaner": "cleaner_launch.py"
    }

    if name not in launch_files: return

    use_sim_time = getattr(app.state.ros_node, 'use_sim_time', False)
    cmd = ["ros2", "launch", "roboclean", launch_files[name]]
    
    if use_sim_time:
        cmd.append("use_sim_time:=True")
    if name == "explorer":
        cmd.append(f"persistent_map_dir:={MAP_STORAGE_DIR}")
        cmd.append(f"persistent_map_name:={MAP_STORAGE_BASENAME}")

    proc = await asyncio.create_subprocess_exec(*cmd, start_new_session=True)

    if name == "explorer": explorer_process = proc
    else: cleaner_process = proc

    await proc.wait()

    process_status = "finished" if proc.returncode == 0 else f"ERROR: {proc.returncode}"
    await broadcast({
        "type": "status_update",
        "data": {
            "node": name,
            "status": process_status,
        },
    })

    if (name == "explorer" and current_mode == "explore") or \
       (name == "cleaner" and current_mode == "clean"):
        await set_mode("idle")

    if name == "explorer": explorer_process = None
    else: cleaner_process = None

class ROS2Bridge(Node):
    def __init__(self):
        super().__init__("web_bridge")
        self.last_ledge_time = 0.0

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        self.last_broadcast = {"odom": 0.0, "map": 0.0, "vacuum": 0.0, "keepout": 0.0}
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.create_subscription(OccupancyGrid, "/vacuum_map", self.vacuum_callback, 10)
        self.create_subscription(OccupancyGrid, "/keepout_filter_mask", self.keepout_callback, map_qos)
        self.create_subscription(Path, "/plan", self.path_callback, 10)
        self.cleaning_polygon_pub = self.create_publisher(PolygonStamped, '/cleaning_polygon', map_qos)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cancel_client = self.create_client(CancelGoalSrv, '/navigate_to_pose/_action/cancel_goal')
        self.slam_reset_client = self.create_client(SlamReset, '/slam_toolbox/reset')
        self.map_saver_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.posegraph_serializer_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.clean_pub = self.create_publisher(Int32MultiArray, '/cleaning_commands', 10)
        self.sensor_req_pub = self.create_publisher(String, '/sensor_req', 10)
        self.create_subscription(String, '/sensor_data', self.sensor_callback, 10)
        self.create_subscription(Int32MultiArray, '/proximity_sensors', self.proximity_callback, 10)
        self.create_subscription(String, '/system_metrics', self.system_metrics_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.create_timer(0.1, self.publish_pose)
        self.create_timer(0.05, self.process_command_queue)

    def process_command_queue(self):
        while not ros_command_queue.empty():
            cmd = ros_command_queue.get()
            if cmd['type'] == 'cmd_vel':
                msg = Twist()
                msg.linear.x = cmd['linear']
                msg.angular.z = cmd['angular']
                self.cmd_vel_pub.publish(msg)
            elif cmd['type'] == 'cleaning':
                msg = Int32MultiArray()
                msg.data = [cmd['vacuum'], cmd['brush']]
                self.clean_pub.publish(msg)
            elif cmd['type'] == 'cancel_goal':
                req = CancelGoalSrv.Request()
                self.cancel_client.call_async(req)
            elif cmd['type'] == 'sensor_req':
                self.sensor_req_pub.publish(String(data=cmd['req_str']))

    def prepare_fresh_exploration(self):
        slam_reset_ok = self._call_service_and_wait(
            self.slam_reset_client,
            SlamReset.Request(),
            '/slam_toolbox/reset',
        )
        global_clear_ok = self._call_service_and_wait(
            self.clear_global_costmap_client,
            ClearEntireCostmap.Request(),
            '/global_costmap/clear_entirely_global_costmap',
        )
        local_clear_ok = self._call_service_and_wait(
            self.clear_local_costmap_client,
            ClearEntireCostmap.Request(),
            '/local_costmap/clear_entirely_local_costmap',
        )
        deleted_files = delete_persistent_map_files(self.get_logger())

        self.get_logger().info(
            "Explore reset summary: "
            f"slam_reset={slam_reset_ok}, "
            f"global_costmap_cleared={global_clear_ok}, "
            f"local_costmap_cleared={local_clear_ok}, "
            f"deleted_files={len(deleted_files)}"
        )

        return {
            "slam_reset": slam_reset_ok,
            "global_costmap_cleared": global_clear_ok,
            "local_costmap_cleared": local_clear_ok,
            "deleted_map_files": len(deleted_files),
        }

    def save_persistent_map_and_posegraph(self):
        os.makedirs(MAP_STORAGE_DIR, exist_ok=True)
        map_url = os.path.join(MAP_STORAGE_DIR, MAP_STORAGE_BASENAME)
        posegraph_file = f"{map_url}.posegraph"

        if not self.map_saver_client.wait_for_service(timeout_sec=5.0):
            return {
                "status": "map_save_unavailable",
                "map_saved": False,
                "posegraph_saved": False,
                "error": "Map saver service /slam_toolbox/save_map not available",
            }

        save_request = SaveMap.Request()
        save_request.name.data = map_url
        save_future = self.map_saver_client.call_async(save_request)

        save_done = threading.Event()
        save_future.add_done_callback(lambda _: save_done.set())
        if not save_done.wait(10.0):
            return {
                "status": "map_save_timeout",
                "map_saved": False,
                "posegraph_saved": False,
                "error": "Timed out while saving map",
            }

        if save_future.exception() is not None:
            return {
                "status": "map_save_error",
                "map_saved": False,
                "posegraph_saved": False,
                "error": str(save_future.exception()),
            }

        save_response = save_future.result()
        result_value = getattr(save_response, 'result', True)
        if isinstance(result_value, bool):
            map_saved = result_value
        elif isinstance(result_value, int):
            map_saved = (result_value == getattr(save_response, 'RESULT_SUCCESS', 0))
        else:
            map_saved = True

        if not map_saved:
            return {
                "status": "map_save_failed",
                "map_saved": False,
                "posegraph_saved": False,
                "error": f"Map saver reported failure for {map_url}.yaml/.pgm",
            }

        if not self.posegraph_serializer_client.wait_for_service(timeout_sec=5.0):
            return {
                "status": "posegraph_save_unavailable",
                "map_saved": True,
                "posegraph_saved": False,
                "error": "Serialize service /slam_toolbox/serialize_map not available",
            }

        posegraph_request = SerializePoseGraph.Request()
        posegraph_request.filename = posegraph_file
        posegraph_future = self.posegraph_serializer_client.call_async(posegraph_request)

        posegraph_done = threading.Event()
        posegraph_future.add_done_callback(lambda _: posegraph_done.set())
        if not posegraph_done.wait(10.0):
            return {
                "status": "posegraph_save_timeout",
                "map_saved": True,
                "posegraph_saved": False,
                "error": "Timed out while serializing posegraph",
            }

        if posegraph_future.exception() is not None:
            return {
                "status": "posegraph_save_error",
                "map_saved": True,
                "posegraph_saved": False,
                "error": str(posegraph_future.exception()),
            }

        posegraph_response = posegraph_future.result()
        posegraph_result = getattr(posegraph_response, 'result', 255)
        posegraph_saved = (posegraph_result == 0)

        if not posegraph_saved:
            return {
                "status": "posegraph_save_failed",
                "map_saved": True,
                "posegraph_saved": False,
                "error": f"SLAM posegraph serialization failed for {posegraph_file} (result={posegraph_result})",
            }

        return {
            "status": "saved",
            "map_saved": True,
            "posegraph_saved": True,
            "map_path": f"{map_url}.yaml/.pgm",
            "posegraph_path": posegraph_file,
        }

    def _call_service_and_wait(self, client, request, service_name: str, timeout_sec: float = 2.0) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(f"Service unavailable during exploration reset: {service_name}")
            return False

        future = client.call_async(request)
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())

        if not done_event.wait(timeout_sec):
            self.get_logger().warn(f"Service call timed out during exploration reset: {service_name}")
            return False

        if future.exception() is not None:
            self.get_logger().warn(
                f"Service call failed during exploration reset: {service_name}: {future.exception()}"
            )
            return False

        response = future.result()
        if hasattr(response, "success"):
            return bool(getattr(response, "success"))

        return True

    def publish_pose(self):
        global latest_pose
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            latest_pose = {
                "x": t.transform.translation.x,
                "y": t.transform.translation.y,
                "z": t.transform.translation.z,
                "qx": t.transform.rotation.x,
                "qy": t.transform.rotation.y,
                "qz": t.transform.rotation.z,
                "qw": t.transform.rotation.w,
            }

            if (time.time() - self.last_broadcast["odom"]) > 0.1:
                schedule_broadcast({"type": "pose_update", "data": latest_pose})
                self.last_broadcast["odom"] = time.time()

        except (LookupException, TimeoutException, TransformException):
            pass

    def process_map(self, msg, map_type):
        raw_bytes = msg.data.tobytes()
        compressed_data = zlib.compress(raw_bytes)
        meta = json.dumps(
            {"type": "map_update", "map_type": map_type, **get_map_info(msg)}
        ).encode("utf-8")
        return struct.pack(f"<I", len(meta)) + meta + compressed_data

    def map_callback(self, msg):
        global latest_map
        latest_map = msg
        if (time.time() - self.last_broadcast["map"]) > 1.0:
            self.last_broadcast["map"] = time.time()
            
            async def compress_and_send():
                compressed_data = await asyncio.to_thread(self.process_map, msg, "exploration")
                await broadcast(compressed_data, True)
                
            if main_event_loop and main_event_loop.is_running():
                asyncio.run_coroutine_threadsafe(compress_and_send(), main_event_loop)

    def vacuum_callback(self, msg):
        global latest_vacuum_map
        latest_vacuum_map = msg
        if (time.time() - self.last_broadcast["vacuum"]) > 1.0:
            self.last_broadcast["vacuum"] = time.time()

            async def compress_and_send():
                compressed_data = await asyncio.to_thread(self.process_map, msg, "vacuum")
                await broadcast(compressed_data, True)
                
            if main_event_loop and main_event_loop.is_running():
                asyncio.run_coroutine_threadsafe(compress_and_send(), main_event_loop)

    def keepout_callback(self, msg):
        global latest_keepout_map
        latest_keepout_map = msg
        if (time.time() - self.last_broadcast.get("keepout", 0.0)) > 1.0:
            self.last_broadcast["keepout"] = time.time()

            async def compress_and_send():
                compressed_data = await asyncio.to_thread(self.process_map, msg, "keepout")
                await broadcast(compressed_data, True)
                
            if main_event_loop and main_event_loop.is_running():
                asyncio.run_coroutine_threadsafe(compress_and_send(), main_event_loop)

    def path_callback(self, msg):
        points = []
        last_added = None
        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            if last_added is None or ((p.x - last_added.x)**2 + (p.y - last_added.y)**2) > 0.01:
                points.append({"x": p.x, "y": p.y})
                last_added = p

        if len(msg.poses) > 0:
            last_pose = msg.poses[-1].pose.position
            if last_added is None or last_added.x != last_pose.x or last_added.y != last_pose.y:
                points.append({"x": last_pose.x, "y": last_pose.y})

        schedule_broadcast({"type": "path_update", "data": points})

    def sensor_callback(self, msg):
        try:
            parts = msg.data.split(':')
            if len(parts) == 2:
                key = parts[0].strip()
                val = parts[1].strip()
                sensor_values[key] = val
                if key in sensor_events:
                    sensor_events[key].set()
                schedule_broadcast({"type": "sensor_update", "data": {key: val}})
        except Exception as e:
            self.get_logger().warn(f"Failed to parse sensor data: {e}")

    def proximity_callback(self, msg):
        schedule_broadcast({"type": "proximity_update", "data": list(msg.data)})

    def system_metrics_callback(self, msg):
        try:
            metrics = json.loads(msg.data)
            schedule_broadcast({"type": "system_metrics", "data": metrics})
        except Exception as e:
            self.get_logger().warn(f"Failed to parse system metrics: {e}")


FRONTEND_PATH = os.getenv("FRONTEND_PATH", "/opt/frontend")
STATIC_PATH = os.path.join(FRONTEND_PATH, "assets")
INDEX_PATH = os.path.join(FRONTEND_PATH, "index.html")

app = FastAPI()
app.add_middleware(GZipMiddleware, minimum_size=1000)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount("/assets", StaticFiles(directory=os.path.join(FRONTEND_PATH, "assets")), name="assets")
app.mount("/src", StaticFiles(directory=os.path.join(FRONTEND_PATH, "src")), name="src")

@app.on_event("startup")
async def startup_event():
    global main_event_loop
    main_event_loop = asyncio.get_running_loop()
    
    app.state.cleaning_state = {"vacuum": 0, "brush": 0}    

@app.get("/")
async def serve_frontend():
    if os.path.exists(INDEX_PATH):
        return FileResponse(INDEX_PATH)
    return {"ERROR": f"Frontend not found at {INDEX_PATH}"}

@app.get("/pose")
async def get_pose():
    return latest_pose or {"ERROR": "No pose data yet"}

@app.get("/map")
async def get_map():
    return serialize_map(latest_map) or {"ERROR": "No map data yet"}

@app.get("/vacuum_map")
async def get_vacuum_map():
    return serialize_map(latest_vacuum_map) or {"ERROR": "No vacuum map data yet"}

@app.post("/start-explore")
async def start_explorer():
    global explorer_process, cleaner_process, latest_map, latest_vacuum_map
    
    if current_mode == "explore": return {"status": "already running"}
    
    await set_mode("explore")
    await terminate_process(cleaner_process)

    bridge_node = app.state.ros_node
    ros_command_queue.put({'type': 'cancel_goal'})
    ros_command_queue.put({'type': 'cmd_vel', 'linear': 0.0, 'angular': 0.0})

    try:
        reset_report = await asyncio.to_thread(bridge_node.prepare_fresh_exploration)
    except Exception as exc:
        raise HTTPException(status_code=500, detail={"status": "reset_error", "error": str(exc)}) from exc

    failed_steps = [
        step
        for step in ("slam_reset", "global_costmap_cleared", "local_costmap_cleared")
        if not reset_report.get(step, False)
    ]
    if failed_steps:
        raise HTTPException(
            status_code=503,
            detail={"status": "reset_failed", "failed_steps": failed_steps, **reset_report},
        )

    latest_map = None
    latest_vacuum_map = None

    asyncio.create_task(run_node("explorer"))
    return {"status": "Explorer started", **reset_report}

@app.post("/start-vacuum")
async def start_vacuum():
    global explorer_process
    if current_mode == "clean": return {"status": "already running"}
    
    await set_mode("clean")
    await terminate_process(explorer_process)
    asyncio.create_task(run_node("cleaner"))
    return {"status": "Vacuum started"}

@app.post("/abort")
async def abort():
    global explorer_process, cleaner_process
    ros_command_queue.put({'type': 'cancel_goal'})
    ros_command_queue.put({'type': 'cmd_vel', 'linear': 0.0, 'angular': 0.0})

    await set_mode("idle")
    await terminate_process(explorer_process)
    await terminate_process(cleaner_process)
    return {"status": "idle"}

@app.post("/save-map")
async def save_map():
    bridge_node = app.state.ros_node
    report = await asyncio.to_thread(bridge_node.save_persistent_map_and_posegraph)

    if not report.get("map_saved", False) or not report.get("posegraph_saved", False):
        raise HTTPException(status_code=503, detail=report)

    return report

@app.post("/move")
async def move_robot(data: MoveCommand):
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.pose.position.x = data.x
    goal_msg.pose.pose.position.y = data.y
    goal_msg.pose.pose.orientation.x = data.qx
    goal_msg.pose.pose.orientation.y = data.qy
    goal_msg.pose.pose.orientation.z = data.qz
    goal_msg.pose.pose.orientation.w = data.qw

    bridge_node = app.state.ros_node
    try:
        await asyncio.to_thread(bridge_node.nav_client.wait_for_server, 2.0)
        bridge_node.nav_client.send_goal_async(goal_msg)
        return {"status": "moving", "target": {"x": data.x, "y": data.y}}
    except Exception:
        return {"error": "Action server not available"}

@app.post("/manual_control")
async def manual_control(data: ManualControlCommand):
    ros_command_queue.put({'type': 'cmd_vel', 'linear': data.linear, 'angular': data.angular})
    return {"status": "sent", "linear": data.linear, "angular": data.angular}

@app.post("/api/cleaning")
async def set_cleaning(data: CleaningCommand):
    ros_command_queue.put({'type': 'cleaning', 'vacuum': data.vacuum, 'brush': data.brush})
    app.state.cleaning_state = {"vacuum": data.vacuum, "brush": data.brush}
    return {"status": "sent", "vacuum": data.vacuum, "brush": data.brush}

@app.get("/explorer_status")
async def explorer_status():
    if explorer_process is None:
        return {"status": "idle"}
    if explorer_process.returncode is None:
        return {"status": "running"}
    return {
        "status": (
            "finished" if explorer_process.returncode == 0
            else f"ERROR: {explorer_process.returncode}"
        )
    }

@app.get("/vacuum_status")
async def vacuum_status():
    if cleaner_process is None:
        return {"status": "idle"}
    if cleaner_process.returncode is None:
        return {"status": "running"}
    return {
        "status": (
            "finished" if cleaner_process.returncode == 0
            else f"ERROR: {cleaner_process.returncode}"
        )
    }


@app.post("/coverage_polygon")
async def coverage_polygon(data: dict):
    points = data.get("points", [])
    coverage = compute_polygon_coverage(points)
    return {
        "status": "ok",
        **coverage,
    }

@app.post("/cleaning_polygon")
async def set_cleaning_polygon(data: dict):
    global latest_cleaning_polygon

    points = data.get("points", [])
    if not isinstance(points, list) or len(points) < 3:
        raise HTTPException(status_code=400, detail="Need at least 3 polygon points")

    ros_polygon = PolygonStamped()
    ros_polygon.header.frame_id = "map"
    ros_polygon.header.stamp = app.state.ros_node.get_clock().now().to_msg()

    converted_points = []
    for point in points:
        try:
            px = float(point["x"])
            py = float(point["y"])
        except (TypeError, ValueError, KeyError):
            raise HTTPException(status_code=400, detail="Invalid point format. Expected {'x': number, 'y': number}")

        point_msg = Point32()
        point_msg.x = px
        point_msg.y = py
        point_msg.z = 0.0
        ros_polygon.polygon.points.append(point_msg)
        converted_points.append({"x": px, "y": py})

    app.state.ros_node.cleaning_polygon_pub.publish(ros_polygon)
    latest_cleaning_polygon = ros_polygon

    schedule_broadcast({
        "type": "cleaning_polygon_update",
        "data": {
            "frame_id": ros_polygon.header.frame_id,
            "points": converted_points
        }
    })

    return {
        "status": "sent",
        "polygon": {
            "frame_id": ros_polygon.header.frame_id,
            "points": converted_points
        }
    }


def handle_keepout_stroke(data: dict):
    if latest_map is None:
        return
        
    points = data.get("points", [])
    if not points: 
        return
    
    radius = data.get("radius", DEFAULT_STROKE_RADIUS_M)
    mode = data.get("mode", "pen")
    fill_value = KEEPOUT_FILL if mode == "pen" else 0
    
    map_dir = MAP_STORAGE_DIR
    basename = MAP_STORAGE_BASENAME
    barriers_file = os.path.join(map_dir, f"{basename}_keepout.npz")
    
    res = float(latest_map.info.resolution)
    ox = float(latest_map.info.origin.position.x)
    oy = float(latest_map.info.origin.position.y)
    w = int(latest_map.info.width)
    h = int(latest_map.info.height)
    
    # load or init np array
    if os.path.exists(barriers_file):
        try:
            npz = np.load(barriers_file)
            grid = npz['data']
            if grid.shape != (h, w):
                grid = np.zeros((h, w), dtype=KEEPOUT_DTYPE)
        except Exception:
            grid = np.zeros((h, w), dtype=KEEPOUT_DTYPE)
    else:
        grid = np.zeros((h, w), dtype=KEEPOUT_DTYPE)
        
    def world_to_grid(wx, wy):
        gx = (float(wx) - ox) / res
        gy = (float(wy) - oy) / res
        return int(max(0, min(gx, w - 1))), int(max(0, min(gy, h - 1)))
        
    grid_points = [world_to_grid(p["x"], p["y"]) for p in points]
    r_pixels = max(1, int(radius / res)) if res > 0 else 1
    
    for i in range(len(grid_points)):
        p1c, p1r = grid_points[i]
        
        rr, cc = sk_disk((p1r, p1c), r_pixels, shape=(h, w))
        if len(rr) > 0:
            grid[rr, cc] = fill_value
        
        if i > 0:
            p0c, p0r = grid_points[i-1]
            dist = max(abs(p1c - p0c), abs(p1r - p0r))
            if dist > 1:
                for step in range(1, dist):
                    t = step / float(dist)
                    ic = int(round(p0c + t * (p1c - p0c)))
                    ir = int(round(p0r + t * (p1r - p0r)))
                    rr, cc = sk_disk((ir, ic), r_pixels, shape=(h, w))
                    if len(rr) > 0:
                        grid[rr, cc] = fill_value
                    
    np.savez_compressed(barriers_file, data=grid)

@app.post("/auto-outline-polygon")
async def auto_outline_polygon():
    global latest_map

    if latest_map is None:
        raise HTTPException(status_code=400, detail="No map data available yet")

    width = int(latest_map.info.width)
    height = int(latest_map.info.height)
    grid = np.array(latest_map.data, dtype=np.int16).reshape((height, width))

    result = extract_largest_wall_outline_polygon(
        grid=grid,
        resolution=float(latest_map.info.resolution),
        origin_x=float(latest_map.info.origin.position.x),
        origin_y=float(latest_map.info.origin.position.y),
        occupied_threshold=50,
    )

    if not result.get('success'):
        raise HTTPException(status_code=422, detail=result.get('error', 'Failed to extract outline polygon'))

    return await set_cleaning_polygon({'points': result['points']})

@app.get("/api/sensor/{type}/{pin}")
async def get_sensor(type: str, pin: int):
    if type not in ['d', 'a']:
        return {"error": "Invalid type. Use 'd' or 'a'"}

    req_str = f"{type},{pin}"
    ros_command_queue.put({'type': 'sensor_req', 'req_str': req_str})

    key = f"S{type.upper()}-{pin}"
    if key not in sensor_events:
        sensor_events[key] = asyncio.Event()
    
    sensor_events[key].clear()
    
    try:
        await asyncio.wait_for(sensor_events[key].wait(), timeout=2.0)
        return {
            "pin": pin,
            "type": type,
            "key": key,
            "value": int(sensor_values[key]),
            "timestamp": time.time()
        }
    except asyncio.TimeoutError:
        return {"error": "Timeout waiting for sensor data", "key": key}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.add(websocket)
    try:
        node = app.state.ros_node
        mode = "idle"
        if explorer_process is not None:
            mode = "explore"
        elif cleaner_process is not None:
            mode = "clean"

        cleaning_state = app.state.cleaning_state

        initial_state = {
            "type": "initial_state",
            "data": {
                "mode": current_mode,
                "pose": latest_pose,
                "vacuum": cleaning_state.get("vacuum", 0),
                "brush": cleaning_state.get("brush", 0)
            }
        }
        await websocket.send_json(initial_state)

        if latest_map is not None:
            await websocket.send_bytes(node.process_map(latest_map, "exploration"))
        if latest_vacuum_map is not None:
            await websocket.send_bytes(node.process_map(latest_vacuum_map, "vacuum"))
        if latest_keepout_map is not None:
            await websocket.send_bytes(node.process_map(latest_keepout_map, "keepout"))
        if latest_cleaning_polygon is not None:
            await websocket.send_json({
                "type": "cleaning_polygon_update",
                "data": serialize_cleaning_polygon(latest_cleaning_polygon)
            })

        while True:
            data = await websocket.receive_json()
            action = data.get("action")
            
            if action == "manual_control":
                await manual_control(ManualControlCommand(**data))
            elif action == "start-explore":
                await start_explorer()
            elif action == "start-vacuum":
                await start_vacuum()
            elif action == "abort":
                await abort()
            elif action == "move":
                await move_robot(MoveCommand(**data))
            elif action == "cleaning":
                await set_cleaning(CleaningCommand(**data))
            elif action == "save-map":
                try:
                    await save_map()
                    await websocket.send_json({ "type": "save_map_status", "success": True })
                except Exception as e:
                    detail = getattr(e, "detail", str(e))
                    await websocket.send_json({ "type": "save_map_status", "success": False, "error": detail })
            elif action == "auto-outline-polygon":
                try:
                    res = await auto_outline_polygon()
                    await websocket.send_json({ "type": "auto_outline_status", "success": True, "polygon": res.get("polygon") })
                except Exception as e:
                    detail = getattr(e, "detail", str(e))
                    await websocket.send_json({ "type": "auto_outline_status", "success": False, "error": detail })
            elif action == "cleaning_polygon":
                try:
                    await set_cleaning_polygon(data)
                except Exception:
                    pass
            elif action == "keepout_stroke":
                try:
                    await asyncio.to_thread(handle_keepout_stroke, data)
                except Exception as e:
                    print(f"Error handling keepout stroke: {e}")
                    pass
            elif action == "coverage_polygon":
                try:
                    res = await coverage_polygon(data)
                    await websocket.send_json({"type": "coverage_update", "success": True, **res, "room": data.get("room")})
                except Exception:
                    await websocket.send_json({"type": "coverage_update", "success": False, "room": data.get("room")})

    except (WebSocketDisconnect, Exception):
        connected_clients.discard(websocket)

def ros_spin():
    rclpy.init()
    node = ROS2Bridge()
    app.state.ros_node = node
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main():
    threading.Thread(
        target=ros_spin,
        daemon=True,
    ).start()
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()
