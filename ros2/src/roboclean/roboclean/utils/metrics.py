import json
import time
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from dataclasses import dataclass


@dataclass
class MetricsSnapshot:
    """Single metrics snapshot"""
    timestamp: float
    elapsed_time: float
    num_frontiers: int = 0
    map_coverage: float = 0.0
    total_distance: float = 0.0
    goals_sent: int = 0
    goals_completed: int = 0
    
    def to_dict(self):
        return {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.timestamp)),
            'elapsed_time_sec': round(self.elapsed_time, 2),
            'num_frontiers': self.num_frontiers,
            'map_coverage_percent': round(self.map_coverage, 2),
            'total_distance_m': round(self.total_distance, 3),
            'goals_sent': self.goals_sent,
            'goals_completed': self.goals_completed,
        }


class ExplorationMetricsTracker:
    """Tracks exploration metrics"""
    
    def __init__(self, logger=None):
        self.start_time = time.time()
        self.history = []
        self.last_position = None
        self.total_distance = 0.0
        self.goals_sent = 0
        self.goals_completed = 0
        self.last_num_frontiers = 0
        self.logger = logger  # Optional logger from ROS node
    
    def _log(self, level, msg):
        """Helper to log with or without ROS logger"""
        if self.logger:
            if level == 'info':
                self.logger.info(msg)
            elif level == 'debug':
                self.logger.debug(msg)
        else:
            # Fallback to print
            print(f"[{level.upper()}] {msg}")
    
    def record_position(self, msg: Odometry):
        """Update position and calculate distance"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Initialize on first call
        if self.last_position is None:
            self.last_position = (x, y)
            self._log('info', f"Initial position: ({x:.2f}, {y:.2f})")
            return
        
        dx = x - self.last_position[0]
        dy = y - self.last_position[1]
        distance = np.hypot(dx, dy)
        
        # Track ANY movement (threshold: 1cm)
        if distance > 0.01:
            self.total_distance += distance
            self._log(
                'debug',
                f"Movement detected: +{distance:.3f}m "
                f"(from {self.last_position} to ({x:.2f}, {y:.2f}), "
                f"total: {self.total_distance:.2f}m)"
            )
        
        self.last_position = (x, y)
    
    def record_goal_sent(self):
        """Called when a new goal is sent"""
        self.goals_sent += 1
    
    def record_goal_completed(self):
        """Called when a goal is completed"""
        self.goals_completed += 1
    
    def calculate_map_coverage(self, map_data: OccupancyGrid) -> float:
        """Calculate percentage of map explored"""
        if map_data is None:
            return 0.0
        
        data = np.array(map_data.data, dtype=np.int8)
        explored = np.sum(data >= 0)
        total = len(data)
        
        return (explored / total * 100.0) if total > 0 else 0.0
    
    def count_frontiers(self, num_frontiers: int):
        """Update frontier count"""
        self.last_num_frontiers = num_frontiers
    
    def get_snapshot(self, map_data: OccupancyGrid) -> MetricsSnapshot:
        """Get current metrics snapshot"""
        elapsed = time.time() - self.start_time
        
        snapshot = MetricsSnapshot(
            timestamp=time.time(),
            elapsed_time=elapsed,
            num_frontiers=self.last_num_frontiers,
            map_coverage=self.calculate_map_coverage(map_data),
            total_distance=self.total_distance,
            goals_sent=self.goals_sent,
            goals_completed=self.goals_completed,
        )
        
        self.history.append(snapshot)
        return snapshot
    
    def save_metrics(self, filename: str):
        """Save metrics history to JSON"""
        data = {
            'total_duration_sec': time.time() - self.start_time,
            'final_metrics': {
                'total_distance_m': round(self.total_distance, 3),
                'goals_sent': self.goals_sent,
                'goals_completed': self.goals_completed,
            },
            'snapshots': [s.to_dict() for s in self.history]
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
    
    def print_summary(self):
        """Print exploration summary"""
        if not self.history:
            return
        
        final = self.history[-1]
        print("\n" + "━"*40)
        print("EXPLORATION BENCHMARK SUMMARY")
        print("━"*40)
        print(f"Total Duration:       {final.elapsed_time:.1f} seconds")
        print(f"Total Distance:       {final.total_distance:.2f} meters")
        print(f"Map Coverage:         {final.map_coverage:.1f}%")
        print(f"Frontier Points:      {final.num_frontiers}")
        print(f"Goals Sent:           {final.goals_sent}")
        print(f"Goals Completed:      {final.goals_completed}")
        goals_sent = final.goals_sent
        if goals_sent > 0:
            success_rate = (final.goals_completed / goals_sent) * 100
        else:
            success_rate = 0
        print(f"Success Rate:         {success_rate:.1f}%")
        print("━"*40 + "\n")
