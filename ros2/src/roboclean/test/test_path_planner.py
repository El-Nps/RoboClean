#!/usr/bin/env python3
"""
Test and visualize generate_path_new on a complete map
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon as MplPolygon
from matplotlib.colors import ListedColormap
from path_planer import CoveragePathGenerator
from nav_msgs.msg import OccupancyGrid


def visualize_path(occupancy_data, width, height, resolution, origin_x, origin_y, path, polygons=None, title="Coverage Path"):
    """
    Visualize the occupancy grid and the generated path.
    
    Args:
        occupancy_data: 1D list of occupancy values
        width, height: Grid dimensions
        resolution: Cell size in meters
        origin_x, origin_y: Map origin in world coordinates
        path: List of [x, y] waypoints
        title: Plot title
    """
    # Convert 1D data to 2D grid for visualization
    grid = np.array(occupancy_data).reshape((height, width))
    
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # Create custom colormap: unknown (white), free (light gray), occupied (dark gray)
    colors = ['white', 'lightgray', 'black']
    bounds = [-1.5, -0.5, 50.5, 100.5]
    norm = plt.matplotlib.colors.BoundaryNorm(bounds, len(colors))
    cmap = ListedColormap(colors)
    
    # Display grid (row-major: grid[y, x] → rows=y-axis, cols=x-axis, no transpose needed)
    extent = [origin_x, origin_x + width * resolution,
              origin_y, origin_y + height * resolution]
    im = ax.imshow(grid, extent=extent, origin='lower', cmap=cmap, aspect='auto')
    
    # Draw boustrophedon polygons
    if polygons:
        poly_colors = plt.cm.tab10(np.linspace(0, 1, max(len(polygons), 1)))
        for i, poly in enumerate(polygons):
            coords = np.array(poly.exterior.coords)
            patch = MplPolygon(coords, closed=True,
                               facecolor=poly_colors[i % len(poly_colors)],
                               edgecolor='blue', linewidth=1.5,
                               alpha=0.25, zorder=2,
                               label=f'Polygon {i}' if i < 10 else None)
            ax.add_patch(patch)

    # Plot path
    if path and len(path) > 0:
        path_array = np.array(path)
        ax.plot(path_array[:, 0], path_array[:, 1], 'r-', linewidth=2, label='Coverage Path', alpha=0.8)
        ax.plot(path_array[:, 0], path_array[:, 1], 'bo', markersize=3, alpha=0.6)
        
        # Mark start and end
        ax.plot(path_array[0, 0], path_array[0, 1], 'go', markersize=10, label='Start', zorder=5)
        ax.plot(path_array[-1, 0], path_array[-1, 1], 'rs', markersize=10, label='End', zorder=5)
        
        # Add path statistics
        path_length = 0
        for i in range(len(path) - 1):
            dist = np.linalg.norm(np.array(path[i+1]) - np.array(path[i]))
            path_length += dist
        
        stats_text = f"Path Length: {path_length:.2f}m\nWaypoints: {len(path)}"
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                fontsize=10)
    
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


def set_obstacle_rect(occupancy_grid, x_min, y_min, x_max, y_max, value=100):
    """Set a rectangular obstacle in OccupancyGrid using grid coordinates."""
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height

    x0 = max(0, min(width, x_min))
    x1 = max(0, min(width, x_max))
    y0 = max(0, min(height, y_min))
    y1 = max(0, min(height, y_max))

    for y in range(y0, y1):
        row_offset = y * width
        for x in range(x0, x1):
            occupancy_grid.data[row_offset + x] = value


def test_generate_path_new(pattern='simple_rectangle', robot_width=0.3):
    """
    Test the generate_path_new function with a given pattern.
    
    Args:
        pattern: Type of test grid ('simple_rectangle', 'simple_L', 'rooms', 'corridor')
        robot_width: Robot width in meters
    """
    print(f"\n{'='*60}")
    print(f"Testing generate_path_new with pattern: {pattern}")
    print(f"Robot width: {robot_width}m")
    print(f"{'='*60}")
    
    # Create test grid (returns MockOccupancyGrid object)
    map = OccupancyGrid()
    map.info.width = 50
    map.info.height = 50
    map.info.resolution = 0.1
    map.data = [0] * (map.info.width * map.info.height)
    # Place two rectangular obstacles (x/y in grid cells).
    set_obstacle_rect(map, x_min=10, y_min=10, x_max=20, y_max=13, value=100)
    set_obstacle_rect(map, x_min=30, y_min=25, x_max=40, y_max=28, value=100)
    map.info.origin.position.x = 0
    map.info.origin.position.y = 0

    occupancy_grid = map
    
    # Initialize path generator
    generator = CoveragePathGenerator(robot_width=robot_width)
    
    # Generate path
    path, polygons = generator.generate_path_new(occupancy_grid)
    
    # Visualize
    fig = visualize_path(
        occupancy_grid.data,
        occupancy_grid.info.width,
        occupancy_grid.info.height,
        occupancy_grid.info.resolution,
        occupancy_grid.info.origin.position.x,
        occupancy_grid.info.origin.position.y,
        path,
        polygons=polygons,
        title=f"Coverage Path - {pattern} (robot={robot_width}m)"
    )
    
    print(f"Path generation complete!")
    print(f"  Generated {len(path)} waypoints")
    if len(path) > 1:
        total_distance = 0
        for i in range(len(path) - 1):
            dist = np.linalg.norm(np.array(path[i+1]) - np.array(path[i]))
            total_distance += dist
        print(f"  Total path length: {total_distance:.2f}m")
    
    plt.show()
    
    return path


def test_all_patterns(robot_width=0.3):
    """Test all available patterns"""
    patterns = ['simple_rectangle', 'simple_L', 'rooms', 'corridor']
    
    for pattern in patterns:
        test_generate_path_new(pattern=pattern, robot_width=robot_width)


if __name__ == "__main__":
    # Test single pattern
    test_generate_path_new(pattern='corridor', robot_width=0.3)
    
    # Uncomment to test all patterns
    # test_all_patterns(robot_width=0.3)
