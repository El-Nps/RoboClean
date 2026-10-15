"""
Room Segmentation Module for RoboClean Frontend
Processes occupancy grids and returns segmented room data
"""

import numpy as np
import base64
from io import BytesIO
from skimage.morphology import disk, binary_dilation
from skimage.measure import find_contours, label
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

def extract_largest_wall_outline_polygon(grid, resolution, origin_x, origin_y, occupied_threshold=50):
    """
    Phase 1 MVP: create one polygon by tracing the largest contour on a raw wall mask.
    No preprocessing and no postprocessing/simplification are applied.
    """
    if grid is None:
        return {'error': 'No grid provided'}

    if grid.ndim != 2:
        return {'error': 'Grid must be 2D'}

    height, width = grid.shape
    if height <= 1 or width <= 1:
        return {'error': 'Grid too small'}

    wall_mask = (grid >= occupied_threshold).astype(np.uint8)
    if not np.any(wall_mask):
        return {'error': 'No occupied cells found in map'}

    contours = find_contours(wall_mask, 0.5)
    if not contours:
        return {'error': 'No contour found in wall mask'}

    largest = max(contours, key=lambda contour: len(contour))
    if len(largest) < 3:
        return {'error': 'Largest contour has fewer than 3 points'}

    points = []
    for py, px in largest:
        world = _pixel_to_world(px, py, resolution, origin_x, origin_y, height)
        x = world['x']
        y = world['y']
        if not (np.isfinite(x) and np.isfinite(y)):
            return {'error': 'Computed non-finite polygon point'}
        points.append(world)

    if len(points) < 3:
        return {'error': 'Polygon has fewer than 3 points'}

    return {
        'success': True,
        'points': points,
        'point_count': len(points),
    }

def _pixel_to_world(px, py, resolution, origin_x, origin_y, height):
    return {
        'x': float(px * resolution + origin_x),
        'y': float((py + 1.0) * resolution + origin_y),
    }
