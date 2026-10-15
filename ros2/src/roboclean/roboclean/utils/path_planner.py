import os
import json
import tempfile
import copy
import numpy as np
from scipy import ndimage
from shapely.geometry import Polygon, Point
from roboclean.utils import sweep

class CoveragePathGenerator:
    def __init__(
        self,
        robot_width=0.3,
        occupied_threshold=70,
        opening_kernel_m=0.15,
        closing_kernel_m=0.15,
        min_obstacle_area_m2=0.03,
    ):
        self.robot_width = robot_width
        self.occupied_threshold = occupied_threshold
        self.opening_kernel_m = opening_kernel_m
        self.closing_kernel_m = closing_kernel_m
        self.min_obstacle_area_m2 = min_obstacle_area_m2
        # Determine path to the Ruby script automatically
        current_dir = os.path.dirname(os.path.realpath(__file__))
        self.ruby_script = os.path.join(current_dir, "boustrophedon_decomposition.rb")

    def _meters_to_kernel_cells(self, kernel_m, resolution):
        if kernel_m is None or kernel_m <= 0.0 or resolution <= 0.0:
            return 1
        cells = int(np.ceil(float(kernel_m) / float(resolution)))
        cells = max(1, cells)
        if cells % 2 == 0:
            cells += 1
        return cells

    def _area_to_cells(self, area_m2, resolution):
        if area_m2 is None or area_m2 <= 0.0 or resolution <= 0.0:
            return 1
        cell_area = float(resolution) * float(resolution)
        return max(1, int(np.ceil(float(area_m2) / cell_area)))
    
    def preprocess_costmap(self, costmap, keepout_mask=None):
        width = costmap.info.width
        height = costmap.info.height
        resolution = float(costmap.info.resolution)

        opening_kernel_size = self._meters_to_kernel_cells(self.opening_kernel_m, resolution)
        closing_kernel_size = self._meters_to_kernel_cells(self.closing_kernel_m, resolution)
        min_obstacle_cells = self._area_to_cells(self.min_obstacle_area_m2, resolution)

        data = np.array(costmap.data, dtype=np.int16).reshape((height, width))
        
        # Apply keepout mask if provided
        if keepout_mask is not None:
            try:
                mask = np.asarray(keepout_mask)
                if mask.shape != data.shape:
                    if mask.size == data.size:
                        mask = mask.reshape(data.shape)
                    else:
                        raise ValueError("keepout_mask has incompatible shape")
                data[mask != 0] = 100
            except Exception as e:
                print(f"[preprocess_costmap] Failed to apply keepout_mask: {e}")

        unknown_mask = data == -1
        known_mask = ~unknown_mask

        occupied_mask = (data >= self.occupied_threshold) & known_mask

        if opening_kernel_size > 1:
            opening_structure = np.ones((opening_kernel_size, opening_kernel_size), dtype=bool)
            occupied_mask = ndimage.binary_opening(occupied_mask, structure=opening_structure)

        if closing_kernel_size > 1:
            closing_structure = np.ones((closing_kernel_size, closing_kernel_size), dtype=bool)
            occupied_mask = ndimage.binary_closing(occupied_mask, structure=closing_structure)

        occupied_mask &= known_mask

        if min_obstacle_cells > 1:
            labeled, num_features = ndimage.label(occupied_mask)
            if num_features > 0:
                component_sizes = np.bincount(labeled.ravel())
                small_components = component_sizes < min_obstacle_cells
                small_components[0] = False
                occupied_mask[small_components[labeled]] = False

        processed = np.full((height, width), -1, dtype=np.int8)
        processed[known_mask] = 0
        processed[occupied_mask] = 100

        changed_cells = int(np.count_nonzero(processed != data))
        print(
            f"[generate_path_new] Preprocessed map: changed {changed_cells} cells "
            f"(occ_thresh={self.occupied_threshold}, open={opening_kernel_size} cells/{self.opening_kernel_m:.3f}m, "
            f"close={closing_kernel_size} cells/{self.closing_kernel_m:.3f}m, "
            f"min_blob={min_obstacle_cells} cells/{self.min_obstacle_area_m2:.4f}m²)"
        )

        cleaned_costmap = copy.deepcopy(costmap)
        cleaned_costmap.data = processed.flatten().tolist()
        return cleaned_costmap
    
    def do_boustrophedon(self, costmap, cleaning_polygon=None):
        width = costmap.info.width
        height = costmap.info.height
        resolution = costmap.info.resolution
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y

        polygon = None
        min_ix = 0
        max_ix = width
        min_iy = 0
        max_iy = height

        if cleaning_polygon is not None:
            try:
                polygon = cleaning_polygon if isinstance(cleaning_polygon, Polygon) else Polygon(cleaning_polygon)
                if not polygon.is_valid:
                    polygon = polygon.buffer(0)
                if polygon.is_empty:
                    polygon = None
            except Exception as e:
                print(f"[do_boustrophedon] Invalid cleaning polygon, fallback to full map: {e}")
                polygon = None

        if polygon is not None:
            (min_x, min_y, max_x, max_y) = polygon.bounds
            min_ix = max(0, int(np.floor((min_x - origin_x) / resolution)))
            max_ix = min(width, int(np.ceil((max_x - origin_x) / resolution)))
            min_iy = max(0, int(np.floor((min_y - origin_y) / resolution)))
            max_iy = min(height, int(np.ceil((max_y - origin_y) / resolution)))

            if min_ix >= max_ix or min_iy >= max_iy:
                print("[do_boustrophedon] Cleaning polygon outside map bounds")
                return []

            print("Converting bounded costmap at x=%d..%d, y=%d..%d for Boustrophedon Decomposition" % (min_ix, max_ix, min_iy, max_iy))
        else:
            print("Converting full costmap at x=%d..%d, y=%d..%d for Boustrophedon Decomposition" % (min_ix, max_ix, min_iy, max_iy))

        # 1. Schritt: Costmap-Daten für das Skript aufbereiten
        rows = []
        for ix in range(min_ix, max_ix):
            column = []
            for iy in range(min_iy, max_iy):
                x = (ix + 0.5) * resolution + origin_x
                y = (iy + 0.5) * resolution + origin_y
                data = costmap.data[int(iy*costmap.info.width+ix)]
                if data == -1:
                    # Unknown or not inside polygon: Treat as obstacle
                    column.append(0)
                elif polygon is not None and not polygon.covers(Point(x, y)):
                    column.append(0)
                elif data == 0:
                     # Freespace (non-lethal)
                    column.append(-1)
                else:
                     # Obstacle
                    column.append(0)
            rows.append(column)

        # ... (Aufruf des Ruby-Skripts bleibt gleich)
        polygons = [] 
        # [Hier kommt dein os.popen Code hin, der 'polygons' füllt]
        with tempfile.NamedTemporaryFile(delete=False,mode='w') as ftmp:
            ftmp.write(json.dumps(rows))
            ftmp.flush()
            boustrophedon_script = self.ruby_script
            print(f"Data written to temporary file: {ftmp.name}")
            with os.popen("%s %s" % (boustrophedon_script, ftmp.name)) as fscript:
                output = fscript.read()
                lines = output.strip().split('\n')
                if lines:
                    json_data = lines[-1] # Take the very last line
                    try:
                        polygons = json.loads(json_data)
                    except json.JSONDecodeError:
                        print("Failed to parse the last line as JSON.")
        # 2. Schritt: Ergebnisse vom Skript zurück in Weltkoordinaten wandeln
        polygons_ret = []
        for poly in polygons:
            points = []
            for point in poly:
                # Sicherheits-Check (Clamping)
                # Verhindert, dass Punkte exakt auf der Kante (z.B. 199.0) landen
                grid_x = max(0.0, min(float(point[0]) + float(min_ix), float(width - 1)))
                grid_y = max(0.0, min(float(point[1]) + float(min_iy), float(height - 1)))

                # Umrechnung: Grid-Index -> Welt-Koordinate
                # Wir addieren 0.5, um in der MITTE der Zelle zu landen (stabiler für Nav2)
                world_x = (grid_x + 0.5) * resolution + origin_x
                world_y = (grid_y + 0.5) * resolution + origin_y
                
                points.append((world_x, world_y))
            
            if points:
                polygons_ret.append(Polygon(points))
                print(f"Creating polygon from Boustrophedon Decomposition {str(points)}")
        return polygons_ret

    def generate_path(self, costmap, cleaning_polygon=None, keepout_mask=None):
        preprocessed_costmap = self.preprocess_costmap(costmap, keepout_mask=keepout_mask)
        polygons = self.do_boustrophedon(preprocessed_costmap, cleaning_polygon=cleaning_polygon)
        full_path = []
        for idx, poly in enumerate(polygons):
            try:
                sweep_path = sweep.sweep(poly, self.robot_width)
                
                if not sweep_path:
                    print(f"[generate_path] Polygon {idx}: sweep returned empty path")
                    continue
                
                print(f"[generate_path] Polygon {idx}: generated {len(sweep_path)} waypoints")
                
                # Step 4: Connect paths intelligently
                if full_path and sweep_path:
                    # Check both orderings and pick the one with shorter transition
                    last_point = full_path[-1]
                    path_start = sweep_path[0]
                    path_end = sweep_path[-1]
                    
                    dist_normal = np.linalg.norm(np.array(last_point) - np.array(path_start))
                    dist_reversed = np.linalg.norm(np.array(last_point) - np.array(path_end))
                    
                    if dist_reversed < dist_normal:
                        sweep_path = sweep_path[::-1]
                        print(f"[generate_path] Polygon {idx}: reversed path (transition: {dist_reversed:.2f}m)")
                    else:
                        print(f"[generate_path] Polygon {idx}: normal path (transition: {dist_normal:.2f}m)")
                
                full_path.extend(sweep_path)
                
            except Exception as e:
                print(f"[generate_path] Error sweeping polygon {idx}: {e}")
                continue

        full_path = self._sanitize_path_for_costmap(full_path, preprocessed_costmap)
        
        print(f"[generate_path] Generated complete path with {len(full_path)} waypoints")
        return full_path, polygons

    def _sanitize_path_for_costmap(self, path, costmap):
        if not path:
            return path

        resolution = costmap.info.resolution
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        width = costmap.info.width
        height = costmap.info.height

        min_x = origin_x
        min_y = origin_y
        max_x = origin_x + (width * resolution)
        max_y = origin_y + (height * resolution)
        eps = max(resolution * 1e-3, 1e-6)

        sanitized = []
        clipped_count = 0

        for point in path:
            if point is None or len(point) < 2:
                continue

            wx = float(point[0])
            wy = float(point[1])

            wx_clamped = min(max(wx, min_x + eps), max_x - eps)
            wy_clamped = min(max(wy, min_y + eps), max_y - eps)

            if wx != wx_clamped or wy != wy_clamped:
                clipped_count += 1

            sanitized.append([wx_clamped, wy_clamped])

        if clipped_count > 0:
            print(f"[generate_path] Clipped {clipped_count} waypoints to map bounds")

        return sanitized
