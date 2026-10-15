import numpy as np
import shapely
from shapely.geometry import LineString, Polygon, MultiLineString
from shapely.affinity import rotate
from math import cos, sin, atan2, sqrt, ceil
import math
from shapely.geometry import LineString, Polygon
from math import *
if __name__ == "__main__":
	import matplotlib.pyplot as plt
	import matplotlib.patches

def sweep(poly: Polygon, robot_width, bStartTop=False):
    """
	Returns a list of points to follow for a coverage path through a given Polygon. To calculate the sweep patern the Polygon is first rotated to align the longest edge with the horizontal axis. After calculating the path it is rotated back to the original orientation.
	"""
    if not check_if_Robot_fits(poly, robot_width):
        return [[poly.centroid.x, poly.centroid.y]]
    angle_rad = get_angle_of_longest_side_to_horizontal(poly)
    # rotate the polygon so that the longest edge is horizontal
    rot_angle = -math.degrees(angle_rad) + 90 
    # use shapely's rotate function to rotate the polygon around its centroid (more stable than rotation matrix)
    poly_rot = rotate(poly, rot_angle, origin='centroid')
    path_rot_coords = calc_path(poly_rot, robot_width, bStartTop)
    
    if not path_rot_coords:
        return [[poly.centroid.x, poly.centroid.y]]

    # rotate back points to original orientation
    path_ls = LineString(path_rot_coords)
    path_final = rotate(path_ls, -rot_angle, origin='centroid')
    
    return [list(p) for p in path_final.coords]

def calc_path(poly: Polygon, robot_width, bStartTop=False):
	"""
	Takes in a monotone shapely Polygon without obstacles and calculates a coverage path with given width through it.
	poly = Polygon(((0,0), (10,0), (10,5), (0,5), (0,0))) 
	"""
	ret = []
	up = False
	pos_next = None

	# Decrease width to integer number of runs
	poly_width = poly.bounds[2]-poly.bounds[0]
	poly_height = poly.bounds[3]-poly.bounds[1]

	if poly_width < robot_width and poly_height < robot_width:
		return ret # Don't bother

	# Reduce size of polygon by width
	poly_inner = poly.buffer(-robot_width/2.0)
	if robot_width >= poly_width or robot_width >= poly_height:
		# if smaller then width try at least one sweep
		xscale = 1-(robot_width/poly_width) if robot_width < poly_width else 0.0
		yscale = 1-(robot_width/poly_height) if robot_width < poly_height else 0.0
		poly_inner = shapely.affinity.scale(poly, xscale, yscale)

	plows = ceil(poly_width/robot_width)
	plow_width = poly_width/plows

	if poly_inner.is_empty:
		# No polygon left
		return ret

	for x in np.arange(poly_inner.bounds[0], poly_inner.bounds[2]+plow_width, plow_width):
		# Correct rounding errors
		if x > poly_inner.bounds[2]: x = poly_inner.bounds[2]

		# Get both intersections of current x with y
		intersections_y = None
		if poly_inner.bounds[0] == poly_inner.bounds[2]:
			# more a line then polygon
			intersections_y = [poly_inner.bounds[1], poly_inner.bounds[3]]
		else:
			try:
				vertline = LineString([(x, poly_inner.bounds[1]), (x, poly_inner.bounds[3])])
				intersections = poly_inner.intersection(vertline)
				if (type(intersections) != type(vertline)): continue
				intersections_y = [l[1] for l in intersections.coords[:]]
			except shapely.errors.TopologicalError:
				continue
		intersections_y.sort()
		if bStartTop: intersections_y.reverse()
		if len(intersections_y) < 2:
			# Only one intersection
			y1 = intersections_y[0]
			ret.append([x, y1])
		else:
			(y1, y2) = intersections_y 

			for i in range(2): # One to follow the current plow, one to drive to the next plow
				if i == 1:
					up = not up
				if up:
					pos_next = [x, y2]
				else:
					pos_next = [x, y1]

				pos_cur = pos_next
				if not pos_cur in ret:
					ret.append(pos_cur)

	return ret

def get_angle_of_longest_side_to_horizontal(poly):
    rect = poly.minimum_rotated_rectangle
    if not hasattr(rect, "exterior"):
        # is probably a point
        return None
    rect_points = rect.exterior.coords[:]
    p1,p2,p3,p4 = [np.array(p) for p in rect_points[:-1]]
    p12 = p1-p2
    p23 = p2-p3
    longest_side = p12 if np.linalg.norm(p12) > np.linalg.norm(p23) else p23
    return atan2(longest_side[1], longest_side[0])

def check_if_Robot_fits(poly: Polygon, robot_width):
	if poly.bounds[2]-poly.bounds[0] < robot_width or poly.bounds[3]-poly.bounds[1] < robot_width:
		return False
	return True