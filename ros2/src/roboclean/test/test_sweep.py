import matplotlib.pyplot as plt
import sys
import os
import numpy as np
from shapely.affinity import rotate
from shapely.geometry import Polygon
from ros2.src.roboclean.roboclean.utils.sweep import sweep, calc_path

def plot_polygon(ax, poly, color="black"):
    x, y = poly.exterior.xy
    ax.plot(x, y, color=color, linewidth=2)


def plot_path(ax, path, color="tab:blue"):
    if not path:
        return
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    ax.plot(xs, ys, marker="o", markersize=3, color=color)


def main():
    base_poly = Polygon([(0.0, 0.0), (0.5, 0.0), (0.5, 0.5), (0.0, 0.5)])
    poly = rotate(base_poly, 30.0, origin="center", use_radians=False)

    robot_width = 0.2
    path = sweep(poly, robot_width, bStartTop=False)
    path2 = calc_path(base_poly, robot_width, bStartTop=False)
    
    print(path)

    assert len(path) > 0, "use_sweep returned an empty path"

    fig, ax = plt.subplots()
    plot_polygon(ax, poly)
    #plot_path(ax, path)
    plot_path(ax, path, color="tab:orange")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("use_sweep path")
    ax.grid(True, linestyle="--", alpha=0.4)

    plt.show()


if __name__ == "__main__":
    main()
