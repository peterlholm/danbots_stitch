"Stiching module"
from pathlib import Path

from os.path import isfile, abspath
from sys import stderr
import open3d as o3d
import time

from . import util
# import registration as reg
from . import noise_removal as nr
# import meshing as mesh
# import evaluate as ev

# Default values.
voxel_size = 0.3

# Pre-stitching cleaning parameters.
epsilon = 0.35
minimum_points = 7
required_share = 0.06

# Post-stitching cleaning parameters.
neighbours_statistical = 10
neighbours_radius = 20
stdev_ratio = 2

# Mesh correction
low_density_threshold = 0.15

# Mesh detail
poisson_depth = 12

components = []

def read_pointclouds(folder, filename='pointcloud.ply'):
    "return list of pointclouds"
    # components = [[o3d.io.read_point_cloud(folder / str(i) / filename % i) for i in range(20)]
    #                   for cr in data["component_ranges"]]
    component_ranges = [[10, 20], [20, 30], [1, 10]]
    components = [[o3d.io.read_point_cloud(str(Path(folder) / str(i) / filename)) for i in range(*cr)]
                      for cr in component_ranges]
    print ("length",len(components))
    print ("length0", len(components[0]), type(components[0]))
    return components

def run_cleaning():
    print("Cleaning.")
    clean_time_start = time.perf_counter()
    components = [[nr.keep_significant_clusters(
        pcd, required_share, epsilon, minimum_points)[0]
                for pcd in pcds]
                for pcds in components]
    clean_time_stop = time.perf_counter()
    print("Cleaning done in {:.3f} seconds".format(clean_time_stop - clean_time_start))

def stitch_run(folder):
    "stitching a folder tree"
    print("stitching folder:", folder)

    # Start timer, parse config and run pipeline.
    overall_time_start = time.perf_counter()
    print("Loading data.")
    components = read_pointclouds(folder)
    print ("slength",len(components))
    print ("length0", len(components[0]), type(components[0]))

    #print (components)

    # components, component_names, visualize, write, evaluate, \
    #     ground_truth, visualize_mesh_density, remove_low_density, \
    #     use_default_parameters, parameters = util.parse_config()

    # if not use_default_parameters:
    #     voxel_size = parameters["voxel_size"]
    #     epsilon = parameters["epsilon"]
    #     minimum_points = parameters["minimum_points"]
    #     required_share = parameters["required_share"]
    #     neighbours_statistical = parameters["neighbours_statistical"]
    #     neighbours_radius = parameters["neighbours_radius"]
    #     stdev_ratio = parameters["stdev_ratio"]
    #     low_density_threshold = parameters["low_density_threshold"]
    #     poisson_depth = parameters["poisson_depth"]

    run_cleaning()
    print ("clength",len(components))
    print ("length0", len(components[0]), type(components[0]))
