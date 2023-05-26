#!/bin/python3
"Stitch a pointcloud to pointcloud"
import sys
from pathlib import Path
from random import random
import argparse
import open3d as o3d
import numpy as np

_DEBUG = False
_SHOW = True
_VERBOSE = True

def stl2pcl(mesh):
    "create pointcloud from vertices in stl"
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    return pcd

def mesh_size(mesh : o3d.geometry.TriangleMesh):
    "calculate average size of mesh"
    max_size = mesh.get_max_bound()
    min_size = mesh.get_min_bound()
    s = 0
    for i in range(2):
        s += max_size[i] - min_size[i]
    s = s / 3
    return s

def mesh_info(mesh):
    "print interesting info about mesh"
    print("Bounding box",mesh.get_axis_aligned_bounding_box())
    print("Oriented Bounding box",mesh.get_oriented_bounding_box())
    print("Vertices", len(mesh.vertices))

def surface_to_pcl(mesh, alg="poisson", point_factor=10, number=1):
    "convert mesh surfaces to pointcloud, point_factor vertices/points"
    if _DEBUG:
        mesh_info(mesh)
    no_points = len(mesh.vertices)//point_factor * number
    if _DEBUG:
        print("algorithm poisson", alg=="poisson")
    if alg=='poisson':
        pcl = mesh.sample_points_poisson_disk(number_of_points=no_points)
    else:
        pcl = mesh.sample_points_uniformly(number_of_points=no_points)
    #print("Resulting number of points", no_points)
    return pcl


if __name__ == "__main__":
    #PICFOLDER = Path(__file__).parent / 'testdata'
    parser = argparse.ArgumentParser(prog='stitch3d', description='Stitch two 3d files and calculate error figures')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('org_file', help="The original stl or pointcloud")
    parser.add_argument('test_file', help="The pointcloud to be measured")
    args = parser.parse_args()
    if args.d:
        _DEBUG=True
    _VERBOSE = args.v
    fil1 = Path(args.org_file)
    fil2 = Path(args.test_file)
    if _VERBOSE:
        print(f"Stitching {fil1} and {fil2}")
    # check files exists
    if not fil1.exists() or not fil2.exists():
        print("input file(s) does not exist")
        sys.exit(1)
    # check file types

    if fil1.suffix=='.stl':
        inmesh = o3d.io.read_triangle_mesh(str(fil1))
        obj_size = mesh_size(inmesh)
        if _VERBOSE:
            print(f"Input object size {obj_size:.2f}")
        #print(mesh_info(mesh))
        #mesh = error = cmp_pcl(fil1, fil2)
        in_pcl = stl2pcl(inmesh)
        in2_pcl = surface_to_pcl(inmesh, point_factor=1, number=10000)
        o3d.io.write_point_cloud('pcloud.ply', in_pcl)
        o3d.io.write_point_cloud('pcloud2.ply', in2_pcl)
    else:
        print("Input file type error")
        sys.exit(1)

    if fil2.suffix=='.ply':
        t_pcl = o3d.io.read_point_cloud(str(fil2))


