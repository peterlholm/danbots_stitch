#!/bin/python3
"Stitch a pointcloud to pointcloud"
import sys
from time import perf_counter
from pathlib import Path
#from random import random
import argparse
import open3d as o3d
import numpy as np
from stitching.stitch import rstitch
from stitching.error_calc import cmp2pcl


_DEBUG = False
_SHOW = True
_VERBOSE = False


def surface_to_pcl(mesh, alg="poisson", number_of_points=100000, init_factor=10):
    "convert mesh surfaces to pointcloud, point_factor vertices/points"
    if _DEBUG:
        mesh_info(mesh)
        print(f"Algoritm: {alg} Number_of_points: {number_of_points} Point_factor: {init_factor}")
    if alg=='poisson':
        pcl = mesh.sample_points_poisson_disk(number_of_points=number_of_points, )
    else:
        pcl = mesh.sample_points_uniformly(number_of_points=number_of_points, init_factor=init_factor)
    return pcl

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

def add_pcl(pcl1, pcl2):
    "concatenate pointclouds with colors"
    p1 = np.asarray(pcl1.points)
    p2 = np.asarray(pcl2.points)
    p3 = np.concatenate((p1, p2), axis=0)
    pcl3 = o3d.geometry.PointCloud()
    pcl3.points = o3d.utility.Vector3dVector(p3)
    if pcl1.has_colors() and pcl2.has_colors():
        p1_c = np.asarray(pcl1.colors)
        p2_c = np.asarray(pcl2.colors)
        p3_c = np.concatenate((p1_c, p2_c), axis=0)
        pcl3.colors = o3d.utility.Vector3dVector(p3_c)
    return pcl3

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog='stitch', description='Stitch a file and a folder')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('-s', required=False, help="Show pointclouds", action='store_true' )
    parser.add_argument('org_file', type=Path, help="The original stl or pointcloud")
    parser.add_argument('test_folder', type=Path, help="The pointcloud to be measured")
    args = parser.parse_args()

    _DEBUG = args.d
    _VERBOSE = args.v
    _SHOW =args.s

    TRANS = [[ 1, 0, 0, 0], [0, 1, 0, 0 ], [0,0,1,0],[0,0,0,1]]

    if _VERBOSE:
        print(f"Stitching {str(args.org_file)} and folder {args.test_folder}")
    # check files exists
    if not args.org_file.exists() or not args.test_folder.exists():
        print("input file(s) does not exist")
        sys.exit(1)
    # check file types

    if args.org_file.suffix=='.stl':
        inmesh = o3d.io.read_triangle_mesh(str(args.org_file))
        obj_size = mesh_size(inmesh)
        if _VERBOSE:
            print(f"Input object size {obj_size:.2f}")
        #print(mesh_info(mesh))
        #mesh = error = cmp_pcl(fil1, fil2)
        in_pcl = surface_to_pcl(inmesh)
        #in2_pcl = surface_to_pcl(inmesh, point_factor=1, number=10000)
        #o3d.io.write_point_cloud('pcloud.ply', in_pcl)
        #o3d.io.write_point_cloud('pcloud2.ply', in2_pcl)
    elif args.org_file.suffix=='.ply':
        in_pcl = o3d.io.read_point_cloud(str(args.org_file))
        if not in_pcl.has_colors():
            if _DEBUG:
                print("putting color on ref")
            in_pcl.paint_uniform_color((1,1,1))
    else:
        print("Input file type error")
        sys.exit(1)
    if args.test_folder.exists():
        # clean new files
        for f in args.test_folder.glob("*new.ply"):
            Path(f).unlink()
        files = sorted(args.test_folder.glob("*.ply"))
        for f in files:
            print("-------------------------------------------")
            print("Stitching:", f)
            print("-------------------------------------------")
            start_time = perf_counter()
            t_pcl = o3d.io.read_point_cloud(str(f))
            transformation = rstitch(in_pcl, t_pcl, verbose=True)
            stop_time = perf_counter()
            print(f"Stitchingtime: {stop_time-start_time:.2f} sec" )
            if transformation is None:
                print(f"-------- Registration of {f} unsuccessfull ---------------")
                continue
            # compare with ref
            new_pcl = t_pcl.transform(transformation)
            rms, min, max, mean = cmp2pcl(in_pcl, new_pcl)
            print(f"RMS error: {rms*1000:.3f} mm")
            col_pcl = add_pcl(in_pcl, new_pcl)
            print(f"New pointcloud with {len(col_pcl.points)} points")
            filename = Path(f).with_suffix('.new.ply')
            in_pcl = col_pcl
            o3d.io.write_point_cloud(str(filename), col_pcl)
            if _DEBUG:
                print("Transformation\n", transformation)
                diff = transformation - TRANS
                #print(diff)
                if diff.max() > 0.005:
                    print("Transformation Error:", diff.max())
                    break
    else:
        print("Input folder does not exist", args.test_folder)
