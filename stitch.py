#!/bin/python3
"Compare a pointcloud to mesh"
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

def disturb_pcl(pcl_file, outfile, dist = 2):
    "create pcl with randome error dist"
    pcl = o3d.io.read_point_cloud(str(pcl_file))
    olist = []
    for p in pcl.points:
        newp = (p[0]+ dist*random(), p[1]+ dist*random(), p[2]+ dist*random())
        olist.append(newp)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(olist)
    o3d.io.write_point_cloud(str(outfile), pcd)

def surface_to_pcl(mesh, alg="poisson", point_factor=10):
    "convert mesh surfaces to pointcloud, point_factor vertices/points"
    if _DEBUG:
        mesh_info(mesh)
    no_points = len(mesh.vertices)//point_factor
    if _DEBUG:
        print("algorithm poisson", alg=="poisson")
    if alg=='poisson':
        pcl = mesh.sample_points_poisson_disk(number_of_points=no_points)
    else:
        pcl = mesh.sample_points_uniformly(number_of_points=no_points)
    #print("Resulting number of points", no_points)
    return pcl

def cmp_stl(mesh_file, pcl_file):
    "compare a mesh and a pointcloud and return a value for the error"
    if not mesh_file.exists() or not pcl_file.exists():
        print("File does not exist", mesh_file, pcl_file)
        raise FileNotFoundError("Input not found")
    mesh = o3d.io.read_triangle_mesh(str(mesh_file))
    org = stl2pcl(mesh)
    #org = surface_to_pcl(mesh, point_factor=1)
    pcl = o3d.io.read_point_cloud(str(pcl_file))
    if _DEBUG:
        #mesh_info(mesh)
        print("Points in reference", len(org.points))
        print("Points in pointcloud", len(pcl.points))
    if _SHOW:
        org.paint_uniform_color([0,0,1])
        pcl.paint_uniform_color([0.5,0.5,0])
        o3d.visualization.draw_geometries([org,pcl])
    dist = pcl.compute_point_cloud_distance(org)
    #print(dist)
    distance = np.asarray(dist)
    pclerror = np.sqrt(np.mean(distance ** 2))
    if _DEBUG:
        print("Min:", np.min(distance))
        print("Max:", np.max(distance))
        print("Mean:", np.mean(distance))
        print("MSQ:", pclerror)
    return pclerror, np.min(distance), np.max(distance), np.mean(distance)

def cmp2pcl(org_pcl, test_pcl):
    "compare 2 pcl a pointcloud and return a value for the error"
    if _VERBOSE:
        print("No Points in reference", len(org_pcl.points))
        print("No Points in testfile", len(test_pcl.points))
    dist = test_pcl.compute_point_cloud_distance(org_pcl)
    distance = np.asarray(dist)
    distance = np.asarray(dist)
    pclerror = np.sqrt(np.mean(distance ** 2))
    if _DEBUG:
        print(f"Min error:  {np.min(distance):.5f}")
        print(f"Max error:  {np.max(distance):.5f}")
        print(f"Mean error: {np.mean(distance):.5f}")
        print(f"MSQ:        {pclerror:.5f}")
    return pclerror, np.min(distance), np.max(distance), np.mean(distance)

def cmp_pcl(org_file, pcl_file):
    "compare a org pcl and a pointcloud and return a value for the error"
    if not org_file.exists() or not pcl_file.exists():
        print("File does not exist", org_file, pcl_file)
        sys.exit(1)
    org = o3d.io.read_point_cloud(str(org_file))
    pcl = o3d.io.read_point_cloud(str(pcl_file))
    if _DEBUG:
        print("Points in reference", len(org.points))
        print("Points in testfile", len(pcl.points))
    dist = pcl.compute_point_cloud_distance(org)
    distance = np.asarray(dist)
    pclerror = np.sqrt(np.mean(distance ** 2))
    return pclerror

if __name__ == "__main__":
    #PICFOLDER = Path(__file__).parent / 'testdata'
    parser = argparse.ArgumentParser(prog='compare3d', description='Compare two 3d files and calculate error figures')
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
        print(f"Comparing {fil1} and {fil2}")
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
    else:
        print("Input file type error")
        sys.exit(1)

    if fil2.suffix=='.ply':
        t_pcl = o3d.io.read_point_cloud(str(fil2))

        # elif fil1.suffix=='.stl':
    #     error = cmp_stl(fil1, fil2)
    # else:
    #     print("Illegal file type")
    #     sys.exit(1)
    #print(f"Stl: {fil1} pcl: {fil2} Error: {error:.2e}")
    rms, vmin, vmax, mean = cmp2pcl(in_pcl, t_pcl)
    print(f"Point ABS distance Error: RMS: {rms:.5f} Min: {vmin:.5f} Max: {vmax:.5f} Mean: {mean:.5f}")
    print(f"Point Rel distance Error: RMS: {rms/obj_size*100:.3f}% Min: {vmin/obj_size*100:.3f}% Max: {vmax/obj_size*100:.3f}% Mean: {mean/obj_size*100:.3f}%")

if __name__=="__mainx__":
    BASEDIR = Path(__file__).parent / 'testdata'
    disturb_pcl(BASEDIR / 'LJ3.ply', BASEDIR / 'LJ3_disturb.ply')
    error = cmp_pcl(BASEDIR / 'LJ3.ply', BASEDIR / 'LJ3.ply')
    print(f"Error: {error:.2e}")
    error = cmp_pcl(BASEDIR / 'LJ3.ply', BASEDIR / 'LJ3_disturb.ply')
    print(f"Error: {error:.2e}")
    files = [('LJ3.stl', 'LJ3.ply'),('LJ3.stl', 'LJ3_face.ply'),('LJ3.stl', 'LJ3_face2.ply'),('LJ3.stl', 'LJ3_dist.ply')]
    # files = [('t9UJscan.stl', 'LJ3_face2.ply')]
    # files = [('LJ3.stl', 'LJ3.ply')]
    files = []
    for i1,o1 in files:
        inmesh = BASEDIR / "testdata" / i1
        inpcl = BASEDIR / "testdata" / o1
        error = cmp_stl(inmesh, inpcl)
        print(f"Stl: {i1} pcl: {o1} Error: {error:.2e}")
