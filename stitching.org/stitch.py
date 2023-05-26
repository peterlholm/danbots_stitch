"Stiching module"
from pathlib import Path
#from os.path import isfile, abspath
#from sys import stderr

import time
from utils.pcl_utils import pcl_info, pcl2pic

from utils.pcl_utils import pcl2jpg, pcl2png
from . import util
from . import registration as reg
from . import noise_removal as nr
# import meshing as mesh
# import evaluate as ev

O3D=True
if O3D:
    import open3d as o3d

_DEBUG = True
_SHOW = True
_TIMING = False

ANTAL = 40

# Default values.
VOXEL_SIZE = 0.3

# Mesh correction
LOW_DENSITY_THRESHOLD = 0.15

# Mesh detail
POISSON_DEPTH = 12

#components = []

def read_pointclouds(folder, filename='pointcloud.ply'):
    "read foldertree with pointcloud files"
    pcls = [o3d.io.read_point_cloud(str(Path(folder) / str(i) / filename)) for i in range(1, ANTAL+1)]
    return pcls

def read_pointcloud_tree(folder, filename='pointcloud.ply', maxnumber=100):
    "Read a standard tree of pointclouds. Returns pcl list"
    pcls = []
    i = 1
    all_ok = True
    while all_ok:
        file = Path(folder) / str(i) / filename
        if file.exists():
            print("appending", str(file))
            pcls.append(o3d.io.read_point_cloud(str(file)))
        else:
            all_ok = False
        i += 1
        if i > maxnumber:
            all_ok = False
    return pcls

def read_pcl_folder(infolder, maxantal=400, color=True):
    "Read a folder with *.ply"
    color_list = [(1.0, 0.0, 0.0),(0.0, 1.0, 0.0),(0.0, 0.0, 1.0),(1.0, 1.0, 0.0),(0.0, 1.0, 1.0),(1.0, 0.0, 1.0)]
    colorno = 0
    if not infolder.exists():
        print("infolder does not exist", infolder)
        return None
    folderfiles = sorted(Path(infolder).glob("*.ply"))
    pcls = []
    antal = 0
    for file in folderfiles:
        inpcl = o3d.io.read_point_cloud(str(file))
        if color:
            if inpcl.has_colors():
                print("file", file, " has color")
            inpcl.paint_uniform_color(color_list[colorno])
            colorno = (colorno +1) % len(color_list)
        if len(inpcl.points)> 50:
            pcls.append(inpcl)
            antal +=1
        else:
            print("To Few points in pointcloud", file)
            print(inpcl)
        if antal>= maxantal:
            break
    print ("read pcl", pcls)
    return pcls

# def write_pointcloud(pcl, outfile):
#     pcl2jpg(pcl, outfile)

def clean_point_cloud(pcd, epsilon=0.35, minimum_points=7, required_share =0.06):
    "clean pointcloud with Pre-stitching cleaning parameters"
    epsilon = 0.35
    minimum_points = 7
    required_share = 0.06
    #print("input points", len(pcd.points) )
    pcd_result, kept_indicies = nr.keep_significant_clusters(pcd, required_share, epsilon, minimum_points)
    if _DEBUG:
        print("Removing ", len(pcd.points) - len(kept_indicies), "points of " + str(len(pcd.points)))
    return pcd_result

def clean_point_cloud_file(path):
    "clean file and save in clean.ply"
    pcl = o3d.io.read_point_cloud(str(path))
    res = clean_point_cloud(pcl, epsilon=0.3, required_share=0.1)
    o3d.io.write_point_cloud(str(path.with_stem('clean')), res)

def reg_point_clouds(components):
    "register point cloud"
    #print("Computing transformations component-wise using RANSAC and ICP.")
    components_with_transformations = [reg.get_transformations(components, VOXEL_SIZE)]
    return components_with_transformations

def transform(components_with_transformations):
    "Applying transformations component-wise."
    #print("Applying transformations component-wise.")
    for component, transformations in components_with_transformations:
        util.transform(component, transformations)

def stitch_run(folder, maxnumber = 100):
    "stitching a folder tree"
    print("Stitching folder:", folder)
    # Start timer, parse config and run pipeline.
    overall_time_start = time.perf_counter()
    print("Loading data.")
    components = read_pointcloud_tree(folder, filename='pointcloud_filter.ply', maxnumber=maxnumber)
    if len(components) == 0:
        print("No pointclouds")
        return False
    stitch_pcl(components, folder)
    overall_timer_stop = time.perf_counter()
    print ("Time consumed", overall_timer_stop-overall_time_start)
    return True

def stitch_pcl(components, outfolder):
    "Stitch all pcl in component"
    time_start = time.perf_counter()
    pcls = []
    no_pointclouds = len(components)
    if _DEBUG:
        pcl_info(components[0])
    if _DEBUG:
        print ("Number pointclouds:", no_pointclouds)
    for i in range(no_pointclouds):
        if _DEBUG:
            print("cleaning pointcloud no", i)
        cpcl = clean_point_cloud(components[i])
        pcls.append(cpcl)
        if _DEBUG:
            pcl2pic(components[i], outfolder / (f"orgin{(i+1):02}.jpg"))
            pcl2jpg(components[i], outfolder / (f"in{(i+1):02}.jpg"))
            #write_pointcloud(components[i], outfolder / (f"in{(i+1):02}.jpg"))
            o3d.io.write_point_cloud(str(outfolder / (f"clean{(i+1):02}.ply")), cpcl)
            pcl2jpg(cpcl, outfolder / (f"clean{(i+1):02}.jpg"))

    if _TIMING:
        print("Cleaning finish", time.perf_counter()-time_start)

    if _DEBUG:
        print("Register pointclouds")
    comp_with_trans = reg_point_clouds(pcls)
    if _TIMING:
        print("Registering finish", time.perf_counter()-time_start)
    #print(registrations)

    if _DEBUG:
        print("Transform pointclouds")
    transform(comp_with_trans)
    if _DEBUG:
        print("Merging component-wise.")
    transformed_components, _ = zip(*comp_with_trans)
    merged_components = []
    print("merging")
    for component, name in zip(transformed_components, ["serie"]):
        print("start", name)
        merged = o3d.geometry.PointCloud()
        for pcl in component:
            merged += pcl
        merged_components.append(merged)
        if _SHOW:
            o3d.visualization.draw(merged, name)
        stitchfile = Path(outfolder) / 'stitch.ply'
        o3d.io.write_point_cloud(str(stitchfile), merged)
        print ("stitchfile: ", stitchfile)
        if _DEBUG:
            pcl2jpg(merged, str(Path(outfolder) / 'stitch.jpg'))
            print("Stitch result")
            pcl_info(merged)
