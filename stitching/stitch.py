"Stitch 2 point clouds"
from pathlib import Path
import open3d as o3d
from . import registration as reg
from . import noise_removal as nr

_DEBUG = True

def show_objects(obj, name=""):
    "Show the object list"
    o3d.visualization.draw_geometries(obj, window_name=name, width=1000, height=1000)
                                  #zoom=0.3412,
                                  #zoom=0.63,
                                  #front=[0.4257, -0.2125, -0.8795],
                                  #front=[-10, 0, -40.8795],
                                  #lookat=[2.6172, 2.0475, 1.532],
                                  #lookat=[0, 0, 6],
                                  #lookat=[0, 0, 10.532],
                                  #up=[-0.0694, -0.9768, 0.2024])
                                  #up=[-10.0694, 0, 0.0])


def read_pointcloud(file: Path):
    "Read a standard pcl"
    if not file.exists():
        print("File does not exists")
        return None
    pcl = o3d.io.read_point_cloud(str(file))
    return pcl


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


def rstitch(org, new):
    "run the stitching"
    color = True
    print("Starting")

    if color:
        if org.has_colors():
            print("infile has color")
        org.paint_uniform_color((1,0,0))

        if new.has_colors():
            print("test file has color")
        new.paint_uniform_color((0,1,0))

    #c_in = clean_point_cloud(org)
    c_test = clean_point_cloud(new)

    objects = [new, c_test]
    show_objects(objects)
    # objects = [ org, new]
    # show_objects(objects)
    return
