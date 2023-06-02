"Stitch 2 point clouds"
from pathlib import Path
import open3d as o3d
from . import registration as reg
from . import noise_removal as nr

_DEBUG = True


VOXEL_SIZE = 0.0001

def color_obj(obj, color=(0,0,0)):
    "add color to object"
    obj.paint_uniform_color(color)
    return obj

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
    epsilon = 0.001

    #print("input points", len(pcd.points) )
    pcd_result, kept_indicies = nr.keep_significant_clusters(pcd, required_share, epsilon, minimum_points)
    if _DEBUG:
        print(f"Kept points: {len(kept_indicies)} Removing  {len(pcd.points) - len(kept_indicies)}")
    return pcd_result


def reg_point_clouds(ref, new):
    "register point cloud and find tranformatin bringing new to ref"
    #print("Computing transformations component-wise using RANSAC and ICP.")
    test_target, transformation = reg.get_transformations(ref, new, VOXEL_SIZE)
    return test_target, transformation

def rstitch(reference, new):
    "run the stitching"
    color=True
    if _DEBUG:
        print(f"Reference: {len(reference.points):8} Points, Color: {reference.has_colors()}")
        print(f"Test:      {len(new.points):8} Points, Color: {reference.has_colors()}")
        color = True
    if color:
        reference.paint_uniform_color((1,0,0))
        new.paint_uniform_color((0,1,0))
    if False:   # cleaning
        print("start cleaning")
        c_org = clean_point_cloud(reference)
        c_test = clean_point_cloud(new, epsilon=1)
        color_obj(c_test)
        objects = [c_org, c_test]
        show_objects(objects)

    test_target, transformation = reg_point_clouds(reference, new)
    if _DEBUG:
        print("Regisering test_target", test_target)
        print("Regisering transformation:", transformation)
        print("Registering information matrix", inf_matrix)

    print("Transformation", transformation)
    # objects = [ org, new]
    # show_objects(objects)
    return
