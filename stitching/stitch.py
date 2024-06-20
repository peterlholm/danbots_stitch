"Stitch 2 point clouds"
from time import perf_counter
from pathlib import Path
import open3d as o3d
import registration as reg
#import noise_removal as nr
from stitching.error_calc import cmp2pcl

_DEBUG = False


VOXEL_SIZE = 0.0005

def color_obj(obj, color=(0,0,0)):
    "add color to object"
    obj.paint_uniform_color(color)
    return obj

def show_objects(obj, name=""):
    "Show the object list"
    o3d.visualization.draw_geometries(obj, window_name=name, width=500, height=500)
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


# def reg_point_clouds(ref, new):
#     "register point cloud and find tranformatin bringing new to ref"
#     #print("Computing transformations component-wise using RANSAC and ICP.")
#     test_target, transformation = reg.get_transformations(ref, new, VOXEL_SIZE)
#     return test_target, transformation

def r_registration(reference, new, verbose=False, noise_removal=False):
    "run the registation and get transformation matrix"
    color=True
    if _DEBUG:
        print("Input information")
        print(f"Reference: {len(reference.points):8} Points, Color: {reference.has_colors()}")
        print(f"Test:      {len(new.points):8} Points, Color: {new.has_colors()}")
    color = not new.has_colors()
    if color:
        reference.paint_uniform_color((1,0,0))
        new.paint_uniform_color((0,1,0))
    # cleaning
    if noise_removal:
        #print("start cleaning")
        c_org = clean_point_cloud(reference)
        o3d.io.write_point_cloud("/tmp/clean_org.ply", c_org )
        c_test = clean_point_cloud(new, epsilon=1)
        o3d.io.write_point_cloud("/tmp/clean_test.ply", c_test )
        test_target, transformation = reg.get_transformations(c_org, c_test, VOXEL_SIZE, verbose=verbose)
    else:
        test_target, transformation = reg.get_transformations(reference, new, VOXEL_SIZE, verbose=verbose)

    #c_org.paint_uniform_color((1,0,0))
    #show_objects([reference, c_org], name="cleaning")

    if _DEBUG:
        print("Regisering test_target", test_target)
        print("Regisering transformation:", transformation)
    # objects = [ org, new]
    # show_objects(objects)
    return transformation


if __name__ == "__main__":
    #ORGFILE = "testdata/test/serie3/fortand.ply"
    ORGFILE = "testdata/test/serie3/ORG/file_start.ply"
    in_pcl = o3d.io.read_point_cloud(ORGFILE)
    TESTFILE = "testdata/test/serie3/file1.ply"
    t_pcl = o3d.io.read_point_cloud(TESTFILE)
    start_time = perf_counter()
    mytransformation = r_registration(in_pcl, t_pcl)
    end_time = perf_counter()
    print("Final Transformation:\n", mytransformation)
    print("Calculation time", end_time-start_time, "sec")
    trans = t_pcl.transform(mytransformation)
    rms, mmin, mmax, mean = cmp2pcl(in_pcl, trans)
    print(f"Error: Rms {rms:.6f} m  Min {mmin:.6f} m Max {mmax:.6f} m Mean {mean:.6f} m")
    #o3d.io.write_point_cloud("trans.ply", trans )
