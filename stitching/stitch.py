"Stitch 2 point clouds"
from pathlib import Path
import open3d as o3d

def read_pointcloud(file: Path):
    "Read a standard pcl"
    if not file.exists():
        print("File does not exists")
        return None
    pcl = o3d.io.read_point_cloud(str(file)))
    return pcl


def stitch(org, new):
    print("Starting")


    return


