"PCL utils"
from pathlib import Path
import numpy as np
import open3d as o3d


def concatenate_pcl(pcl1, pcl2):
    "concatenate 2 pointclouds with colors"
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

def evaluate_registration(source_pcl, test_pcl, transformation):
    "Calculate the errors in the given transformation"
    max_distance = 1
    evaluation = o3d.pipelines.registration.evaluate_registration(source_pcl, test_pcl, max_distance, transformation)
    print(evaluation)
    return evaluation


if __name__ == "__main__":
    TESTDATA_FOLDER = Path(__file__).parent / 'testdata'
    print(TESTDATA_FOLDER)
    mpcl1 = o3d.io.read_point_cloud(str(TESTDATA_FOLDER / 'pcl/file01.ply'))
    mpcl2 = o3d.io.read_point_cloud(str(TESTDATA_FOLDER / 'pcl/file02.ply'))
    res_pcl = concatenate_pcl(mpcl1, mpcl2)
    o3d.io.write_point_cloud('/tmp/concat.ply', res_pcl)
