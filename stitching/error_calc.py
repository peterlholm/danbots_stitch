"Compare pcl  etc to find error figures"

import numpy as np
import open3d as o3d


_DEBUG = True

def surface_to_pcl(mesh, alg="poisson", number_of_points=100000, init_factor=10):
    "convert mesh surfaces to pointcloud, point_factor vertices/points"
    if _DEBUG:
        #mesh_info(mesh)
        print(f"Algoritm: {alg} Number_of_points: {number_of_points} Point_factor: {init_factor}")
    if alg=='poisson':
        pcl = mesh.sample_points_poisson_disk(number_of_points=number_of_points, )
    else:
        pcl = mesh.sample_points_uniformly(number_of_points=number_of_points, init_factor=init_factor)
    return pcl


def cmp2pcl(org_pcl, test_pcl):
    """
    compare 2 pcl a pointcloud and return a value for the error
    the test point cloud must be smaler than the org to return a valid result
    """
    if _DEBUG:
        print("No Points in reference", len(org_pcl.points))
        print("No Points in testfile", len(test_pcl.points))
    dist = org_pcl.compute_point_cloud_distance(test_pcl)
    #print(dist)
    distance = np.asarray(dist)
    if _DEBUG:
        print(distance)
    pclerror = np.sqrt(np.mean(distance ** 2))
    if _DEBUG:
        print(f"Min error:  {np.min(distance):.6f}")
        print(f"Max error:  {np.max(distance):.6f}")
        print(f"Mean error: {np.mean(distance):.6f}")
        print(f"RMS:        {pclerror:.6f} m")
    return pclerror, np.min(distance), np.max(distance), np.mean(distance)

def stitch_error_result(in_pcl, t_pcl, transformation=None):
    "return error from a stitch"
    evalu = o3d.evaluate_registration(in_pcl, t_pcl, transformation)
    print(f"Evaluation of registration: Fitness: {evalu.fitness*100}% RMSE: {evalu.inlier_rmse} CorrespondancesSet: {len(evalu.correspondence_set)}")

if __name__ == "__main__":
    ORGFILE = "testdata/test/serie3/fortand.ply"
    ORGFILE = "testdata/test/serie3/file2.ply"
    in_pcl = o3d.io.read_point_cloud(ORGFILE)
    TESTFILE = "testdata/test/serie3/file1.ply"
    t_pcl = o3d.io.read_point_cloud(TESTFILE)
    pcl_error, min_error, max_error, mean_error = cmp2pcl(in_pcl, t_pcl)
