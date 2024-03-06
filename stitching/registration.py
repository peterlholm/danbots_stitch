"pointcloud registrations"
from time import perf_counter
import copy
import open3d as o3d
import numpy as np

_DEBUG = False
_SHOW = True
_TIMING = False
_TMPFILE = True

# original
GLOBAL_FITNESS = 0.5    # percent to overlap
GLOBAL_RMSE = 0.001
#GLOBAL_FITNESS = 0.2    # percent to overlap
#GLOBAL_RMSE = 0.002

#original
LOCAL_FITNESS = 0.5
LOCAL_RMSE = 0.001


def draw_registration_result(reference, test_source, transformation, axis=False, window_name="registration result", color=True):   # pylint: disable=too-many-arguments
    "Debug draw registration result"
    reference_temp = copy.deepcopy(reference)
    test_temp = copy.deepcopy(test_source)
    if color:
        reference_temp.paint_uniform_color([0, 0.7, 0.1])   # green
        test_temp.paint_uniform_color([1, 0.0, 0.1])        # red
    test_temp.transform(transformation)
    pointclouds =[reference_temp, test_temp]
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, 0, 0])
        pointclouds.append(axis_pcd)
    o3d.visualization.draw_geometries(pointclouds, window_name=window_name, width=500, height=500)

def compute_normal(pcd):
    "creates normalts that all point in same (wrong) direction (due to low radius)"
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
              radius=0.1, max_nn=30))
    normals_load = np.asarray(pcd.normals) * -1  # Flip normals.
    pcd.normals = o3d.utility.Vector3dVector(normals_load)
    # Get new and correctly orientated normals.
    pcd.estimate_normals()


def preprocess_point_cloud(pcd, voxel_size):
    "prepare point cloud by down sample and compute features"
    pcd_down = pcd.voxel_down_sample(voxel_size)
    if _DEBUG:
        print("Preprocession and finding features")
        print(f"Downsample voxel({voxel_size }) to {len(pcd_down.points)} points")
    compute_normal(pcd_down)
    if _TMPFILE:
        o3d.io.write_point_cloud("/tmp/normals.ply", pcd_down)

    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    if _DEBUG:
        print(f"pcd_fpfh features dimension: {pcd_fpfh.dimension()} number features: {pcd_fpfh.num()}")
    return pcd_down, pcd_fpfh

def execute_global_registration(reference_down, target_down,        # pylint: disable=too-many-arguments
                                reference_fpfh, target_fpfh,
                                voxel_size, dist_thres_scalar=1.5,
                                scale=False, edge_length_thres=0.99,
                                converge_itr=(10**8),
                                converge_certainty=0.9999):
    "find global registation"
    if _DEBUG:
        print("Global registration")
    distance_threshold = voxel_size * dist_thres_scalar
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        target_down, reference_down,  target_fpfh, reference_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(scale),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                edge_length_thres),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(
            converge_itr, converge_certainty))
    return result

def execute_local_registration(source_down, reference_down, voxel_size,
                               init_transformation, converge_max_itr=30):
    "Find local registration"
    conver_crit = o3d.pipelines.registration.ICPConvergenceCriteria()
    conver_crit.max_iteration = converge_max_itr
    result_icp = o3d.pipelines.registration.registration_icp(
                    source_down, reference_down, voxel_size, init_transformation,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    criteria=conver_crit)
    return result_icp

def prepare_dataset(ref, test_target, voxel_size):
    "prepare data set "
    if _DEBUG:
        print("Preprocessing reference")
    ref_down, ref_fpfh = preprocess_point_cloud(ref, voxel_size)
    if _DEBUG:
        print("Preprocession test target")
    test_target_down, test_target_fpfh = preprocess_point_cloud(test_target, voxel_size)
    return ref_down, test_target_down, ref_fpfh, test_target_fpfh

def get_transformations(ref, test_target, voxel_size=0.0005, verbose=False):
    "get transformations from pointclouds"
    start_time = perf_counter()
    if _DEBUG:
        print("Get transformations, voxel size", voxel_size)
    ref_down, test_down, ref_fpfh, test_fpfh = prepare_dataset(ref, test_target, voxel_size)
    prepare_time = perf_counter()
    # if _SHOW:
    #     o3d.visualization.draw_geometries([ref_down, test_down], window_name="downsample", height=500, width=500)
    result_ransac = execute_global_registration(
            ref_down, test_down, ref_fpfh, test_fpfh,
            voxel_size)
    global_time = perf_counter()
    if _DEBUG:
        print(f"Global Registration result: Fittnes {result_ransac.fitness} Rms {result_ransac.inlier_rmse}")
    if result_ransac.fitness < GLOBAL_FITNESS or result_ransac.inlier_rmse > GLOBAL_RMSE:
        print("BAD GLOBAL REGISTRATION", result_ransac)
        print(result_ransac.transformation)
        if _SHOW:
            print("global transformation matrix", result_ransac, np.around(result_ransac.transformation,3))
            draw_registration_result(ref_down, test_down, result_ransac.transformation, window_name="Global registration")

        return None, None
    
    local_start = perf_counter()
    result_icp = execute_local_registration(
            test_down, ref_down,
            voxel_size, result_ransac.transformation)
    local_end = perf_counter()
    if _TIMING:
        print("Timing (sec)")
        print("Prepare", prepare_time-start_time, "Global", global_time-prepare_time, "Local", local_end-local_start)
    if result_icp.fitness < LOCAL_FITNESS or result_icp.inlier_rmse >LOCAL_RMSE:
        print("BAD LOCAL REGISTRATION", result_icp)
        if _SHOW:
            print("Local transformation matrix", result_icp, np.around(result_icp.transformation,3))
            draw_registration_result(ref_down, test_down, result_icp.transformation, window_name="Local registration")
            print("global transformation matrix", result_ransac, np.around(result_ransac.transformation,3))
            draw_registration_result(ref_down, test_down, result_ransac.transformation, window_name="Global registration")


    transformation = result_icp.transformation

    # test_down.transform(result_icp.transformation)
    # target += test_down
    # target = target.voxel_down_sample(voxel_size)
        # print("information matrix", inf_matrix)

    return test_target, transformation #, inf_matrix
