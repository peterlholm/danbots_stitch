"pointcloud registrations"
import copy
import open3d as o3d
import numpy as np

_DEBUG = True
_SHOW = False
_TMPFILE = True

def draw_registration_result(source, target, transformation):
    "Debug draw registration result"
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])
    #vis.add_geometry(axis_pcd)

    o3d.visualization.draw_geometries([source_temp, target_temp, axis_pcd],
                                      zoom=0.2,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[0,0,0],
                                      up=[0.0, 0.0, 10.0])
def compute_normal(pcd):
    "creates normalts that all point in same (wrong) direction (due to low radius)"
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
              radius=0.1, max_nn=30))
    normals_load = np.asarray(pcd.normals) * -1  # Flip normals.
    pcd.normals = o3d.utility.Vector3dVector(normals_load)
    # Get new and correctly orientated normals.
    pcd.estimate_normals()


def preprocess_point_cloud(pcd, voxel_size):
    "prepare point cloud by down sample"
    pcd_down = pcd.voxel_down_sample(voxel_size)
    if _TMPFILE:
        o3d.io.write_point_cloud("/tmp/downsample.ply", pcd_down)
    compute_normal(pcd_down)
    if _TMPFILE:
        o3d.io.write_point_cloud("/tmp/normals.ply", pcd_down)

    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def execute_global_registration(source_down, target_down,
                                source_fpfh, target_fpfh,
                                voxel_size, dist_thres_scalar=1.5,
                                scale=False, edge_length_thres=0.99,
                                converge_itr=(10**8),
                                converge_certainty=0.9999):
    "find global registation"
    distance_threshold = voxel_size * dist_thres_scalar
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
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


def execute_local_registration(source_down, target_down, voxel_size,
                               init_transformation, converge_max_itr=30):
    "Find local registration"
    conver_crit = o3d.pipelines.registration.ICPConvergenceCriteria()
    conver_crit.max_iteration = converge_max_itr
    result_icp = o3d.pipelines.registration.registration_icp(
                    source_down, target_down, voxel_size, init_transformation,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    criteria=conver_crit)
    return result_icp


def prepare_dataset(source, target, voxel_size):
    "prepare data set "
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source_down, target_down, source_fpfh, target_fpfh


def get_transformations(org, test_target, voxel_size):
    "get transformations from pointclouds"
    # n_clouds = len(pcds)
    # pivot = n_clouds // 2
    # if _DEBUG:
    #     print("pivot", pivot)
    #target = o3d.geometry.PointCloud() + pcds[pivot]
    target = org
    transformations = []
    transformations.append(np.identity(4))
    # Process from middle (pivot) and out.
    #processing_order = sorted(range(n_clouds), key=lambda i: abs(pivot-i))
    #print("Processing order", processing_order)
    #for i in processing_order:
    if _DEBUG:
        print("Processing:")
    # if i == pivot:
    #     transformations.append(np.identity(4))
    #     continue
    #else:
    source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(test_target, target, voxel_size)
    print("end preparation")
    result_ransac = execute_global_registration(
            source_down, target_down, source_fpfh, target_fpfh,
            voxel_size)

    result_icp = execute_local_registration(
            source_down, target_down,
            voxel_size, result_ransac.transformation)

    transformation = result_icp.transformation

    source_down.transform(result_icp.transformation)
    target += source_down
    target = target.voxel_down_sample(voxel_size)
    if _DEBUG:
        print("calculating information matrix")
        if _SHOW:
            draw_registration_result(test_target, target, result_icp.transformation )
        print(result_icp)
        print("tansformatin matrix", result_icp.transformation)
        inf_matrix = o3d.pipelines.registration.get_information_matrix_from_point_clouds(test_target, target, 2.0, result_icp.transformation)
        print("information matrix", inf_matrix)

    return test_target, transformation, inf_matrix
