from pathlib import Path
import numpy as np
import json
import sys
import copy

O3D=True
if O3D:
    import open3d as o3d

def parse_config():
    config_file = Path(__file__).resolve().parent / "config.json"
    #print(config_file)
    # n_args = len(sys.argv)
    # if n_args == 2:
    #     config_file = sys.argv[1]
    # elif n_args > 2:
    #     sys.exit("Expected zero or one argument.")
    try:
        file = open(config_file)
    except IOError:
        sys.exit("Cannot open {}. Please ensure that the file exists and is in "
                 "json format".format(config_file))
    with file as config:
        data = json.load(config)
        path = data["data_format_string"]
        components = [[o3d.io.read_point_cloud(path % i) for i in range(*cr)]
                      for cr in data["component_ranges"]]
        return components, data["component_names"], \
            data["visualize"], data["write"], \
            data["eval"], data["ground_truth"], \
            data["visualize_mesh_density"], data["remove_low_density"], \
            data["use_default_parameters"], data["parameters"]

def my_parse_config():
    config_file = Path(__file__).resolve().parent / "config.json"
    try:
        file = open(config_file)
    except IOError:
        sys.exit("Cannot open {}. Please ensure that the file exists and is in "
                 "json format".format(config_file))
    with file as config:
        data = json.load(config)
        path = data["data_format_string"]
        components = [[o3d.io.read_point_cloud(path % i) for i in range(*cr)]
                      for cr in data["component_ranges"]]
        # return components, data["component_names"], \
        #     data["visualize"], data["write"], \
        #     data["eval"], data["ground_truth"], \
        #     data["visualize_mesh_density"], data["remove_low_density"], \
        #     data["use_default_parameters"], data["parameters"]
        return

def load_point_clouds(num_images=30):
    return [o3d.io.read_point_cloud("data/points%d.ply" % i)
            for i in range(num_images)]


def transform(pcds, transformations):
    for i in range(len(pcds)):
        pcds[i].transform(transformations[i])


def display_inlier_outlier(cloud, ind, title):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([outlier_cloud, inlier_cloud],
                                      window_name=title)

# Late addition. Makes a LineSet of correspondence pairs.
# Longer distance is darker.
def get_correspondence_pair_lineset(pcd_source, pcd_target, registration_result):
    source_tmp = copy.deepcopy(pcd_source)
    source_tmp.transform(registration_result.transformation)
    points = []
    lines = []
    counter = 0
    color = []
    for correspondence_set in registration_result.correspondence_set:
        index_source = correspondence_set[0]
        point_source = source_tmp.points[index_source]
        index_target = correspondence_set[1]
        point_target = pcd_target.points[index_target]
        points.append(point_source)
        points.append(point_target)
        lines.append([counter, counter+1])
        counter += 2
        # minus as want greater length to have lower value (darker color)
        color.append(- np.linalg.norm(point_source - point_target))

    color = np.asarray(color)
    max_length = -np.amin(color)
    min_length = -np.amax(color)
    color = color + min_length + max_length
    color = color / (max_length)

    colors = [[c,c,c] for c in color]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set
