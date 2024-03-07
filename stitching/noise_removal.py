"noise removal routines"
import numpy as np
import open3d as o3d

_DEBUG = True

# Limit is ratio of total points in a cluster required to keep it.
def keep_significant_clusters(pcd, limit=0.06, eps=0.35, min_points=7):
    "reduse pointcloud to significant clusters"
    # if _DEBUG:
    #     print(f"keeep significant cluster DBSCAN limit {limit} eps {eps} min_points {min_points}")
    pcd_result = o3d.geometry.PointCloud()
    clusters = pcd.cluster_dbscan(eps, min_points)
    # if _DEBUG:
    #     print("DBSCAN result cluster[0]", clusters[0])
    # Messy way to count how many points are in each cluster.
    cluster_indicies = np.array(clusters) + 1
    # Bincount only counts non-negative.
    cluster_indicies_count = np.bincount(cluster_indicies)
    ii_vec = np.nonzero(cluster_indicies_count)[0]
    # (ii_vec - 1) corrects the one added above.
    counts = zip(ii_vec-1, cluster_indicies_count[ii_vec])
    # if _DEBUG:
    #     print("counts", len(counts))
    kept_indicies = []
    for (cluster, count) in counts:
        if cluster == -1:  # Skip the noise.
            # if _DEBUG:
            #     print("skip", count)
            continue
        # if _DEBUG:
        #     print(count)
        kept = count / len(pcd.points)
        if kept >= limit:
            #print("kept", kept, "limit", limit)
            indicies = get_cluster_indicies(clusters, cluster)
            #print("inde", indicies)
            kept_indicies += indicies
            #print("kept:ind", kept_indicies)
            pcd_result += pcd.select_by_index(indicies)
            if _DEBUG:
                print("kept", kept, "limit", limit)
                #print("inde", indicies)
                #print("kept:ind", kept_indicies)
                #print("inserted")
        else:
            pass
    if _DEBUG:
        print(f"keep_significant_clusters: no points: {len(pcd.points)} kept indicies: {len(kept_indicies)}")
    return (pcd_result, kept_indicies)


def get_cluster_indicies(clusters, cluster):
    "get cluster image"
    #print("get_cluster_indices")
    return [i for i, x in enumerate(clusters) if x == cluster]


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
