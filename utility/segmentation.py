#https://github.com/strawlab/python-pcl/blob/master/tests/test_segmentation.py
import pcl
import numpy as np
import random
import open3d as o3d
import pcl.pcl_visualization
from utility.utility import read_cloudpoint, extract_topology, circular_substring_matching
# p = pcl.load("./table_scene_lms400.pcd")


# o1, e1 = read_cloudpoint("../scanned/1.obj")
# vertices1 = np.array(o1, dtype="float32")
# edges1 = np.array(e1)
# # print("vertices1", vertices1)
# # print("edges1", edges1)

# print("vertices1", np.shape(vertices1))

# pcd1 = o3d.geometry.PointCloud()
# pcd1.points = o3d.utility.Vector3dVector(vertices1)

# o3d.visualization.draw_geometries([pcd1])

# boundary1, curvature1, torsion1, points1, boundary_vertices1 = extract_topology(vertices1, edges1)

def region_growing_segmentation(vertices):

    p = pcl.PointCloud()

    p.from_array(vertices)

    vg = p.make_voxel_grid_filter()
    # vg.set_leaf_size(0.01, 0.01, 0.01)
    vg.set_leaf_size(0.5, 0.5, 0.5)

    cloud_filtered = vg.filter()
    tree = cloud_filtered.make_kdtree()
    segment = cloud_filtered.make_RegionGrowing(ksearch=30)
    segment.set_MinClusterSize(200)
    segment.set_MaxClusterSize(25000)
    segment.set_NumberOfNeighbours(5)

    #https://legacy.gitbook.com/book/adioshun/pcl/edit#
    segment.set_SmoothnessThreshold(0.1)
    # segment.set_CurvatureThreshold(0.05)
    segment.set_CurvatureThreshold(0.01)

    segment.set_SearchMethod(tree)
    cluster_indices = segment.Extract()
    # print("cluster", cluster_indices)
    #print("cluster_indices", np.shape(cluster_indices))

    for i in cluster_indices:
        print(np.shape(i)) 
        
    # print("cluster_indices[0]", cluster_indices[0])
        
    cloud_cluster = pcl.PointCloud()

    # cloud_cluster = pcl.PointCloud()
    all_points = []
    for j, indices in enumerate(cluster_indices):
        # print('indices = ' + str(len(indices)))
        points = np.zeros((len(indices), 3), dtype=np.float32)

        for i, indice in enumerate(indices):
            points[i][0] = cloud_filtered[indice][0]
            points[i][1] = cloud_filtered[indice][1]
            points[i][2] = cloud_filtered[indice][2]

        cloud_cluster.from_array(points)
        
        all_points.append(points)
        
    return all_points, indices

    # visual = pcl.pcl_visualization.CloudViewing()

    #print("points", np.shape(all_points))

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(all_points[0])
    # # pcd.points = o3d.utility.Vector3dVector(vertices1[cluster_indices[0]])

    # o3d.visualization.draw_geometries([pcd])

    # # Create an array of random colors for each point in the point cloud
    # color = [0.5, 0.5, 0.5]

    # # Assign the colors to the point cloud
    # pcd.colors = o3d.utility.Vector3dVector(color)


    # # PointXYZ
    # visual.ShowMonochromeCloud(cloud_cluster, b'cloud')
    # # visual.ShowGrayCloud(ptcloud_centred, b'cloud')
    # # visual.ShowColorCloud(ptcloud_centred, b'cloud')
    # # visual.ShowColorACloud(ptcloud_centred, b'cloud')
    # # visual.ShowColorCloud(cloud_cluster, b'cloud')

    # v = True
    # while v:
    #     v = not(visual.WasStopped())

    # render the segmented surfaces in 3d display





