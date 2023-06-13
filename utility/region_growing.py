#https://github.com/strawlab/python-pcl/blob/master/tests/test_segmentation.py
import pcl
import numpy as np
import random
import open3d as o3d
import pcl.pcl_visualization
from utility import read_cloudpoint, extract_topology, circular_substring_matching
# p = pcl.load("./table_scene_lms400.pcd")


o1, e1 = read_cloudpoint("../scanned/1.obj")
vertices1 = np.array(o1, dtype="float32")
edges1 = np.array(e1)
print("vertices1", vertices1)
print("edges1", edges1)

pcd1 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(vertices1)

o3d.visualization.draw_geometries([pcd1])

# boundary1, curvature1, torsion1, points1, boundary_vertices1 = extract_topology(vertices1, edges1)


p = pcl.PointCloud()

# points = np.zeros((15, 3), dtype=np.float32)
# RAND_MAX = 1024.0
# for i in range(0, 15):
#     points[i][0] = 1024 * random.random() / (RAND_MAX + 1.0)
#     points[i][1] = 1024 * random.random() / (RAND_MAX + 1.0)
#     points[i][2] = 1.0

# points[0][2] = 2.0
# points[3][2] = -2.0
# points[6][2] = 4.0

p.from_array(vertices1)

vg = p.make_voxel_grid_filter()
# vg.set_leaf_size(0.01, 0.01, 0.01)
vg.set_leaf_size(0.1, 0.1, 0.1)

cloud_filtered = vg.filter()
tree = cloud_filtered.make_kdtree()
segment = cloud_filtered.make_RegionGrowing(ksearch=50)
segment.set_MinClusterSize(100)
segment.set_MaxClusterSize(25000)
segment.set_NumberOfNeighbours(5)

#https://legacy.gitbook.com/book/adioshun/pcl/edit#
segment.set_SmoothnessThreshold(0.2)
segment.set_CurvatureThreshold(0.05)
segment.set_SearchMethod(tree)
cluster_indices = segment.Extract()

print("cluster_indices", np.shape(cluster_indices))

cloud_cluster = pcl.PointCloud()

# visual = pcl.pcl_visualization.CloudViewing()

# # PointXYZ
# visual.ShowMonochromeCloud(cloud_cluster, b'cloud')
# # visual.ShowGrayCloud(ptcloud_centred, b'cloud')
# # visual.ShowColorCloud(ptcloud_centred, b'cloud')
# # visual.ShowColorACloud(ptcloud_centred, b'cloud')

# v = True
# while v:
#     v = not(visual.WasStopped())

# # render the segmented surfaces in 3d display




