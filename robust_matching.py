import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2
import open3d as o3d
from scipy.spatial.distance import cdist
from scipy.optimize import curve_fit
from scipy.interpolate import splprep, splev
from utility.icp import rotation_matrix, best_fit_transform, nearest_neighbor, icp
import time
from utility.utility import read_cloudpoint, extract_topology, circular_substring_matching
from utility.segmentation import region_growing_segmentation
from utility.boundary_detection import boundary_detection
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.distance import cdist

# Constants
N = 10                                    # number of random points in the dataset
num_tests = 100                             # number of test iterations
dim = 3                                     # number of dimensions of the points
noise_sigma = .001                           # standard deviation error to be added
translation = 5.0                            # max translation of the test set
rotation = .5                              # max rotation (radians) of the test set

# Load OBJ file as a TriangleMesh object
# mesh = o3d.io.read_triangle_mesh("./flask_shatter/1.obj")
#mesh = o3d.io.read_point_cloud("./flask_shatter/1.obj")

# Visualize the mesh
#o3d.visualization.draw_geometries([mesh])
# objFilePath = "./flask_shatter/4.obj"

# #data = np.array(objects["CuscuseraSuperior_cell.002\n"])
# #print("objects", objects)
o1, e1 = read_cloudpoint("./scanned/1.obj")
vertices1 = np.array(o1, dtype="float32")
edges1 = np.array(e1)
print("vertices1", vertices1)
print("edges1", edges1)

points, indices = region_growing_segmentation(vertices1)

pc_array = points[0]
boundary_array = boundary_detection(pc_array)

print("boundary_array", boundary_array)

# now we have the boundary points of the surface, now we should apply gaussian filter and get an ordered list of boundary points

pcd1 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(points[0])
# pcd1.points = o3d.utility.Vector3dVector(boundary_array)


# assuming boundary_array is a 2D array of boundary points
start_point = boundary_array[0]
visited = [0]  # list of visited point indices
unvisited = list(range(1, len(boundary_array)))  # list of unvisited point indices
order = [0]  # list of point indices in the order they are visited
distance_threshold = 2

while unvisited:
    distances = cdist([start_point], boundary_array[unvisited]) # compute the distances between the start_point and the unvisited points
    #print("distances", np.min(distances))
    nearest_idx = unvisited[np.argmin(distances)]
    if np.min(distances) < distance_threshold:
        visited.append(nearest_idx)
        order.append(visited[-1])
        
    unvisited.remove(nearest_idx)

    start_point = boundary_array[visited[-1]]

print("order", order)

boundary_points = []
for i in order:
    boundary_points.append(boundary_array[i])
# assuming x and y are 1D arrays of x and y values for the curve
window_length = 6  # length of the Gaussian filter window
sigma = window_length/3  # standard deviation of the Gaussian filter
smoothed_points = gaussian_filter1d(boundary_points, sigma=sigma, axis=0, mode='nearest', truncate=window_length // 2)

print("smoothed_points", smoothed_points)

print("smoothed_points", np.shape(smoothed_points))
print("order", np.shape(order))

boundary_array = smoothed_points
lines = []
    # lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
    #         [vertices[edge[0]], vertices[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
for i in range(1,len(order)):
    #print("edge", edge)
    lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
        [smoothed_points[i-1], smoothed_points[i]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
print("lines", np.shape(np.asarray(lines))[0])

o3d.visualization.draw_geometries([pcd1]+lines)

# boundary_pc = pcl.PointCloud()
# boundary_pc.from_array(boundary_array.astype(np.float32))

# visual = pcl.pcl_visualization.CloudViewing()
# visual.ShowMonochromeCloud(boundary_pc)
# flag = True
# while flag:
#     if visual.WasStopped() == True:
#         flag = False
#         break

# boundary1, curvature1, torsion1, points1, boundary_vertices1 = extract_topology(vertices1, edges1)
# print("boundary_vertices1", boundary_vertices1)
# np.savetxt('boundary1.txt', boundary_vertices1, fmt='%s')
# print("#################")
# print("curvature1", curvature1)
# print("torsion1", torsion1)
# print("#################")
# o2, e2 = read_cloudpoint("./sphere_shatter/2.obj")
# vertices2 = np.array(o2)
# edges2 = np.array(e2)

# vertice2_original = np.array(o2)

# # # Translate
# # t = np.random.rand(dim)*translation*0.5
# # vertices2 += t

# # # Rotate
# # R = rotation_matrix(np.random.rand(dim), np.random.rand() * rotation)
# # vertices2 = np.dot(R, vertices2.T).T

# # # Add noise
# # vertices2 += np.random.randn(np.shape(vertices2)[0], dim) * noise_sigma * 0.1

# print("vertices2", vertices2)
# print("edges2", edges2)

# boundary2, curvature2, torsion2, points2, boundary_vertices2 = extract_topology(vertices2, edges2)

# # save boundary2 to a file
# print("boundary_vertices2", boundary_vertices2)
# np.savetxt('boundary2.txt', boundary_vertices2, fmt='%s')

# print("#################")
# print("curvature2", curvature2)
# print("torsion2", torsion2)
# print("#################")
# # plt.plot(vertices1)
# # plt.show()

# string1 = list(zip(curvature1, torsion1))
# string2 = list(zip(curvature2, torsion2))
# print("string1", string1)
# print("\nstring2", string2)

# M_dist, substring, s_r1, s_r2 = circular_substring_matching(string1, string2, epsilon=0.5)
# # print M_dist to a file M.txt
# np.savetxt('M.txt', M_dist, fmt='%f')
# print("M_dist", M_dist)
# print("substring", substring)
# print("s_r1", s_r1)
# print("s_r2", s_r2)

# print("points1", points1)
# sub_vertices1 = []
# for i in points1[s_r1[0]:s_r1[1]]:
#     print("vertices1[i]", vertices1[i])
#     sub_vertices1.append(vertices1[i])

# print("points2", points2)
# sub_vertices2 = []
# for i in points2[s_r2[0]:s_r2[1]]:
#     print("vertices2[i]", vertices2[i])
#     sub_vertices2.append(vertices2[i])

# print("sub_vertices1", sub_vertices1)
# print("sub_vertices2", sub_vertices2)

# total_time = 0
# # Run ICP
# start = time.time()
# T, distances, iterations = icp(np.array(sub_vertices2), np.array(sub_vertices1), tolerance=0.00001)
# total_time += time.time() - start

# print("distances,", distances,)
# print("iterations", iterations)

# # Make C a homogeneous representation of B
# C = np.ones((np.shape(vertices2)[0], 4))
# C[:,0:3] = np.copy(vertices2)
# # Transform C
# C = np.dot(T, C.T).T

# # assert np.mean(distances) < 50*noise_sigma                   # mean error should be small
# # assert np.allclose(T[0:3,0:3].T, R, atol=50*noise_sigma)     # T and R should be inverses
# # assert np.allclose(-T[0:3,3], t, atol=50*noise_sigma)        # T and t should be inverses
# print("points2", points2)
# print("vertices2", vertices2)
# print("C", C)
# # print("vertice2_original", vertice2_original)

# pcd1 = o3d.geometry.PointCloud()
# pcd1.points = o3d.utility.Vector3dVector(vertices1)
# pcd2 = o3d.geometry.PointCloud()
# pcd2.points = o3d.utility.Vector3dVector(vertices2)
# pcd3 = o3d.geometry.PointCloud()
# pcd3.points = o3d.utility.Vector3dVector(C[:,0:3])

# sub_boundary = []
# for i in range(s_r1[0], s_r1[1]):
#     sub_boundary.append(points1[i])

# lines = []
# # lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
# #         [vertices[edge[0]], vertices[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# for i in range(1,len(sub_vertices1)):
#     #print("edge", edge)
#     lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
#         [vertices1[sub_boundary[i-1]], vertices1[sub_boundary[i]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# print("lines", np.shape(np.asarray(lines))[0])

# sub_boundary = []
# for i in range(s_r2[0], s_r2[1]):
#     sub_boundary.append(points2[i])
# print("sub_boundary", sub_boundary)

# lines2 = []
# # lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
# #         [vertices[edge[0]], vertices[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# for i in range(1,len(sub_vertices2)):
#     #print("edge", edge)
#     lines2.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
#         [vertices2[sub_boundary[i-1]], vertices2[sub_boundary[i]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# print("lines2", np.shape(np.asarray(lines2))[0])

# o3d.visualization.draw_geometries([pcd1,pcd2,pcd3]+lines+lines2)
