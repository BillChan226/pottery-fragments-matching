import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2
import trimesh
import open3d as o3d
from scipy.spatial.distance import cdist
from scipy.optimize import curve_fit
from scipy.interpolate import splprep, splev
from scipy.misc import derivative
from icp import rotation_matrix, best_fit_transform, nearest_neighbor, icp
import time
import copy
from utility import read_cloudpoint, extract_topology, circular_substring_matching

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

obj = []
edge = []
string_c = []
pot = []
num_of_shatter = 5

for i in range(1, num_of_shatter+1):
    o, e = read_cloudpoint("./sphere_shatter/" + str(i) + ".obj")
    obj.append(np.array(o))
    edge.append(np.array(e))
    boundary, curvature, torsion, points = extract_topology(obj[i-1], edge[i-1])
    pot.append(points)

    # Translate
    t = np.random.rand(dim)*translation*0.8
    obj[i-1] += t

    # Rotate
    R = rotation_matrix(np.random.rand(dim), np.random.rand() * rotation)
    obj[i-1] = np.dot(R, obj[i-1].T).T

    # Add noise
    obj[i-1] += np.random.randn(np.shape(obj[i-1])[0], dim) * noise_sigma

    print("./flask_shatter/" + str(i) + ".obj: ", obj[i-1])

    string = list(zip(curvature, torsion))
    print("string: ", string)
    string_c.append(string)

len_string = np.zeros((num_of_shatter,num_of_shatter))
s_r_index1 = []
s_r_index2 = []

for i in range(num_of_shatter-1):
    s_r1_i = []
    s_r2_i = []
    for j in range(i+1, num_of_shatter):

        M_dist, substring, s_r1, s_r2 = circular_substring_matching(string_c[i], string_c[j], epsilon=0.1)
        len_string[i, j] = s_r1[1] - s_r1[0] + 1
        len_string[j, i] = s_r2[1] - s_r2[0] + 1
        s_r1_i.append(s_r1)
        s_r2_i.append(s_r2)

        print("M_dist", M_dist)
        print("substring", substring)
        print("s_r1", s_r1)
        print("s_r2", s_r2)

    s_r_index1.append(s_r1_i)
    s_r_index2.append(s_r2_i)

print("s_r_index1", s_r_index1)
print("s_r_index2", s_r_index2)
print("len_string", len_string)

reassembly = []

for i in range(num_of_shatter):

    string_length = len_string[i]

    for j in range(num_of_shatter):
        # print("string_length[j]", string_length[j])
        # print("len(pot[i])", len(pot[i]))
        if string_length[j] < 1/8*len(pot[i]) or string_length[j] < 1/8*len(pot[j]):

            len_string[i][j] = 0

print("new len_string", len_string)

pairwise = []
for i in range(num_of_shatter):

    string_length = len_string[i]
    # s_r1_i = s_r_index1[i]
    # s_r2_i = s_r_index2[i]

    max_index = 0

    # for j in range(len(s_r1_i)):
    #     if string_length[-j-1] > string_length[max_index]:
    #         max_index = 4-j
    for j in range(num_of_shatter):
        if string_length[j] > string_length[max_index]:
            max_index = j
    #max_index = max_index + i + 1
    # print("####################")
    # print("i", i)
    # print("max_index", max_index)
    # print("####################")
    # print("s_r1_i", s_r1_i)

    if i > max_index:
        start = max_index
        end = i
    elif max_index >=i:
        start = i
        end = max_index

    print("***************************")
    print("start", start)
    print("end", end)
    print("***************************")

    pairwise.append([start, end])
    
# make all the elements in pairwise unique
pairwise = list(set(tuple(sorted(l)) for l in pairwise))
print("pairwise", pairwise)

pcd4 = o3d.geometry.PointCloud()
pcd4.points = o3d.utility.Vector3dVector(obj[0])
pcd5 = o3d.geometry.PointCloud()
pcd5.points = o3d.utility.Vector3dVector(obj[1])
pcd6 = o3d.geometry.PointCloud()
pcd6.points = o3d.utility.Vector3dVector(obj[2])
pcd7 = o3d.geometry.PointCloud()
pcd7.points = o3d.utility.Vector3dVector(obj[3])

# adjusted_points = copy.deepcopy(obj)
adjusted_points = obj
bounded_lines = []
converted_lines = []

for i in range(len(pairwise)):

    (start, end) = pairwise[i]
    s_r1_i = s_r_index1[start]
    s_r2_i = s_r_index2[start]


    s_r1 = s_r1_i[end-1-start]
    s_r2 = s_r2_i[end-1-start]

    sub_vertices1 = []

    for t in pot[start][s_r1[0]:s_r1[1]]:
        #print("vertices1[i]", obj[i][t])
        sub_vertices1.append(adjusted_points[start][t])

    sub_vertices2 = []
    for t in pot[end][s_r2[0]:s_r2[1]]:
        #print("vertices2[i]", obj[max_index][t])
        sub_vertices2.append(adjusted_points[end][t])


    sub_boundary = []
    for i in range(s_r2[0], s_r2[1]):
        sub_boundary.append(pot[end][i])

    lines = []
    # lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
    #         [vertices[edge[0]], vertices[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
    for i in range(1,len(sub_vertices2)):
        #print("edge", edge)
        lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
            [obj[end][sub_boundary[i-1]], obj[end][sub_boundary[i]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
    print("lines", np.shape(np.asarray(lines))[0])

    bounded_lines.append(lines)

    total_time = 0
    # Run ICP
    start = time.time()
    T, distances, iterations = icp(np.array(sub_vertices2), np.array(sub_vertices1), tolerance=0.00001)
    total_time += time.time() - start

    #print("pot[max_index]", pot[max_index])
    # Make C a homogeneous representation of B
    C = np.ones((np.shape(obj[end])[0], 4))
    C[:,0:3] = np.copy(obj[end])
    # Transform C
    C = np.dot(T, C.T).T

    
    adjusted_points[end] = C[:,0:3]
    reassembly.append(C)
    lines = []
    # lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
    #         [vertices[edge[0]], vertices[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
    for i in range(1,len(sub_vertices2)):
        #print("edge", edge)
        lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
            [adjusted_points[end][sub_boundary[i-1]], adjusted_points[end][sub_boundary[i]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
    print("lines", np.shape(np.asarray(lines))[0])
    converted_lines.append(lines)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(adjusted_points[0][:,0:3])
pcd1 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(adjusted_points[1][:,0:3])
pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(adjusted_points[2][:,0:3])
pcd3 = o3d.geometry.PointCloud()
pcd3.points = o3d.utility.Vector3dVector(adjusted_points[3][:,0:3])
pcd4 = o3d.geometry.PointCloud()
pcd4.points = o3d.utility.Vector3dVector(adjusted_points[4][:,0:3])

# pcd2 = o3d.geometry.PointCloud()
# pcd2.points = o3d.utility.Vector3dVector(reassembly[2][:,0:3])
o3d.visualization.draw_geometries([pcd,pcd1,pcd2,pcd3,pcd4]+bounded_lines[0]+bounded_lines[1]+bounded_lines[2]+bounded_lines[3]+converted_lines[0]+converted_lines[1]+converted_lines[2]+converted_lines[3])

# print("distances,", distances,)
# print("iterations", iterations)

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
