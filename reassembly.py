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
from math import sqrt
from curve import calculate_curvature_and_torsion
from icp import rotation_matrix, best_fit_transform, nearest_neighbor, icp
import time

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


# # calculate 
# def calculate_curvature_points(points):
#     print("points", points)
#     # first approximate a quadratic curve that pass through these 3 points
#     x = np.array([points[0][0], points[1][0], points[2][0]])
#     y = np.array([points[0][1], points[1][1], points[2][1]])
#     z = np.array([points[0][2], points[1][2], points[2][2]])
#     tck, u = splprep([x, y, z], s=0, k=2)
#     # now calculate the curvature at points[1]
#     dx, dy, dz = splev(u, tck, der=1)
#     d2x, d2y, d2z = splev(u, tck, der=2)
#     curvature = np.linalg.norm(np.cross(np.array([dx[1], dy[1], dz[1]]), np.array([d2x[1], d2y[1], d2z[1]]))) / np.linalg.norm(np.array([dx[1], dy[1], dz[1]]))**3
#     torsion = np.sum(np.cross(np.array([dx[1], dy[1], dz[1]]), np.array([d2x[1], d2y[1], d2z[1]])) * np.array([d2x[1], d2y[1], d2z[1]])) / np.linalg.norm(np.cross(np.array([dx[1], dy[1], dz[1]]), np.array([d2x[1], d2y[1], d2z[1]])))**2
    
#     return curvature, torsion

def read_cloudpoint(objFilePath):

    with open(objFilePath) as file:
        object_v = []
        edge = []
        while True:
            line = file.readline()
            if not line:
                break
            
            strs = line.split(" ")


            if strs[0] == "v":
                #print(obj_name)
                object_v.append([float(strs[1]), float(strs[2]), float(strs[3])])
            #print(object_v)
            if strs[0] == "vn":
                continue
            if strs[0] == "f":
                face1 = strs[1].split("/")
                face2 = strs[2].split("/")
                face3 = strs[3].split("/")
                e1 = [int(face1[0])-1, int(face2[0])-1]
                e2 = [int(face1[0])-1, int(face3[0])-1]
                e3 = [int(face2[0])-1, int(face3[0])-1]
                edge.append(e1)
                edge.append(e2)
                edge.append(e3)

    return object_v, edge

def extract_topology(vertices, edges):
    boundary = []
    for e in edges:
        unique_flag = True
        count = 0
        for ee in edges:
            if (e[0] == ee[0] and e[1] == ee[1]) or (e[0] == ee[1] and e[1] == ee[0]):
                count += 1
            if count > 1:
                unique_flag = False
                break
        if unique_flag == True:
            boundary.append(e)

    # order the boundary points in a sequence
    points = []
    #print("boundary", boundary[0].tolist())
    set_boundary = set()
    for e in boundary:
        set_boundary.add(e[0])
        set_boundary.add(e[1])

    print("boundary", boundary)
    print("set of boundary", set_boundary)
    points.append(boundary[0][0])
    
    print("length of boundary", len(boundary))

    set_checked = set()
    set_checked.add(0)
    length_mem = 0
    while len(points) < len(boundary) - 1:
        s_p = points[-1]
        for e in boundary:
            if (e[0] == s_p and e[1] not in points):
                points.append(e[1])
                set_checked.add(e[1])
                break
            elif (e[1] == s_p and e[0] not in points):
                points.append(e[0])
                set_checked.add(e[0])
                break
            # elif (e[0] == s_p and e[1] == points[0]) or (e[1] == s_p and e[0] == points[0]):
            #     if (e[0] == s_p):
            #         points.append(e[1])
            #     if (e[1] == s_p):
            #         points.append(e[0])
                # if points[0] == points[-1]:
                #set_inter = set.intersection(set_boundary, set_checked)
                #set_com = set()
        if length_mem == len(points):
            for i in set_boundary:
                if i not in set_checked:
                    # print("i", i)
                    # print("set_checked", set_checked)
                    points.append(i)
                    set_checked.add(i)
                    print("yes!", points[-1])
                    break
        length_mem = len(points)
    # break
        
        #print("length of points", len(points))
        #if len(points)==65: break

    print("points", points)


    # Define sample data points for a 3D curve
    curvature = []
    torsion = []

    print("max(set_boundary)", max(set_boundary))
    for p in range(len(points)):
        x = [vertices[points[p-1]][0], vertices[points[p]][0], vertices[(points[(p+1)%len(points)])][0]]
        y = [vertices[points[p-1]][1], vertices[points[p]][1], vertices[(points[(p+1)%len(points)])][1]]
        z = [vertices[points[p-1]][2], vertices[points[p]][2], vertices[(points[(p+1)%len(points)])][2]]
        # print("x", x)
        # print("y", y)
        # print("z", z)
        p_cur, p_tor = calculate_curvature_and_torsion(np.array([x, y, z]).T)
        # curvature.append(p_cur[1])
        # torsion.append(p_tor[1])
        curvature.append(p_cur)
        torsion.append(p_tor)
        # print("Curvature:", p_cur[1])
        # print("Torsion:", p_tor[1])
   
    print("boundary", boundary)
    print("num of boundary", np.shape(boundary)[0])

    return boundary, curvature, torsion, points


def circular_substring_matching(C1, C2, epsilon):
    m, n = len(C1), len(C2)
    M = np.zeros((m, n))
    for i in range(m):
        for j in range(n):
            distance_sum = 0
            for q in range(-1, 2):
                p1 = np.array(C1[(i+q)%m])
                p2 = np.array(C2[(j+q)%n])
                distance = np.linalg.norm(p1 - p2)
                distance_sum += distance
            M[i, j] = distance_sum / 3

    matches = []

    print("M[i, j]", M)
    # start_1 = -1
    # start_2 = -1
    # end_1 = -1
    # end_2 = -1
    string_range1 = []
    string_range2 = []
    for i in range(m):
        for j in range(n):
            if M[i, j] < epsilon:
                diagonal_elements = []
                k = 0
                while (i+k < m) and (j+k < n) and (M[i+k, j+k] < epsilon):
                    diagonal_elements.append(M[i+k, j+k])
                    k += 1
                if len(diagonal_elements) >= 2:
                    matches.append(diagonal_elements)
                    string_range1.append([i, i+k])
                    string_range2.append([j, j+k])

    max_length = 0
    max_match = []
    for i, match in enumerate(matches):
        if len(match) > max_length:
            max_length = len(match)
            max_match = match
            s_r1 = string_range1[i]
            s_r2 = string_range2[i]

    return M, max_match, s_r1, s_r2

obj = []
edge = []
string_c = []
pot = []
num_of_shatter = 4
for i in range(1, num_of_shatter+1):
    o, e = read_cloudpoint("./sphere_shatter/" + str(i) + ".obj")
    obj.append(np.array(o))
    edge.append(np.array(e))
    boundary, curvature, torsion, points = extract_topology(obj[i-1], edge[i-1])
    pot.append(points)

    # Translate
    t = np.random.rand(dim)*translation
    obj[i-1] += t

    # Rotate
    R = rotation_matrix(np.random.rand(dim), np.random.rand() * rotation)
    obj[i-1] = np.dot(R, obj[i-1].T).T

    # Add noise
    obj[i-1] += np.random.randn(np.shape(obj[i-1])[0], dim) * noise_sigma * 3

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
    
    s_r1_i = s_r_index1[start]
    s_r2_i = s_r_index2[start]


    s_r1 = s_r1_i[end-1-start]
    s_r2 = s_r2_i[end-1-start]
    
    sub_vertices1 = []
    for t in pot[start][s_r1[0]:s_r1[1]]:
        #print("vertices1[i]", obj[i][t])
        sub_vertices1.append(obj[start][t])

    sub_vertices2 = []
    for t in pot[end][s_r2[0]:s_r2[1]]:
        #print("vertices2[i]", obj[max_index][t])
        sub_vertices2.append(obj[end][t])

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
    reassembly.append(C)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(reassembly[1][:,0:3])
pcd1 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(reassembly[0][:,0:3])
# pcd2 = o3d.geometry.PointCloud()
# pcd2.points = o3d.utility.Vector3dVector(reassembly[2][:,0:3])
o3d.visualization.draw_geometries([pcd])

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
