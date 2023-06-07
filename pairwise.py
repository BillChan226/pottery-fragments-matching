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
                face1 = strs[1].split("//")
                face2 = strs[2].split("//")
                face3 = strs[3].split("//")
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
        curvature.append(p_cur[1])
        torsion.append(p_tor[1])
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



# #data = np.array(objects["CuscuseraSuperior_cell.002\n"])
# #print("objects", objects)
o1, e1 = read_cloudpoint("./flask_shatter/5.obj")
vertices1 = np.array(o1)
edges1 = np.array(e1)

print("vertices1", vertices1)
print("edges1", edges1)

boundary1, curvature1, torsion1, points1 = extract_topology(vertices1, edges1)

o2, e2 = read_cloudpoint("./flask_shatter/4.obj")
vertices2 = np.array(o2)
edges2 = np.array(e2)

# vertice2_original = np.array(o2)

# Translate
t = np.random.rand(dim)*translation
vertices2 += t

# Rotate
R = rotation_matrix(np.random.rand(dim), np.random.rand() * rotation)
vertices2 = np.dot(R, vertices2.T).T

# Add noise
vertices2 += np.random.randn(np.shape(vertices2)[0], dim) * noise_sigma * 3

print("vertices2", vertices2)
print("edges2", edges2)

boundary2, curvature2, torsion2, points2 = extract_topology(vertices2, edges2)
print("curvature2", curvature2)
print("vertices1", vertices1)
# plt.plot(vertices1)
# plt.show()

string1 = list(zip(curvature1, torsion1))
string2 = list(zip(curvature2, torsion2))
print("string1", string1)
print("\nstring2", string2)

# # Translate
# t = np.random.rand(dim)*translation*2
# vertices2 += t

# # Rotate
# R = rotation_matrix(np.random.rand(dim), np.random.rand() * rotation)
# vertices2 = np.dot(R, vertices2.T).T

# # Add noise
# vertices2 += np.random.randn(np.shape(vertices2)[0], dim) * noise_sigma 

M_dist, substring, s_r1, s_r2 = circular_substring_matching(string1, string2, epsilon=0.1)

print("M_dist", M_dist)
print("substring", substring)
print("s_r1", s_r1)
print("s_r2", s_r2)

print("points1", points1)
sub_vertices1 = []
for i in points1[s_r1[0]:s_r1[1]]:
    print("vertices1[i]", vertices1[i])
    sub_vertices1.append(vertices1[i])

print("points2", points2)
sub_vertices2 = []
for i in points2[s_r2[0]:s_r2[1]]:
    print("vertices2[i]", vertices2[i])
    sub_vertices2.append(vertices2[i])

print("sub_vertices1", sub_vertices1)
print("sub_vertices2", sub_vertices2)

total_time = 0
# Run ICP
start = time.time()
T, distances, iterations = icp(np.array(sub_vertices2), np.array(sub_vertices1), tolerance=0.00001)
total_time += time.time() - start

print("distances,", distances,)
print("iterations", iterations)

# Make C a homogeneous representation of B
C = np.ones((np.shape(vertices2)[0], 4))
C[:,0:3] = np.copy(vertices2)
# Transform C
C = np.dot(T, C.T).T

# assert np.mean(distances) < 50*noise_sigma                   # mean error should be small
# assert np.allclose(T[0:3,0:3].T, R, atol=50*noise_sigma)     # T and R should be inverses
# assert np.allclose(-T[0:3,3], t, atol=50*noise_sigma)        # T and t should be inverses
print("points2", points2)
print("vertices2", vertices2)
print("C", C)
# print("vertice2_original", vertice2_original)

pcd1 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(vertices1)
pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(vertices2)
pcd3 = o3d.geometry.PointCloud()
pcd3.points = o3d.utility.Vector3dVector(C[:,0:3])

sub_boundary = []
for i in range(s_r1[0], s_r1[1]):
    sub_boundary.append(points1[i])

lines = []
# lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
#         [vertices[edge[0]], vertices[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
for i in range(1,len(sub_vertices1)):
    #print("edge", edge)
    lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
        [vertices1[sub_boundary[i-1]], vertices1[sub_boundary[i]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
print("lines", np.shape(np.asarray(lines))[0])

sub_boundary = []
for i in range(s_r2[0], s_r2[1]):
    sub_boundary.append(points2[i])
print("sub_boundary", sub_boundary)

lines2 = []
# lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
#         [vertices[edge[0]], vertices[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
for i in range(1,len(sub_vertices2)):
    #print("edge", edge)
    lines2.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
        [vertices2[sub_boundary[i-1]], vertices2[sub_boundary[i]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
print("lines2", np.shape(np.asarray(lines2))[0])


o3d.visualization.draw_geometries([pcd1,pcd2,pcd3]+lines+lines2)

#  o3d.visualization.draw_geometries([pcd1,pcd2])
# # Visualize the boundary edges
# lines1 = []
# # lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
# #         [vertices[edge[0]], vertices[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# for edge in boundary1:
#     #print("edge", edge)
#     lines1.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
#         [vertices1[edge[0]], vertices1[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# print("lines1", np.shape(np.asarray(lines1))[0])

# lines2 = []
# # lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
# #         [vertices[edge[0]], vertices[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# for edge in boundary2:
#     #print("edge", edge)
#     lines2.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
#         [vertices2[edge[0]], vertices2[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# print("lines2", np.shape(np.asarray(lines2))[0])

# o3d.visualization.draw_geometries([pcd1,pcd2]+lines1+lines2)

# boundary_edges = boundary1
# # boundary_edges = edges#boundary
# # Visualize the boundary edges
# lines = []
# # lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
# #         [vertices[edge[0]], vertices[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# for edge in boundary_edges:
#     print("edge", edge)
#     lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
#         [vertices1[edge[0]], vertices1[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# print("lines", np.shape(np.asarray(lines))[0])
# #o3d.visualization.draw_geometries([pcd1] + lines)

# boundary_edges = boundary2
# # boundary_edges = edges#boundary
# # Visualize the boundary edges
# lines2 = []
# # lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
# #         [vertices[edge[0]], vertices[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# for edge in boundary_edges:
#     print("edge", edge)
#     lines2.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
#         [vertices2[edge[0]], vertices2[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# print("lines2", np.shape(np.asarray(lines2))[0])
# o3d.visualization.draw_geometries([pcd1, pcd2] + lines + lines2)

# triangle_indices = np.asarray(mesh.triangles)

# triangle_vertices = np.asarray(mesh.vertices)

# #triangle_normals = mesh.triangle_normals

# print("Triangle indices:\n", triangle_indices)
# print("triangle_vertices", triangle_vertices)

# #o3d.visualization.draw_geometries([mesh])

# # Extract the boundary edges

# _, cluster, _ =mesh.cluster_connected_triangles()
# print("cluster", cluster)
# mesh_edges = mesh.get_non_manifold_edges(allow_boundary_edges=False)

# edges = np.asarray(mesh_edges).tolist()

# boundary = np.unique(edges)

# print("mesh_edges:", mesh_edges)
# print("edges", edges)
# print("boundary", boundary)
# print("num of mesh_edges:", np.shape(mesh_edges)[0])
# print("num of edges:", np.shape(edges)[0])


# # print("edges[0]", edges[0])
# # print("num of edges:", np.shape(edges)[0])
# # edge_cleaned = []
# # edge_cleaned.append(edges[8])
# # add_flag = True
# # while add_flag == True:
# #     # print("edges", edges)
# #     added = False
# #     for e in edges:
        
# #         #print("e", e)
# #         #if e[0] == edge_cleaned[-1][1] or e[1] == edge_cleaned[-1][1] or e[0] == edge_cleaned[-1][0] or e[1] == edge_cleaned[-1][1]:
# #         if len(set(e).intersection(set(np.concatenate(edge_cleaned))))>0:
# #             # print("np.concatenate(edge_cleaned))", set(np.concatenate(edge_cleaned)))
# #             if e in edge_cleaned:
# #                 # print("++++++++")
# #                 # print("e", e)
# #                 # print("edge_cleaned", edge_cleaned)
# #                 # print("--------")
# #                 continue
            
# #             else:
# #                 edge_cleaned.append(e)
# #                 # print("edge_cleaned", edge_cleaned)
# #                 added = True
                
# #     if added == False:
# #         add_flag = False
            

# # print("edges:", edges)
# # print("edge_cleaned", edge_cleaned)
# # print("num of edges:", np.shape(edges)[0])
# # print("num of cleanup up edges:", np.shape(edge_cleaned)[0])

# #boundary_edges = boundary
# boundary_edges = mesh_edges
# # Visualize the boundary edges
# lines = []
# for edge in boundary_edges:
#     lines.append(o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
#         [mesh.vertices[edge[0]], mesh.vertices[edge[1]]]), lines=o3d.utility.Vector2iVector([[0, 1]])))
# print("lines", np.shape(np.asarray(lines))[0])
# o3d.visualization.draw_geometries([mesh] + lines)


# objFilePath = 'sphere.obj'
# #objFilePath = 'Sketchfab.obj'
# with open(objFilePath) as file:
#     objects = {}
#     object_v = []
#     while True:
#         line = file.readline()
#         if not line:
#             break
        
#         strs = line.split(" ")

#         if strs[0] == "o":
#             obj_name = strs[1]
#             print("obj_name", obj_name)
#             object_v = []
#             objects[obj_name] = object_v

#         if strs[0] == "v":
#             #print(obj_name)
#             objects[obj_name].append([float(strs[1]), float(strs[2]), float(strs[3])])
#         #print(object_v)
#         if strs[0] == "vn" or strs[0] == "f":
#             continue


# #data = np.array(objects["CuscuseraSuperior_cell.002\n"])
# #print("objects", objects)
# data = np.array(objects["sphere1\n"])

# # index_1 = np.random.choice(data.shape[0],100,replace=False)

# # data=data[index_1]

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.scatter3D(data[:,0], data[:,1], data[:,2], c=data[:,2], cmap='cool', s=5)
# plt.show()

# # Convert the data to an Open3D point cloud object
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(data)
# pcd.estimate_normals()

# # Define parameters for the algorithm
# area_threshold = 0.01  # Minimum area of a polygon to be considered
# angle_threshold = np.pi / 6  # Maximum angle deviation from region normal

# #o3d.visualization.draw_geometries([pcd])


# poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]
# # bbox = pcd.get_axis_aligned_bounding_box()
# # p_mesh_crop = poisson_mesh.crop(bbox)


#poisson_mesh = poisson_mesh.simplify_quadric_decimation(100)

#o3d.visualization.draw_geometries([poisson_mesh])



# triangle_indices = np.asarray(poisson_mesh.triangles)

# triangle_vertices = poisson_mesh.vertices

# triangle_normals = poisson_mesh.triangle_normals

# print("Triangle indices:\n", triangle_indices)

# poisson_mesh.compute_triangle_normals()
# print("area", poisson_mesh.get_surface_area())
# print(np.shape(np.asarray(poisson_mesh.triangle_normals)))
# #print("number of triangles", poisson_mesh.triangle_material_ids)

# mesh = poisson_mesh


# # Cluster connected triangles in the mesh
# clusters = mesh.cluster_connected_triangles()

# # Find boundary edges for each cluster
# edge_lists = []
# for cluster in clusters:
#     # Convert triangle indices to edge indices
#     edges = set()
#     for triangle in cluster:
#         print("triangle",triangle)
#         for i in range(3):
#             edge = tuple(sorted([triangle[i], triangle[(i+1)%3]]))
#             if edge in edges:
#                 edges.remove(edge)
#             else:
#                 edges.add(edge)
#     # Extract edge list from edge indices
#     edge_list = [(mesh.triangles[e[0], e[1]], mesh.triangles[e[0], (e[1]+1)%3]) for e in edges]
#     edge_lists.append(edge_list)

# # Visualize the edge curves
# colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
# mesh_edge_list = []
# for i, edge_list in enumerate(edge_lists):
#     color = colors[i % len(colors)]
#     for edge in edge_list:
#         vertices = [mesh.vertices[edge[0]], mesh.vertices[edge[1]]]
#         line = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(vertices))
#         line.paint_uniform_color(color)
#         mesh_edge_list.append(line)
# o3d.visualization.draw_geometries([mesh] + mesh_edge_list)


# # Initialize variables for storing region information
# regions = []  # List of regions, each containing a set of triangle indices
# region_normals = []  # List of region normals

# # Iterate over each triangle in the mesh
# for i in range(np.shape(triangle_indices)[0]):

#     if np.shape(regions)[0]!=0:
#         if i in np.concatenate(regions):  # Triangle already in a region
#             continue
#     vertices = []
#     vertices.extend(triangle_indices[i].tolist())
#     normal = triangle_normals[i]
#     triangle = poisson_mesh.select_by_index(triangle_indices[i])
#     area = triangle.get_surface_area()
#     print("area", triangle.get_surface_area())
#     region = set([i])  # Start with a new region containing only the current triangle
#     region_normal = normal
#     # Propagate region by checking neighboring triangles
#     while True:
#         added_to_region = False
#         for j in range(np.shape(triangle_indices)[0]):
#             # Skip triangles already in the region
#             if j in region:
#                 continue
#             if np.shape(regions)[0]!=0:
#                 if j in np.concatenate(regions):
#                     continue
#             # Check if the triangle neighbours the region and meets similarity criteria
#             vertices_j = triangle_indices[j]
#             normal_j = triangle_normals[j]
#             triangle = poisson_mesh.select_by_index(triangle_indices[j])
#             area = triangle.get_surface_area()
#             #print("dot", normal_j)
#             # print("vertices", vertices)
#             # print("set(vertices)", set(vertices))
#             # print("set(vertices_j)", set(vertices_j))

#             # print(set(vertices).intersection(set(vertices_j)))
#             if len(set(vertices).intersection(set(vertices_j))) > 1:
#                 if np.dot(region_normal/np.linalg.norm(region_normal), normal_j) > 0.8:
#                 # Add to region and update region normal
#                     vertices.extend(vertices_j)
#                     region.add(j)
#                     region_normal = region_normal + area*normal_j
#                     added_to_region = True
#                     # print("set(vertices)", set(vertices))
#                     # print("set(vertices_j)", set(vertices_j))
#                     print(np.dot(region_normal/np.linalg.norm(region_normal), normal_j))
#         if not added_to_region:  # No additional triangles meet criteria
#             break
                
        
#     regions.append(np.array(list(region)))
#     region_normals.append(region_normal)
#     print("regions", regions)

# print("len(regions)", len(regions))
# # Visualize the regions in different colors
# colors = np.random.rand(len(regions), 3)
# colors = colors*255
# color_list = []
# for i in range(np.shape(triangle_indices)[0]):
#     for j in range(len(regions)):
#         if i in regions[j]:
#             color_list.append(colors[j].tolist())
#             break
# print("color_list", color_list)
# mesh.vertex_colors = o3d.utility.Vector3dVector(color_list)

# # Show the mesh with colored regions
# o3d.visualization.draw_geometries([mesh])


# for i in range(np.shape(triangle_indices)[0]):
#     triangle = poisson_mesh.select_by_index(triangle_indices[i])
#     # o3d.visualization.draw_geometries([triangle])
#     print("area", triangle.get_surface_area())
# # Estimate surface normals for the point cloud
# pcd.estimate_normals()
# hull, hull_indices = pcd.compute_convex_hull()

# # Extract the vertices of the triangles that form the convex hull
# vertices = np.asarray(hull.vertices)

# # Determine the unique set of vertices to obtain the boundary
# boundary = np.unique(vertices.reshape(-1,3), axis=0)

# # Order the boundary points by creating a path that connects them
# path = []
# start_point = boundary[0]
# boundary = np.delete(boundary, 0, axis=0)
# while len(boundary) > 0:
#     distances = np.linalg.norm(boundary - start_point, axis=1)
#     nearest_idx = np.argmin(distances)
#     path.append(start_point)
#     start_point = boundary[nearest_idx]
#     boundary = np.delete(boundary, nearest_idx, axis=0)
# path.append(start_point)

# # visualize the boundary points
# boundary_pcd = o3d.geometry.PointCloud()
# boundary_pcd.points = o3d.utility.Vector3dVector(path)
# o3d.visualization.draw_geometries([boundary_pcd])