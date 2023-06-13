import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2
import trimesh
import open3d as o3d
from scipy.spatial.distance import cdist
from scipy.optimize import curve_fit
from math import sqrt

objFilePath = 'sphere.obj'
#objFilePath = 'Sketchfab.obj'
with open(objFilePath) as file:
    objects = {}
    object_v = []
    while True:
        line = file.readline()
        if not line:
            break
        
        strs = line.split(" ")

        if strs[0] == "o":
            obj_name = strs[1]
            print("obj_name", obj_name)
            object_v = []
            objects[obj_name] = object_v

        if strs[0] == "v":
            #print(obj_name)
            objects[obj_name].append([float(strs[1]), float(strs[2]), float(strs[3])])
        #print(object_v)
        if strs[0] == "vn" or strs[0] == "f":
            continue


#data = np.array(objects["CuscuseraSuperior_cell.002\n"])
#print("objects", objects)
data = np.array(objects["sphere1\n"])

# index_1 = np.random.choice(data.shape[0],100,replace=False)

# data=data[index_1]

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter3D(data[:,0], data[:,1], data[:,2], c=data[:,2], cmap='cool', s=5)
plt.show()

# Convert the data to an Open3D point cloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(data)
pcd.estimate_normals()

# Define parameters for the algorithm
area_threshold = 0.01  # Minimum area of a polygon to be considered
angle_threshold = np.pi / 6  # Maximum angle deviation from region normal

#o3d.visualization.draw_geometries([pcd])


poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]
# bbox = pcd.get_axis_aligned_bounding_box()
# p_mesh_crop = poisson_mesh.crop(bbox)


#poisson_mesh = poisson_mesh.simplify_quadric_decimation(100)

o3d.visualization.draw_geometries([poisson_mesh])



triangle_indices = np.asarray(poisson_mesh.triangles)

triangle_vertices = poisson_mesh.vertices

triangle_normals = poisson_mesh.triangle_normals

print("Triangle indices:\n", triangle_indices)

poisson_mesh.compute_triangle_normals()
print("area", poisson_mesh.get_surface_area())
print(np.shape(np.asarray(poisson_mesh.triangle_normals)))
#print("number of triangles", poisson_mesh.triangle_material_ids)

mesh = poisson_mesh

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


# Initialize variables for storing region information
regions = []  # List of regions, each containing a set of triangle indices
region_normals = []  # List of region normals

# Iterate over each triangle in the mesh
for i in range(np.shape(triangle_indices)[0]):

    if np.shape(regions)[0]!=0:
        if i in np.concatenate(regions):  # Triangle already in a region
            continue
    vertices = []
    vertices.extend(triangle_indices[i].tolist())
    normal = triangle_normals[i]
    triangle = poisson_mesh.select_by_index(triangle_indices[i])
    area = triangle.get_surface_area()
    print("area", triangle.get_surface_area())
    region = set([i])  # Start with a new region containing only the current triangle
    region_normal = normal
    # Propagate region by checking neighboring triangles
    while True:
        added_to_region = False
        for j in range(np.shape(triangle_indices)[0]):
            # Skip triangles already in the region
            if j in region:
                continue
            if np.shape(regions)[0]!=0:
                if j in np.concatenate(regions):
                    continue
            # Check if the triangle neighbours the region and meets similarity criteria
            vertices_j = triangle_indices[j]
            normal_j = triangle_normals[j]
            triangle = poisson_mesh.select_by_index(triangle_indices[j])
            area = triangle.get_surface_area()
            #print("dot", normal_j)
            # print("vertices", vertices)
            # print("set(vertices)", set(vertices))
            # print("set(vertices_j)", set(vertices_j))

            # print(set(vertices).intersection(set(vertices_j)))
            if len(set(vertices).intersection(set(vertices_j))) > 1:
                if np.dot(region_normal/np.linalg.norm(region_normal), normal_j) > 0.8:
                # Add to region and update region normal
                    vertices.extend(vertices_j)
                    region.add(j)
                    region_normal = region_normal + area*normal_j
                    added_to_region = True
                    # print("set(vertices)", set(vertices))
                    # print("set(vertices_j)", set(vertices_j))
                    print(np.dot(region_normal/np.linalg.norm(region_normal), normal_j))
        if not added_to_region:  # No additional triangles meet criteria
            break
                
        
    regions.append(np.array(list(region)))
    region_normals.append(region_normal)
    print("regions", regions)

print("len(regions)", len(regions))
# Visualize the regions in different colors
colors = np.random.rand(len(regions), 3)
colors = colors*255
color_list = []
for i in range(np.shape(triangle_indices)[0]):
    for j in range(len(regions)):
        if i in regions[j]:
            color_list.append(colors[j].tolist())
            break
print("color_list", color_list)
mesh.vertex_colors = o3d.utility.Vector3dVector(color_list)

# Show the mesh with colored regions
o3d.visualization.draw_geometries([mesh])


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