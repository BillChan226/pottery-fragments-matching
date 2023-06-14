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


import pcl 
import pcl.pcl_visualization

FLT_EPSILON = 1e-1 # 这个数我瞎设的，他说ismuchsmaller我也不知道具体应该多少

'''
Inputs:
    points_array: all points array
    point_array: single point
    neighbor_idx: kdtree nearest indice
    u,v: getCoordinateSystemOnPlane
    angle_threshold: rad
Outputs:
    True or False
Reference: https://pointclouds.org/documentation/boundary_8hpp_source.html
'''
def isBoundaryPoint(points_array, point_array, neighbor_idx, u, v, angle_threshold):
    if neighbor_idx.size <= 3: return False
    max_dif = 0
    cp = 0
    angle_array = np.zeros(neighbor_idx.size)
    for i in range(neighbor_idx.size):
        delta = points_array[neighbor_idx[0][i]] - point_array # 1 * 3 XYZ
        if all(delta == np.zeros(3)): continue
        angle = np.arctan2(np.dot(v, delta), np.dot(u, delta)) # (rad)the angles are fine between -PI and PI too
        angle_array[cp] = angle
        cp = cp + 1
    if cp == 0: return False
    angle_array = np.resize(angle_array, cp)
    angle_array = np.sort(angle_array) # 升序
    # Compute the maximal angle difference between two consecutive angles
    for i in range(angle_array.size - 1):
        dif = angle_array[i+1] - angle_array[i]
        if max_dif < dif: max_dif = dif
    # Get the angle difference between the last and the first
    dif = 2 * np.pi - angle_array[angle_array.size - 1] + angle_array[0]
    if max_dif < dif: max_dif = dif

    return (max_dif > angle_threshold)

'''
Inputs: tuple
Outputs: array( np.array([[]]) )
i.e: np.array([[1,2,3]])
# L109
# https://pointclouds.org/documentation/cuda_2common_2include_2pcl_2cuda_2common_2eigen_8h_source.html
'''
def unitOrthogonal(tuple):
    def isMuchSmallerThan(x, y):
        global FLT_EPSILON
        prec_sqr = FLT_EPSILON * FLT_EPSILON
        if x*x <= prec_sqr*y*y:
            return True
        else:
            return False
    if (not isMuchSmallerThan(tuple[0], tuple[1])) or (not isMuchSmallerThan(tuple[1], tuple[2])):
        invum = 1.0 / np.sqrt(tuple[0]**2 + tuple[1]**2)
        perp_x = -tuple[1] * invum
        perp_y = tuple[0] * invum
        perp_z = 0.0
    else:
        invum = 1.0 / np.sqrt(tuple[2]**2 + tuple[1]**2)
        perp_x = 0.0
        perp_y = -tuple[2] * invum
        perp_z = tuple[1] * invum
    perp_array = np.array([[perp_x, perp_y, perp_z]])
    return perp_array

'''
Inputs: point_normal(pcl.PointCloud_Normal)注意这里的normal已经是单位向量了而且是个tuple
pcl::Normal::Normal	(	
float 	n_x,
float 	n_y,
float 	n_z,
float 	_curvature
)		
注意哦：他们四个数的平方和是1，在之前算法线的时候已经算完了
Outputs: u and v
'''
def getCoordinateSystemOnPlane(point_normal):
    normal_tuple = point_normal
    # v = p_coeff_v.unitOrthogonal();
	# u = p_coeff_v.cross3(v);
    # v是normal的正交向量
    v = unitOrthogonal(normal_tuple)
    u = np.cross(point_normal[:3], v)
    return u, v

'''
Inputs: pointCloud(pcl.pointCloud)
Outputs: normals(pcl.PointCloud_Normal)
Reference by demo NormalEstimation.py
'''
def compute_normals(pointCloud):
    ne = pointCloud.make_NormalEstimation()
    tree = pointCloud.make_kdtree()
    ne.set_SearchMethod(tree)
    ne.set_KSearch(30)
    normals = ne.compute()
    return normals

'''
Inputs: 3D-pointcloud(points array) (n*3)XYZ
Outputs: 3D-pointcloud(points array) (n*3)XYZ
'''
def boundary_detection(points):
    # Create a PointCloud
    #points[np.isnan(points)] = 0 # Delete NAN
    #xyz_points = points[:, :3] # Change n*5 to n*3
    print("points", points)
    xyz_points = points
    # xyz_points = np.array(points)
    pc = pcl.PointCloud(xyz_points)
    # Calc Normals 计算法线
    pc_normals = compute_normals(pc)
    # Build a kdtree
    pc_kdtree = pc.make_kdtree_flann()

    boundary_array = np.zeros((1,3))
    for i in range(len(xyz_points)):
        # Find nearestKSearch points
        K = 40
        searchPoint = pcl.PointCloud()
        searchPoint.from_array(np.array([xyz_points[i]]))
        [neighbor_idx, neighbor_dist] = pc_kdtree.nearest_k_search_for_cloud(searchPoint, K)
        # pc[neighbor_idx[0][i]]
        # getCoordinateSystemOnPlane, Obtain a coordinate system on the least-squares plane
        u, v = getCoordinateSystemOnPlane(pc_normals[i])
        # isBoundaryPoint
        if isBoundaryPoint(xyz_points, xyz_points[i], neighbor_idx, u, v, angle_threshold=1.5):
            boundary_array = np.vstack([boundary_array, xyz_points[i]])
    boundary_array = np.delete(boundary_array, 0, axis=0)

    return boundary_array



'''
Test Code use bunny.pcd
调整那个threshold哈
就是isBoundaryPoints()的那个angle_threshold
kdtree的K我也是瞎设的，可以都调调看
'''

pc_array = points[0]
boundary_array = boundary_detection(pc_array)

pcd1 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(boundary_array)

o3d.visualization.draw_geometries([pcd1])

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
