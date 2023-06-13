import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2
#import trimesh
import open3d as o3d
from scipy.spatial.distance import cdist
from scipy.optimize import curve_fit
from scipy.interpolate import splprep, splev
from scipy.misc import derivative
from math import sqrt
from curve import calculate_curvature_and_torsion
from icp import rotation_matrix, best_fit_transform, nearest_neighbor, icp
import time
import copy

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

    s_r1, s_r2 = [0, 1], [0, 1]
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
