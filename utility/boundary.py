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
from utility.curve import calculate_curvature_and_torsion
from utility.utility import read_cloudpoint, extract_topology, circular_substring_matching
from utility.icp import rotation_matrix, best_fit_transform, nearest_neighbor, icp
import time
import copy

# the input will be a numpy array of point clouds of a fracture surface with thickness
# now we need to extract the boundary lines of the fracture surface

