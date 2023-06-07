import numpy as np
from scipy.interpolate import splprep, splev
from scipy.misc import derivative
#from scipy.spatial import Circle

# # Define sample data points for a 3D curve
# x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
# y = [2, 3, -1, 2, 4, 5, 6, 1, 2, 4]
# z = [-1, -4, -2, 1, 3, -2, -4, -6, 1, 0]

# # Fit a spline curve to the data points
# tck, u = splprep([x, y, z], s=0)

# # Create a new set of points along the curve
# u_new = np.linspace(u.min(), u.max(), 100)
# x_new, y_new, z_new = splev(u_new, tck)

# Compute curvature and torsion

# def curvature(t):
#     """Compute the curvature of a 3D curve at each point"""
#     x_t, y_t, z_t = splev(t, tck, der=1)
#     x_t2, y_t2, z_t2 = splev(t, tck, der=2)
#     num = np.linalg.norm(np.cross([x_t, y_t, z_t], [x_t2, y_t2, z_t2]))
#     denom = np.linalg.norm([x_t, y_t, z_t])**3
#     return num / denom

# def torsion(t):
#     """Compute the torsion of a 3D curve at each point"""
#     x_t, y_t, z_t = splev(t, tck, der=1)
#     x_t2, y_t2, z_t2 = splev(t, tck, der=2)
#     x_t3, y_t3, z_t3 = splev(t, tck, der=3)
#     num = np.dot([x_t, y_t, z_t], np.cross([x_t2, y_t2, z_t2], [x_t3, y_t3, z_t3]))
#     denom = np.linalg.norm(np.cross([x_t, y_t, z_t], [x_t2, y_t2, z_t2])**2)
#     return num / denom

# # curvature_list = [curvature(t) for t in u_new]
# # torsion_list = [torsion(t) for t in u_new]

# # # Print the curvature and torsion values
# # print("Curvature:", curvature_list)
# # print("Torsion:", torsion_list)


import numpy as np

def calculate_curvature_and_torsion(points):
    # # Calculate the first and second derivatives
    # d1 = np.gradient(points, axis=0)
    # d2 = np.gradient(d1, axis=0)

    # # Calculate the tangent and normal vectors
    # t = d1 / np.linalg.norm(d1, axis=1)[:, None]
    # n = d2 - np.sum(d2 * t, axis=1)[:, None] * t
    # n = n / np.linalg.norm(n, axis=1)[:, None]

    # # Calculate the binormal vector
    # b = np.cross(t, n)

    # # Calculate the curvature and torsion
    # curvature = np.linalg.norm(np.cross(d1, d2), axis=1) / np.linalg.norm(d1, axis=1)**3
    # torsion = np.sum(np.cross(d1, d2) * b, axis=1) / np.linalg.norm(np.cross(d1, d2), axis=1)**2

    # return curvature, torsion
    # Compute the circle that best fits the points
    #circle = Circle.from_points(points)
    curvature = (points[0]-points[1])
    torsion = (points[1]-points[2])

    # Define the vectors AB, AC, and BC
    AB = points[1] - points[0]
    AC = points[2] - points[0]

    # Calculate the normal vector to the plane defined by the 3 points
    normal = np.cross(AB, AC)

    # Calculate the midpoint of AB and AC
    mid_AB = (points[0] + points[1]) / 2
    mid_AC = (points[0] + points[2]) / 2

    # Calculate the intersection of the two perpendicular bisectors
    D = np.cross(mid_AB, normal)
    E = np.cross(mid_AC, normal)
    P = np.cross(D, E)

    # Calculate the radius of the circle
    radius = np.linalg.norm(P - points[0])
    curvature = 1/radius
    torsion = [0,0, 0]
    return [0, curvature, 0], torsion
    # return curvature, torsion
    
