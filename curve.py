import numpy as np
from scipy.interpolate import splprep, splev
from scipy.misc import derivative

# def calculate_curvature_and_torsion(points):
#     # # Calculate the first and second derivatives
#     # d1 = np.gradient(points, axis=0)
#     # d2 = np.gradient(d1, axis=0)

#     # # Calculate the tangent and normal vectors
#     # t = d1 / np.linalg.norm(d1, axis=1)[:, None]
#     # n = d2 - np.sum(d2 * t, axis=1)[:, None] * t
#     # n = n / np.linalg.norm(n, axis=1)[:, None]

#     # # Calculate the binormal vector
#     # b = np.cross(t, n)

#     # # Calculate the curvature and torsion
#     # curvature = np.linalg.norm(np.cross(d1, d2), axis=1) / np.linalg.norm(d1, axis=1)**3
#     # torsion = np.sum(np.cross(d1, d2) * b, axis=1) / np.linalg.norm(np.cross(d1, d2), axis=1)**2

#     # return curvature, torsion
#     # Compute the circle that best fits the points
#     #circle = Circle.from_points(points)
#     curvature = (points[0]-points[1])
#     torsion = (points[1]-points[2])

#     # Define the vectors AB, AC, and BC
#     AB = points[1] - points[0]
#     AC = points[2] - points[0]

#     # Calculate the normal vector to the plane defined by the 3 points
#     normal = np.cross(AB, AC)

#     # Calculate the midpoint of AB and AC
#     mid_AB = (points[0] + points[1]) / 2
#     mid_AC = (points[0] + points[2]) / 2

#     # Calculate the intersection of the two perpendicular bisectors
#     D = np.cross(mid_AB, normal)
#     E = np.cross(mid_AC, normal)
#     P = np.cross(D, E)

#     # Calculate the radius of the circle
#     radius = np.linalg.norm(P - points[0])
#     curvature = 1/radius
#     torsion = [0,0, 0]
#     return [0, curvature, 0], torsion
#     # return curvature, torsion
    

def calculate_curvature_and_torsion(points):
    #print("points", points)
    # first approximate a quadratic curve that pass through these 3 points
    x = np.array([points[0][0], points[1][0], points[2][0]])
    y = np.array([points[0][1], points[1][1], points[2][1]])
    z = np.array([points[0][2], points[1][2], points[2][2]])
    # make sure the derivative you get is invariant to rotation and translation of the curve
    tck, u = splprep([x, y, z], s=0, k=2)
    print("u", u)
    # now calculate the curvature at points[1]
    dx, dy, dz = splev(u, tck, der=1)
    d2x, d2y, d2z = splev(u, tck, der=2)
    curvature = np.linalg.norm(np.cross(np.array([dx[1], dy[1], dz[1]]), np.array([d2x[1], d2y[1], d2z[1]]))) / np.linalg.norm(np.array([dx[1], dy[1], dz[1]]))**3
    torsion = np.sum(np.cross(np.array([dx[1], dy[1], dz[1]]), np.array([d2x[1], d2y[1], d2z[1]])) * np.array([d2x[1], d2y[1], d2z[1]])) / np.linalg.norm(np.cross(np.array([dx[1], dy[1], dz[1]]), np.array([d2x[1], d2y[1], d2z[1]])))**2
    # print("curvature", curvature)
    # print("torsion", torsion)
    return curvature, torsion