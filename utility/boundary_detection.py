
import pcl 
#import pcl.pcl_visualization
import numpy as np

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