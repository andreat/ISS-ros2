import numpy as np
cimport numpy as np
from libc cimport math
cimport cython
from ISS.algorithms.utils.mathutils.angle cimport pi_2_pi
from scipy.spatial import KDTree

@cython.boundscheck(False)
@cython.wraparound(False)
cdef np.float64_t[:,::1] vehicle_coord_world(np.ndarray[np.float64_t, ndim=1] center,                 
                np.ndarray[np.float64_t, ndim=2] points, 
                double yaw):
    cdef np.ndarray[np.float64_t, ndim=2] rot_matrix = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    cdef np.ndarray[np.float64_t, ndim=2] points_ = np.dot(points, rot_matrix) + center
    return points_

@cython.boundscheck(False)
@cython.wraparound(False)
cdef np.float64_t[:,::1] world_coord_vehicle(np.ndarray[np.float64_t, ndim=1] center,                 
                np.ndarray[np.float64_t, ndim=2] points, 
                double yaw):
    cdef np.ndarray[np.float64_t, ndim=2] rot_matrix = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    cdef np.ndarray[np.float64_t, ndim=2] points_ = np.dot(points - center, rot_matrix.T)
    return points_

@cython.boundscheck(False)
@cython.wraparound(False)
cdef int collision_check(np.ndarray[np.float64_t, ndim=1] center,                 
                np.ndarray[np.float64_t, ndim=2] points, 
                double yaw, 
                double length=4.4, 
                double width=2.2):        
    # cdef np.ndarray[np.float64_t, ndim=2] rot_matrix = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    # cdef np.ndarray[np.float64_t, ndim=2] points_ = np.dot(points - center, rot_matrix.T)
    cdef np.ndarray[np.float64_t, ndim=2] points_ = np.asarray(world_coord_vehicle(center, points, yaw))
    cdef np.ndarray[np.uint8_t, ndim=1] check_x = np.logical_and(np.greater_equal(points_[:,0], -length/2.), np.less_equal(points_[:,0], length/2.))
    cdef np.ndarray[np.uint8_t, ndim=1] check_y = np.logical_and(np.greater_equal(points_[:,1], -width/2.), np.less_equal(points_[:,1], width/2.))    
    return sum(np.logical_and(check_x, check_y))

@cython.boundscheck(False)
@cython.wraparound(False)
cdef int collision_check_fronthalf(np.ndarray[np.float64_t, ndim=1] center,                 
                np.ndarray[np.float64_t, ndim=2] points, 
                double yaw, 
                double length=4.4, 
                double width=2.2):        
    # cdef np.ndarray[np.float64_t, ndim=2] rot_matrix = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    # cdef np.ndarray[np.float64_t, ndim=2] points_ = np.dot(points - center, rot_matrix.T)
    cdef np.ndarray[np.float64_t, ndim=2] points_ = np.asarray(world_coord_vehicle(center, points, yaw))
    cdef np.ndarray[np.uint8_t, ndim=1] check_x = np.logical_and(np.greater_equal(points_[:,0], 0.), np.less_equal(points_[:,0], length/2.))
    cdef np.ndarray[np.uint8_t, ndim=1] check_y = np.logical_and(np.greater_equal(points_[:,1], -width/2.), np.less_equal(points_[:,1], width/2.))    
    return sum(np.logical_and(check_x, check_y))

def collision_check_index(center, points, yaw, length=4.4, width=2.2):            
    points_ = np.asarray(world_coord_vehicle(center, points, yaw))
    check_x = np.logical_and(np.greater_equal(points_[:,0], -length/2.), np.less_equal(points_[:,0], length/2.))
    check_y = np.logical_and(np.greater_equal(points_[:,1], -width/2.), np.less_equal(points_[:,1], width/2.))
    return np.logical_and(check_x, check_y)

## Bicycle Model
cdef tuple bicycle_model_move(double x, 
    double y, 
    double yaw, 
    double distance, 
    double steer, 
    double L):
    x += distance * math.cos(yaw)
    y += distance * math.sin(yaw)
    yaw += pi_2_pi(distance * math.tan(steer) / L)  # distance/2        

    return (x, y, yaw)

## 2D Collision Checker
class CollisionChecker(object):

    def __init__(self, points, vehicle_length, vehicle_width) -> None:        
        self.points = np.array([(point[0], point[1]) for point in points])
        self.kdtree = KDTree(self.points)
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width
    
    def check_point(self, point, half=False):
        point = np.array(point)
        potential_index = self.kdtree.query_ball_point(point[:2], self.vehicle_length)
        potential_points = self.points[potential_index]
        if half:
            return collision_check_fronthalf(point[:2], potential_points, point[2], self.vehicle_length, self.vehicle_width) == 0
        return collision_check(point[:2], potential_points, point[2], self.vehicle_length, self.vehicle_width) == 0
    
    def check_point_index(self, point):
        point = np.array(point)
        potential_index = self.kdtree.query_ball_point(point[:2], self.vehicle_length)
        potential_points = self.points[potential_index]
        potential_index = np.array(potential_index)
        return potential_index[collision_check_index(point[:2], potential_points, point[2], self.vehicle_length, self.vehicle_width)]

    def check_path(self, path, half=False):
        path = np.array(path)
        path_xy = path[:, :2]
        potential_indices = self.kdtree.query_ball_point(path_xy, self.vehicle_length)        
        for i, index in enumerate(potential_indices):
            potential_points = self.points[index]
            if half:
                if collision_check_fronthalf(path[i][:2], potential_points, path[i][2], self.vehicle_length, self.vehicle_width) > 0:
                    return False
            else:
                if collision_check(path[i][:2], potential_points, path[i][2], self.vehicle_length, self.vehicle_width) > 0:
                    return False
        return True