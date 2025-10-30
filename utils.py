import numpy as np
from scipy.interpolate import interp1d
import math

def smooth_path(path, num_points=100):
    """
    Smooth a path using cubic spline interpolation
    
    Args:
        path: List of (x,y) waypoints
        num_points: Number of points in smoothed path
        
    Returns:
        Tuple of (smoothed_x, smoothed_y) arrays
    """
    path = np.array(path)
    x = path[:, 0]
    y = path[:, 1]
    
    # Create parameter for interpolation (cumulative distance)
    t = np.zeros(len(x))
    for i in range(1, len(x)):
        t[i] = t[i-1] + math.hypot(x[i]-x[i-1], y[i]-y[i-1])
        
    # Interpolate with cubic splines
    fx = interp1d(t, x, kind='cubic')
    fy = interp1d(t, y, kind='cubic')
    
    # Generate smooth path
    tnew = np.linspace(0, t[-1], num_points)
    xnew = fx(tnew)
    ynew = fy(tnew)
    
    return list(zip(xnew, ynew))

def path_length(path):
    """Calculate total length of a path"""
    length = 0
    for i in range(len(path)-1):
        x1, y1 = path[i]
        x2, y2 = path[i+1]
        length += math.hypot(x2-x1, y2-y1)
    return length

def angle_diff(a, b):
    """Compute smallest angle difference between two angles"""
    diff = (b - a + math.pi) % (2 * math.pi) - math.pi
    return diff

def clip_angle(theta):
    """Clip angle to [-pi, pi]"""
    return math.atan2(math.sin(theta), math.cos(theta))

def to_grid_coords(x, y, resolution=1.0):
    """Convert world coordinates to grid indices"""
    return (int(round(x/resolution)),
            int(round(y/resolution)))

def to_world_coords(i, j, resolution=1.0):
    """Convert grid indices to world coordinates"""
    return (i * resolution,
            j * resolution)