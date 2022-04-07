# from https://gis.stackexchange.com/questions/22895/finding-minimum-area-rectangle-for-given-points
import sys
import os.path as osp
import os
import numpy as np
from scipy.spatial import ConvexHull

def minimum_bounding_rectangle(points):
    """
    Find the smallest bounding rectangle for a set of points.
    Returns a set of points representing the corners of the bounding box.

    :param points: an nx2 matrix of coordinates
    :rval: an nx2 matrix of coordinates
    """
    from scipy.ndimage.interpolation import rotate
    pi2 = np.pi/2.

    # get the convex hull for the points
    hull_points = points[ConvexHull(points).vertices]

    # calculate edge angles
    edges = np.zeros((len(hull_points)-1, 2))
    edges = hull_points[1:] - hull_points[:-1]

    angles = np.zeros((len(edges)))
    angles = np.arctan2(edges[:, 1], edges[:, 0])

    angles = np.abs(np.mod(angles, pi2))
    angles = np.unique(angles)

    # find rotation matrices
    rotations = np.vstack([
        np.cos(angles),
        np.cos(angles-pi2),
        np.cos(angles+pi2),
        np.cos(angles)]).T
    rotations = rotations.reshape((-1, 2, 2))

    # apply rotations to the hull
    rot_points = np.dot(rotations, hull_points.T)

    # find the bounding points
    min_x = np.nanmin(rot_points[:, 0], axis=1)
    max_x = np.nanmax(rot_points[:, 0], axis=1)
    min_y = np.nanmin(rot_points[:, 1], axis=1)
    max_y = np.nanmax(rot_points[:, 1], axis=1)

    # find the box with the best area
    areas = (max_x - min_x) * (max_y - min_y)
    best_idx = np.argmin(areas)

    # return the best box
    x1 = max_x[best_idx]
    x2 = min_x[best_idx]
    y1 = max_y[best_idx]
    y2 = min_y[best_idx]
    r = rotations[best_idx]

    rval = np.zeros((4, 2))
    rval[0] = np.dot([x1, y2], r)
    rval[1] = np.dot([x2, y2], r)
    rval[2] = np.dot([x2, y1], r)
    rval[3] = np.dot([x1, y1], r)

    return rval

def iter_loadtxt(filename, delimiter=' ', dtype=float):
    def iter_func():
        with open(filename, 'r') as infile:
            for line in infile:
                line = line.rstrip().split(delimiter)
                for item in line:
                    yield dtype(item)
        iter_loadtxt.rowlength = len(line)

    data = np.fromiter(iter_func(), dtype=dtype)
    data = data.reshape((-1, iter_loadtxt.rowlength))
    return data

def loadPoint2D(f):
    path = os.path.dirname(os.path.realpath(__file__))
    txt = iter_loadtxt(os.path.join(path, f))
    pts2D = txt[:, [0, 1]]
    return  pts2D

p2D=loadPoint2D(sys.argv[1])
rect=minimum_bounding_rectangle(p2D)
line = ' '.join((str(rect[0][0]), str(rect[0][1]), str(0.)))
with open(sys.argv[2], "a") as myfile:
    myfile.write("%s\n" % line)
line = ' '.join((str(rect[1][0]), str(rect[1][1]), str(0.)))
with open(sys.argv[2], "a") as myfile:
    myfile.write("%s\n" % line)
line = ' '.join((str(rect[2][0]), str(rect[2][1]), str(0.)))
with open(sys.argv[2], "a") as myfile:
    myfile.write("%s\n" % line)
line = ' '.join((str(rect[3][0]), str(rect[3][1]), str(0.)))
with open(sys.argv[2], "a") as myfile:
    myfile.write("%s\n" % line)
