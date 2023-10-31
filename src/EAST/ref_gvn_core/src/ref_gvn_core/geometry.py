#!/usr/bin/env python3
"""
Geometry utility functions.
"""

import numpy as np
import numpy.linalg as npla


class FindIntersectionError(Exception):
    """
    User Defined exceptions for functionality that finding intersections between two geometric object.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "FindIntersection exception: {0}".format(self.msg)
        else:
            return "FindIntersection exception"


def ball_and_path(x, r, path):
    """
    Find the furthest intersection between a ball and a path in 2D/3D. (d=2/3)
    Inputs:
        x: coordinates of the ball center (d, )
        r: radius of the ball
        path: a series of waypoints (num_pts, d)
    Output
        status: running of this algorithm
                -2: failed
                -1: path reduced to point
                0: succeeded

        x_star  : furthest intersection (d, )
        B_idx: the furthest index of path xg lies in
                Example: B_idx = 6
                 (0)               (5)                (6)
                 START---->....---> A-----x_star-----> B----> .... ---> END

    Ref: personal notes
    """
    # default return
    status, x_star, B_idx = -2, [], []

    plan_fail = False
    if len(path) < 2:
        raise FindIntersectionError("path len is too short, >=2 waypoints is required!")

    # Test if planning failed
    if np.size(path) == 4:
        identical_criteria = path[0, :] == path[1, :]
        if np.all(identical_criteria):
            plan_fail = True

    # loop backward
    if plan_fail:
        B_idx = 1
        x_star = path[1, :]
        status = -1
    else:
        for path_idx in range(len(path) - 1):
            B = path[-path_idx - 1]
            A = path[-path_idx - 2]
            B_idx = -path_idx - 1
            dAB = npla.norm(A - B)  # segment length ptA from ptB
            u = x - A
            v = B - A
            w1 = np.inner(u, v) / dAB ** 2
            w1hat = max(min(w1, 1), 0)  # normalized to [0,1]
            dist2segAB = npla.norm(u - w1hat * v)

            if dist2segAB > r:
                continue
            else:
                # distance from x to line AB
                dist2lineAB = npla.norm(u - w1 * v)
                # print('DEBUG dist to line AB is %.2f' %(dist2lineAB))
                w2 = np.sqrt(r ** 2 - dist2lineAB ** 2) / dAB  # ratio of |v|
                w = w1 + w2
                w = max(min(w, 1), 0)
                x_star = (1 - w) * A + w * B
                status = 0
                break

    return status, x_star, B_idx
