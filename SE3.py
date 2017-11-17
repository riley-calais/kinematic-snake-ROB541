from math import sin, cos, pi
import numpy


class SE3:
    def __init__(self, x, y, z, q):
        """
        :param x: x value of the position
        :param y: y value of the position
        :param z: z value of the position
        :param q: quaternion representing the orientation, as an angle-axis list (w, x, y, z)
        """
        # the axis for our quaternions is always just the y axis, so the rotation matrix is simplified
        row1 = [q[0]**2 - q[2]**2, 0, 2 * q[0] * q[2], x]
        row2 = [0, q[0]**2 + q[2]**2, 0, y]
        row3 = [-2 * q[0] * q[2], 0, q[0]**2 - q[2]**2, z]
        row4 = [0, 0, 0, 1]
        self.matrix = numpy.matrix([row1, row2, row3, row4])
