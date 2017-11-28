from copy import deepcopy
import numpy
import GeoUtils


class SE3:
    def __init__(self, x, y, z, theta):
        """
        :param x: x value of the position
        :param y: y value of the position
        :param z: z value of the position
        :param q: quaternion representing the orientation, as an angle-axis list (w, x, y, z)
        """
        # the axis for our quaternions is always just the y axis, so the rotation matrix is simplified
        q = GeoUtils.quat_from_theta(theta)
        row1 = [q[0]**2 - q[2]**2, 0, 2 * q[0] * q[2], x]
        row2 = [0, q[0]**2 + q[2]**2, 0, y]
        row3 = [-2 * q[0] * q[2], 0, q[0]**2 - q[2]**2, z]
        row4 = [0, 0, 0, 1]
        self.matrix = numpy.matrix([row1, row2, row3, row4])
        self.theta = theta

    def hard_reset(self, m):
        """ sometimes it's easier to just pass in a matrix"""
        self.matrix = deepcopy(m)

    def reset(self, h):
        """ just copy another SE3 object in"""
        self.matrix = deepcopy(h.matrix)
        self.theta = h.theta

    def mult_right(self, h):
        """ multiplies the current SE3 object by another one, which is coming from the right"""
        self.matrix = self.matrix * h.matrix
        self.theta += h.theta

    def x(self):
        """ getter for the x value"""
        return GeoUtils.x_from_matrix(self.matrix)

    def y(self):
        """ getter for the y value"""
        return GeoUtils.y_from_matrix(self.matrix)

    def z(self):
        """ getter for the y value"""
        return GeoUtils.z_from_matrix(self.matrix)

    def vertex(self):
        """"""
        return [self.x(), self.y(), self.z()]


def inverse(m):
    temp = SE3(0, 0, 0, 0)
    temp.matrix = m.matrix.getI()
    temp.theta = -m.theta
    return temp

