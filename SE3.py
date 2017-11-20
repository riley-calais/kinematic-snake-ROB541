import numpy
import GeoUtils


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

    def hard_reset(self, m):
        """ sometimes it's easier to just pass in a matrix"""
        self.matrix = m

    def mult_right(self, h):
        """ multiplies the current SE3 object by another one, which is coming from the right"""
        self.matrix = self.matrix * h.matrix

    def mult_right_mat(self, h):
        """ multiplies the current SE3 object by a raw SE3 matrix, which is coming from the right"""
        self.matrix = self.matrix * h

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

    def inverse(self):
        return self.matrix.getI()
