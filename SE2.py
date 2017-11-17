from math import sin, cos, pi
import GeoUtils


class SE2:
    """ 3x3 matrix SE(2) representation of (x, y, theta)"""
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        row1 = [cos(theta), -1 * sin(theta), x]
        row2 = [sin(theta), cos(theta), y]
        row3 = [0, 0, 1]
        self.matrix = GeoUtils.matrix_wrapper(row1, row2, row3)
        self.theta = theta  # we need to track theta separately

    def x(self):
        """ getter for the x value"""
        return GeoUtils.x_from_matrix(self.matrix)

    def y(self):
        """ getter for the y value"""
        return GeoUtils.y_from_matrix(self.matrix)


if __name__ == '__main__':
    g = SE2(0.0, 0.0, 0.0)
    h = SE2(0.01, 0.0, pi/100)
    print("x = {}, y = {}, theta = {}".format(g.x(), g.y(), g.theta))
    print("")
    for i in range(10):
        g = GeoUtils.se2_mult(g, h)
    print("x = {}, y = {}, theta - pi = {}".format(g.x(), g.y(), g.theta - pi))



