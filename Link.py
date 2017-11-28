import math
from numpy import matrix
import SE3
import GeoUtils
import Primitives


class Link:
    def __init__(self, color):
        self.color = color
        x = 0.0
        y = 0.0
        z = 0.0
        y_diff = 0.2
        z_diff = 0.2
        cap_diff = 0.2
        self.length = 2.0
        half_length = self.length * 0.5

        # vector from the proximal end of the link to the medial
        self.gmp = SE3.SE3(x, y, z, GeoUtils.quat_from_theta(0.0))

        # there are four vertices for the top rectangle
        self.top_back_left = matrix([[x - (half_length - cap_diff)], [y + y_diff], [z - z_diff], [1]])
        self.top_back_right = matrix([[x + (half_length - cap_diff)], [y + y_diff], [z - z_diff], [1]])
        self.top_front_left = matrix([[x - (half_length - cap_diff)], [y + y_diff], [z + z_diff], [1]])
        self.top_front_right = matrix([[x + (half_length - cap_diff)], [y + y_diff], [z + z_diff], [1]])
        # there are another four vertices for the bottom rectangle
        self.bottom_back_left = matrix([[x - (half_length - cap_diff)], [y - y_diff], [z - z_diff], [1]])
        self.bottom_back_right = matrix([[x + (half_length - cap_diff)], [y - y_diff], [z - z_diff], [1]])
        self.bottom_front_left = matrix([[x - (half_length - cap_diff)], [y - y_diff], [z + z_diff], [1]])
        self.bottom_front_right = matrix([[x + (half_length - cap_diff)], [y - y_diff], [z + z_diff], [1]])
        # there are two more vertices for the end caps
        self.right_cap = matrix([[x + half_length], [y], [z], [1]])
        self.left_cap = matrix([[x - half_length], [y], [z], [1]])

    def update(self, h):
        """"""
        self.gmp.hard_reset(h)

    def update_com(self, center, beta):
        self.gmp.hard_reset(center.gmp.matrix * beta.matrix)

    def shift(self, velocity):
        """ add the components of g_dot into the SE(3) matrix"""
        x = self.x() + velocity.x()
        y = self.y() + velocity.y()
        z = self.z() + velocity.z()
        theta = GeoUtils.angle_from_mat(self.gmp.matrix) + GeoUtils.angle_from_mat(velocity.matrix)
        self.gmp = SE3.SE3(x, y, z, GeoUtils.quat_from_theta(theta))


    def shift_mat(self, h):
        self.gmp.mult_right_mat(h)

    def displacement(self):
        """"""
        return (self.length * 0.5) + math.sqrt(self.x()**2 + self.y()**2 + self.z()**2)

    def x(self):
        return GeoUtils.x_from_matrix(self.gmp.matrix)

    def y(self):
        return GeoUtils.y_from_matrix(self.gmp.matrix)

    def z(self):
        return GeoUtils.z_from_matrix(self.gmp.matrix) - 0.3  # subtract 0.3 to get to where it started

    def draw_joint(self, vertex_list):
        # the top rectangle is all top vertices
        Primitives.rectangle(self.gmp.matrix * self.top_back_left, self.gmp.matrix * self.top_back_right,
                             self.gmp.matrix * self.top_front_left, self.gmp.matrix * self.top_front_right, vertex_list,
                             self.color)
        # the bottom rectangle is all bottom vertices
        Primitives.rectangle(self.gmp.matrix * self.bottom_back_left, self.gmp.matrix * self.bottom_back_right,
                             self.gmp.matrix * self.bottom_front_left, self.gmp.matrix * self.bottom_front_right,
                             vertex_list, self.color)
        # the front rectangle is all front vertices
        Primitives.rectangle(self.gmp.matrix * self.top_front_left, self.gmp.matrix * self.top_front_right,
                             self.gmp.matrix * self.bottom_front_left, self.gmp.matrix * self.bottom_front_right,
                             vertex_list, self.color)
        # the back rectangle is all back vertices
        Primitives.rectangle(self.gmp.matrix * self.top_back_left, self.gmp.matrix * self.top_back_right,
                             self.gmp.matrix * self.bottom_back_left, self.gmp.matrix * self.bottom_back_right,
                             vertex_list, self.color)

        # the end caps are all left or all right vertices, wih the caps
        Primitives.triangle(self.gmp.matrix * self.top_back_left, self.gmp.matrix * self.left_cap,
                            self.gmp.matrix * self.top_front_left, vertex_list, self.color)
        Primitives.triangle(self.gmp.matrix * self.top_back_left, self.gmp.matrix * self.left_cap,
                            self.gmp.matrix * self.bottom_back_left, vertex_list, self.color)
        Primitives.triangle(self.gmp.matrix * self.bottom_back_left, self.gmp.matrix * self.left_cap,
                            self.gmp.matrix * self.bottom_front_left, vertex_list, self.color)
        Primitives.triangle(self.gmp.matrix * self.bottom_front_left, self.gmp.matrix * self.left_cap,
                            self.gmp.matrix * self.top_front_left, vertex_list, self.color)

        Primitives.triangle(self.gmp.matrix * self.top_back_right, self.gmp.matrix * self.right_cap,
                            self.gmp.matrix * self.top_front_right, vertex_list, self.color)
        Primitives.triangle(self.gmp.matrix * self.top_back_right, self.gmp.matrix * self.right_cap,
                            self.gmp.matrix * self.bottom_back_right, vertex_list, self.color)
        Primitives.triangle(self.gmp.matrix * self.bottom_back_right, self.gmp.matrix * self.right_cap,
                            self.gmp.matrix * self.bottom_front_right, vertex_list, self.color)
        Primitives.triangle(self.gmp.matrix * self.bottom_front_right, self.gmp.matrix * self.right_cap,
                            self.gmp.matrix * self.top_front_right, vertex_list, self.color)

if __name__ == '__main__':
    pass


