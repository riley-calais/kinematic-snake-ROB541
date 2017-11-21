import math
from numpy import matrix
from OpenGL.GL import *
import SE3
import GeoUtils
import Primitives


class Link:
    def __init__(self):
        q_theta = GeoUtils.quat_from_theta(0.0)  # start with an angle of zero
        x = 0.0
        y = 0.0
        z = 0.0
        y_diff = 0.2
        z_diff = 0.2
        cap_diff = 0.2
        self.length = 2.0
        half_length = self.length * 0.5

        # position and orientation of the proximal frame of the link
        # self.g = SE3.SE3(x, y, z, q_theta)
        # vector from the proximal end of the link to the medial
        self.gmp = SE3.SE3(x, y, z, q_theta)

        # there are four vertices for the top rectangle
        # self.top_back_left = SE3.SE3(x - (half_length - cap_diff), y + y_diff, z - z_diff, q_theta)
        self.top_back_left = matrix([[x - (half_length - cap_diff)], [y + y_diff], [z - z_diff], [1]])
        # self.top_back_right = SE3.SE3(x + (half_length - cap_diff), y + y_diff, z - z_diff, q_theta)
        self.top_back_right = matrix([[x + (half_length - cap_diff)], [y + y_diff], [z - z_diff], [1]])
        # self.top_front_left = SE3.SE3(x - (half_length - cap_diff), y + y_diff, z + z_diff, q_theta)
        self.top_front_left = matrix([[x - (half_length - cap_diff)], [y + y_diff], [z + z_diff], [1]])
        # self.top_front_right = SE3.SE3(x + (half_length - cap_diff), y + y_diff, z + z_diff, q_theta)
        self.top_front_right = matrix([[x + (half_length - cap_diff)], [y + y_diff], [z + z_diff], [1]])
        # there are another four vertices for the bottom rectangle
        # self.bottom_back_left = SE3.SE3(x - (half_length - cap_diff), y - y_diff, z - z_diff, q_theta)
        self.bottom_back_left = matrix([[x - (half_length - cap_diff)], [y - y_diff], [z - z_diff], [1]])
        # self.bottom_back_right = SE3.SE3(x + (half_length - cap_diff), y - y_diff, z - z_diff, q_theta)
        self.bottom_back_right = matrix([[x + (half_length - cap_diff)], [y - y_diff], [z - z_diff], [1]])
        # self.bottom_front_left = SE3.SE3(x - (half_length - cap_diff), y - y_diff, z + z_diff, q_theta)
        self.bottom_front_left = matrix([[x - (half_length - cap_diff)], [y - y_diff], [z + z_diff], [1]])
        # self.bottom_front_right = SE3.SE3(x + (half_length - cap_diff), y - y_diff, z + z_diff, q_theta)
        self.bottom_front_right = matrix([[x + (half_length - cap_diff)], [y - y_diff], [z + z_diff], [1]])
        # there are two more vertices for the end caps
        # self.left_cap = SE3.SE3(x - half_length, y, z, q_theta)
        self.right_cap = matrix([[x + half_length], [y], [z], [1]])
        # self.left_cap = SE3.SE3(x - half_length, y, z, q_theta)
        self.left_cap = matrix([[x - half_length], [y], [z], [1]])

    def update(self, h):
        """"""
        self.gmp = h
        # first we update the current matrix
        # self.gmp.mult_right_mat(h)
        # then we have to update all the rigidly attached vertices, starting with the top
        # adjoint = GeoUtils.adjoint_inverse(self.gmp.matrix, self.top_back_left.matrix, h)
        # self.top_back_left.mult_right_mat(adjoint)
        # adjoint = GeoUtils.adjoint_inverse(self.gmp.matrix, self.top_back_right.matrix, h)
        # self.top_back_right.mult_right_mat(adjoint)
        # adjoint = GeoUtils.adjoint_inverse(self.gmp.matrix, self.top_front_left.matrix, h)
        # self.top_front_left.mult_right_mat(adjoint)
        # adjoint = GeoUtils.adjoint_inverse(self.gmp.matrix, self.top_front_right.matrix, h)
        # self.top_front_right.mult_right_mat(adjoint)
        # # bottom vertices
        # adjoint = GeoUtils.adjoint_inverse(self.gmp.matrix, self.bottom_back_left.matrix, h)
        # self.bottom_back_left.mult_right_mat(adjoint)
        # adjoint = GeoUtils.adjoint_inverse(self.gmp.matrix, self.bottom_back_right.matrix, h)
        # self.bottom_back_right.mult_right_mat(adjoint)
        # adjoint = GeoUtils.adjoint_inverse(self.gmp.matrix, self.bottom_front_left.matrix, h)
        # self.bottom_front_left.mult_right_mat(adjoint)
        # adjoint = GeoUtils.adjoint_inverse(self.gmp.matrix, self.bottom_front_right.matrix, h)
        # self.bottom_front_right.mult_right_mat(adjoint)
        # # end cap vertices
        # adjoint = GeoUtils.adjoint_inverse(self.gmp.matrix, self.left_cap.matrix, h)
        # self.left_cap.mult_right_mat(adjoint)
        # adjoint = GeoUtils.adjoint_inverse(self.gmp.matrix, self.right_cap.matrix, h)
        # self.right_cap.mult_right_mat(adjoint)

    def shift(self, h):
        """ try just multiplying g_dot into the SE(3) matrix"""
        self.gmp.mult_right(h)

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
                             self.gmp.matrix * self.top_front_left, self.gmp.matrix * self.top_front_right, vertex_list)
        # the bottom rectangle is all bottom vertices
        Primitives.rectangle(self.gmp.matrix * self.bottom_back_left, self.gmp.matrix * self.bottom_back_right,
                             self.gmp.matrix * self.bottom_front_left, self.gmp.matrix * self.bottom_front_right,
                             vertex_list)
        # the front rectangle is all front vertices
        Primitives.rectangle(self.gmp.matrix * self.top_front_left, self.gmp.matrix * self.top_front_right,
                             self.gmp.matrix * self.bottom_front_left, self.gmp.matrix * self.bottom_front_right,
                             vertex_list)
        # the back rectangle is all back vertices
        Primitives.rectangle(self.gmp.matrix * self.top_back_left, self.gmp.matrix * self.top_back_right,
                             self.gmp.matrix * self.bottom_back_left, self.gmp.matrix * self.bottom_back_right,
                             vertex_list)

        # the end caps are all left or all right vertices, wih the caps
        Primitives.triangle(self.gmp.matrix * self.top_back_left, self.gmp.matrix * self.left_cap,
                            self.gmp.matrix * self.top_front_left, vertex_list)
        Primitives.triangle(self.gmp.matrix * self.top_back_left, self.gmp.matrix * self.left_cap,
                            self.gmp.matrix * self.bottom_back_left, vertex_list)
        Primitives.triangle(self.gmp.matrix * self.bottom_back_left, self.gmp.matrix * self.left_cap,
                            self.gmp.matrix * self.bottom_front_left, vertex_list)
        Primitives.triangle(self.gmp.matrix * self.bottom_front_left, self.gmp.matrix * self.left_cap,
                            self.gmp.matrix * self.top_front_left, vertex_list)

        Primitives.triangle(self.gmp.matrix * self.top_back_right, self.gmp.matrix * self.right_cap,
                            self.gmp.matrix * self.top_front_right, vertex_list)
        Primitives.triangle(self.gmp.matrix * self.top_back_right, self.gmp.matrix * self.right_cap,
                            self.gmp.matrix * self.bottom_back_right, vertex_list)
        Primitives.triangle(self.gmp.matrix * self.bottom_back_right, self.gmp.matrix * self.right_cap,
                            self.gmp.matrix * self.bottom_front_right, vertex_list)
        Primitives.triangle(self.gmp.matrix * self.bottom_front_right, self.gmp.matrix * self.right_cap,
                            self.gmp.matrix * self.top_front_right, vertex_list)

if __name__ == '__main__':
    q_theta = GeoUtils.quat_from_theta(0.0)  # start with an angle of zero
    x = 0.0
    y = 0.0
    z = 0.0
    y_diff = 0.2
    z_diff = 0.2
    cap_diff = 0.2
    length = 2.0
    half_length = length * 0.5

    # position and orientation of the proximal frame of the link
    # self.g = SE3.SE3(x, y, z, q_theta)
    # vector from the proximal end of the link to the medial
    gmp = SE3.SE3(x, y, z, q_theta)

    # there are four vertices for the top rectangle
    # self.top_back_left = SE3.SE3(x - (half_length - cap_diff), y + y_diff, z - z_diff, q_theta)
    top_back_left = matrix([[x - (half_length - cap_diff)], [y + y_diff], [z - z_diff], [1]])

    print(gmp.matrix * top_back_left)


