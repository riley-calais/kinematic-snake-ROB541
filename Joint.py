from OpenGL.GL import *

import SE3
import GeoUtils
import Primitives


class Joint:
    def __init__(self, x, y, z, theta, alpha):
        # vector from the proximal end of the joint to the distal end
        q_theta = GeoUtils.quat_from_theta(theta)
        xdiff = 0.5
        ydiff = 0.2
        zdiff = 0.2
        capdiff = 0.2

        self.g = SE3.SE3(x, y, z, q_theta)
        # vector from the proximal end of the joint to the medial
        x = x / 2.0
        self.gmp = SE3.SE3(x, y, z, q_theta)
        # these are all rotary joints, alpha is the rotation portion of an SE(3) matrix
        self.alpha = SE3.SE3(0.0, 0.0, 0.0, GeoUtils.quat_from_theta(alpha))

        # there are four vertices for the top rectangle
        self.top_back_left = SE3.SE3(x - xdiff, y + ydiff, z - zdiff, q_theta)
        self.top_back_right = SE3.SE3(x + xdiff, y + ydiff, z - zdiff, q_theta)
        self.top_front_left = SE3.SE3(x - xdiff, y + ydiff, z + zdiff, q_theta)
        self.top_front_right = SE3.SE3(x + xdiff, y + ydiff, z + zdiff, q_theta)
        # there are another four vertices for the bottom rectangle
        self.bottom_back_left = SE3.SE3(x - xdiff, y - ydiff, z - zdiff, q_theta)
        self.bottom_back_right = SE3.SE3(x + xdiff, y - ydiff, z - zdiff, q_theta)
        self.bottom_front_left = SE3.SE3(x - xdiff, y - ydiff, z + zdiff, q_theta)
        self.bottom_front_right = SE3.SE3(x + xdiff, y - ydiff, z + zdiff, q_theta)
        # there are two more vertices for the end caps
        self.left_cap = SE3.SE3(x - xdiff - capdiff, y, z, q_theta)
        self.right_cap = SE3.SE3(x + xdiff + capdiff, y, z, q_theta)

    def draw_joint(self):
        glColor3f(0, 1, 0)
        # glBegin(GL_TRIANGLES)
        # the top rectangle is all top vertices
        Primitives.rectangle(self.top_back_left.vertex(), self.top_back_right.vertex(), self.top_front_left.vertex(), self.top_front_right.vertex())
        # the bottom rectangle is all bottom vertices
        Primitives.rectangle(self.bottom_back_left.vertex(), self.bottom_back_right.vertex(), self.bottom_front_left.vertex(), self.bottom_front_right.vertex())
        # glColor3f(0, 0, 1)
        # the front rectangle is all front vertices
        Primitives.rectangle(self.top_front_left.vertex(), self.top_front_right.vertex(), self.bottom_front_left.vertex(), self.bottom_front_right.vertex())
        # the back rectangle is all back vertices
        Primitives.rectangle(self.top_back_left.vertex(), self.top_back_right.vertex(), self.bottom_back_left.vertex(), self.bottom_back_right.vertex())

        # glColor3f(0, 1, 0)
        # the end caps are all left or all right vertices, wih the caps
        Primitives.triangle(self.top_back_left.vertex(), self.left_cap.vertex(), self.top_front_left.vertex())
        Primitives.triangle(self.top_back_left.vertex(), self.left_cap.vertex(), self.bottom_back_left.vertex())
        Primitives.triangle(self.bottom_back_left.vertex(), self.left_cap.vertex(), self.bottom_front_left.vertex())
        # glColor3f(1, 0, 0)
        Primitives.triangle(self.bottom_front_left.vertex(), self.left_cap.vertex(), self.top_front_left.vertex())

        # glColor3f(0, 1, 0)
        Primitives.triangle(self.top_back_right.vertex(), self.right_cap.vertex(), self.top_front_right.vertex())
        Primitives.triangle(self.top_back_right.vertex(), self.right_cap.vertex(), self.bottom_back_right.vertex())
        Primitives.triangle(self.bottom_back_right.vertex(), self.right_cap.vertex(), self.bottom_front_right.vertex())
        # glColor3f(1, 0, 0)
        Primitives.triangle(self.bottom_front_right.vertex(), self.right_cap.vertex(), self.top_front_right.vertex())
        # glEnd()
