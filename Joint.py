import SE3
import GeoUtils
import Primitives

class Joint:
    def __init__(self, x, y, z, theta, alpha):
        # vector from the proximal end of the joint to the distal end
        q_theta = GeoUtils.quat_from_theta(theta)

        self.h = SE3.SE3(x, y, z, q_theta)
        # vector from the proximal end of the joint to the
        self.hmp = SE3.SE3(x / 2.0, y, z, q_theta)
        # assuming these are rotary joints, alpha is the theta value of an SE(2) matrix
        self.alpha = SE3.SE3(0.0, 0.0, 0.0, GeoUtils.quat_from_theta(alpha))

        # create the surfaces around the joint, from the default configuration
        # the top surface is higher y, varying x and z
        self.top = [SE3.SE3(x - 0.2, y + 0.2, -0.5, q_theta),  # back left
                    SE3.SE3(x + 0.2, y + 0.2, -0.5, q_theta),  # back right
                    SE3.SE3(x - 0.2, y + 0.2, 0.5, q_theta),   # front left
                    SE3.SE3(x + 0.2, y + 0.2, 0.5, q_theta)]   # front right
        # the bottom surface is lower y, varying x and z
        self.bottom = [SE3.SE3(x - 0.2, y - 0.2, -0.5, q_theta),  # back left
                       SE3.SE3(x + 0.2, y - 0.2, -0.5, q_theta),  # back right
                       SE3.SE3(x - 0.2, y - 0.2, 0.5, q_theta),  # front left
                       SE3.SE3(x + 0.2, y - 0.2, 0.5, q_theta)]  # front right
        # the left surface is lower x, varying y and z
        self.left = [SE3.SE3(x - 0.2, y - 0.2, -0.5, q_theta),  # back bottom
                     SE3.SE3(x - 0.2, y + 0.2, -0.5, q_theta),  # back top
                     SE3.SE3(x - 0.2, y - 0.2, 0.5, q_theta),  # front bottom
                     SE3.SE3(x - 0.2, y + 0.2, 0.5, q_theta)]  # front top
        # the right surface is higher x, varying y and z
        self.right = [SE3.SE3(x + 0.2, y - 0.2, -0.5, q_theta),  # back bottom
                      SE3.SE3(x + 0.2, y + 0.2, -0.5, q_theta),  # back top
                      SE3.SE3(x + 0.2, y - 0.2, 0.5, q_theta),  # front bottom
                      SE3.SE3(x + 0.2, y + 0.2, 0.5, q_theta)]  # front top

    def draw_joint(self):
        Primitives.rectangle(self.top[0].vertex(), self.top[1].vertex(), self.top[2].vertex(), self.top[3].vertex())
        Primitives.rectangle(self.bottom[0].vertex(), self.bottom[1].vertex(), self.bottom[2].vertex(), self.bottom[3].vertex())
        Primitives.rectangle(self.left[0].vertex(), self.left[1].vertex(), self.left[2].vertex(), self.left[3].vertex())
        Primitives.rectangle(self.right[0].vertex(), self.right[1].vertex(), self.right[2].vertex(), self.right[3].vertex())


