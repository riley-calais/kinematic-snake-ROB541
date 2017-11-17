import SE2


class Joint:
    def __init__(self, x, y, theta, alpha):
        # vector from the proximal end of the joint to the distal end
        self.h = SE2.SE2(x, y, theta)
        # vector from the proximal end of the joint to the
        self.hmp = SE2.SE2(x / 2.0, y, theta)
        # assuming these are rotary joints, alpha is the theta value of an SE(2) matrix
        self.alpha = SE2.SE2(0.0, 0.0, alpha)

        # create the surfaces around the joint, from the default configuration
        # the top surface is higher y, varying x and z
        self.top = [(x - 0.5, y + 0.5, -0.5),  # back left
                    (x + 0.5, y + 0.5, -0.5),  # back right
                    (x + 0.5, y + 0.5, 0.5),   # front right
                    (x - 0.5, y + 0.5, 0.5)]   # front left
        # the bottom surface is lower y, varying x and z
        self.bottom = [(x - 0.5, y - 0.5, -0.5),  # back left
                       (x + 0.5, y - 0.5, -0.5),  # back right
                       (x + 0.5, y - 0.5, 0.5),  # front right
                       (x - 0.5, y - 0.5, 0.5)]  # front left
        # the left surface is lower x, varying y and z
        self.left = [(x - 0.5, y - 0.5, -0.5),  # back bottom
                     (x - 0.5, y + 0.5, -0.5),  # back top
                     (x - 0.5, y + 0.5, 0.5),  # front top
                     (x - 0.5, y - 0.5, 0.5)]  # front bottom
        # the right surface is higher x, varying y and z
        self.right = [(x + 0.5, y - 0.5, -0.5),  # back bottom
                      (x + 0.5, y + 0.5, -0.5),  # back top
                      (x + 0.5, y + 0.5, 0.5),  # front top
                      (x + 0.5, y - 0.5, 0.5)]  # front bottom



