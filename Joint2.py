import SE2


class Joint:
    def __init__(self, x, y, theta, alpha):
        # vector from the proximal end of the joint to the distal end
        self.h = SE2.SE2(x, y, theta)
        # vector from the proximal end of the joint to the
        self.hmp = SE2.SE2(x / 2.0, y, theta)
        # assuming these are rotary joints, alpha is the theta value of an SE(2) matrix
        self.alpha = SE2.SE2(0.0, 0.0, alpha)

        # create the lines around the joint, from the default configuration
        # the top surface is higher y, varying x
        self.top = [SE2.SE2(x - 0.5, y + 0.5, theta),  # left
                    SE2.SE2(x + 0.5, y + 0.5, theta)]  # right
        # the bottom surface is lower y, varying x
        self.bottom = [SE2.SE2(x - 0.5, y - 0.5, theta),  # left
                       SE2.SE2(x + 0.5, y - 0.5, theta)]  # right
        # the left surface is lower x, varying y
        self.left = [SE2.SE2(x - 0.5, y - 0.5, theta),  # bottom
                     SE2.SE2(x - 0.5, y + 0.5, theta)]  # top
        # the right surface is higher x, varying y
        self.right = [SE2.SE2(x + 0.5, y - 0.5, theta),  # bottom
                      SE2.SE2(x + 0.5, y + 0.5, theta)]  # top




if __name__ == '__main__':
    j = Joint(0.0, 0.0, 0.0, 0.0)

