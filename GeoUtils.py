from numpy import matrix
from math import sin, cos
import SE2


def se2_mult(g, h):
    """
    multiplies two SE2 objects, performing g composed with h
    :param g: the left SE2 object
    :param h: the right SE2 object
    :return: a new SE2 object that combines the two
    """
    m = g.matrix * h.matrix
    x = x_from_matrix(m)
    y = y_from_matrix(m)
    # we have to just add the thetas since we can't use inverse trig functions reliably
    theta = g.theta + h.theta
    return SE2.SE2(x, y, theta)


def se2_mult_by_mat(g, h, h_theta):
    """
        multiplies SE2 object and a numpy.matrix object, performing g composed with h
        :param g: the left SE2 object
        :param h: the right numpy.matrix object
        :return: a new SE2 object that combines the two
        """
    m = g.matrix * h
    x = x_from_matrix(m)
    y = y_from_matrix(m)
    # we have to just add the thetas since we can't use inverse trig functions reliably
    theta = g.theta + h_theta
    return SE2.SE2(x, y, theta)


def alpha_sum_rotation(links):
    """
    adds up the rotational portion of three links
    :param links: a list of PrismaticJoint and RotationalJoint objects
    :return: the sum of the alpha values for the RotationalJoint objects
    """
    alpha_sum = 0.0
    for l in links:
        alpha_sum += l.alpha.theta
    return alpha_sum


def alpha_sum_x(links):
    """
    adds up the x portion of three links
    :param links: a list of PrismaticJoint and RotationalJoint objects
    :return: the sum of the alpha values for the PrismaticJoint objects
    """
    alpha_sum = 0.0
    for l in links:
        alpha_sum += l.alpha.x()
    return alpha_sum


def quat_from_theta(theta):
    """"""
    return [cos(theta / 2.0), 0.0, sin(theta / 2.0), 0.0]

def x_from_matrix(m):
    """
    returns the x value from an SE(3) matrix
    :param m: the SE(3) matrix, a numpy.matrix object
    :return: the x value from the matrix
    """
    return m.item((0, 3))


def y_from_matrix(m):
    """
    returns the y value from an SE(3) matrix
    :param m: the SE(3) matrix, a numpy.matrix object
    :return: the y value from the matrix
    """
    return m.item((1, 3))


def z_from_matrix(m):
    """
    returns the z value from an SE(3) matrix
    :param m: the SE(3) matrix, a numpy.matrix object
    :return: the z value from the matrix
    """
    return m.item((2, 3))


def matrix_wrapper(row1, row2, row3):
    """ wrapper for the matrix constructor, which saves like 2 characters"""
    return matrix([row1, row2, row3])


def calc_g_delta(g1, g2):
    """ calculate g delta
            we don't have a continuous function for calculus, so we'll just approximate it"""
    return SE2.SE2(g2.x() - g1.x(), g2.y() - g1.y(), g2.theta - g1.theta)


def calc_g_dot(theta, g_delta):
    """ calculates d_dot from the current angle, and approximated deltas"""
    row1 = [(-1 * sin(theta)) * g_delta.theta, (-1 * cos(theta)) * g_delta.theta, g_delta.x()]
    row2 = [cos(theta) * g_delta.theta, (-1 * sin(theta)) * g_delta.theta, g_delta.y()]
    row3 = [0, 0, 0]
    return matrix_wrapper(row1, row2, row3)


def calc_spatial_jacobian_2d(link0, link1):
    """ calculates the spatial Jacobian for g_2d"""
    row1 = [link0.axis[0], cos(link0.alpha_real()), 0]
    row2 = [link0.axis[1], sin(link0.alpha_real()), -1 * (link1.length() + link1.alpha_real())]
    row3 = [link0.axis[2], link1.axis[2], 1]  # this might not be the correct way to get the value, but it works for my purposes
    return [row1, row2, row3]


def calc_spatial_jacobian_1d(link0, link1):
    """ calculates the spatial Jacobian for g_1d"""
    row1 = [link0.axis[0], cos(link0.alpha_real())]
    row2 = [link0.axis[1], sin(link0.alpha_real())]
    row3 = [link0.axis[2], link1.axis[2]]  # this might not be the correct way to get the value, but it works for my purposes
    return [row1, row2, row3]


def TeRg2d(link0, link1, link2):
    """ right lifted action for the third link"""
    alpha01 = alpha_sum_rotation([link0, link1])
    alpha012 = alpha_sum_rotation([link0, link1, link2])
    length01 = alpha_sum_x([link0, link1])
    length012 = alpha_sum_x([link0, link1, link2])
    row1 = [1, 0, -1 * (link0.length()*sin(link0.alpha_real()) + (link1.length() + length01)*sin(alpha01) +
                        (link2.length() + length012)*sin(alpha012))]
    row2 = [0, 1, link0.length()*cos(link0.alpha_real()) + (link1.length() + length01)*cos(alpha01) +
            (link2.length() + length012)*cos(alpha012)]
    row3 = [0, 0, 1]
    return matrix_wrapper(row1, row2, row3)


def TeRg1d(link0, link1):
    """ right lifted action for the second link"""
    alpha01 = alpha_sum_rotation([link0, link1])
    length01 = alpha_sum_x([link0, link1])
    row1 = [1, 0, -1 * (link0.length() * sin(link0.alpha_real()) + (link1.length() + length01) * sin(alpha01))]
    row2 = [0, 1, link0.length() * cos(link0.alpha_real()) + (link1.length() + length01) * cos(alpha01)]
    row3 = [0, 0, 1]
    return matrix_wrapper(row1, row2, row3)


def calc_jacobian_2d(link0, link1, link2):
    """ calculates the full Jacobian for the g_2d by multiplying the spatial Jacobian and the lifted action"""
    spatial_jacobian = calc_spatial_jacobian_2d(link0, link1)
    lifted_action = TeRg2d(link0, link1, link2)
    return lifted_action * spatial_jacobian


def calc_jacobian_1d(link0, link1):
    """ calculates the full Jacobian for the g_1d by multiplying the spatial Jacobian and the lifted action"""
    spatial_velocity = calc_spatial_jacobian_1d(link0, link1)
    lifted_action = TeRg1d(link0, link1)
    return lifted_action * spatial_velocity


def alpha_dot(link1, link2):
    """ returns the diff between the alpha values of two states of a link"""
    return link2.alpha_real() - link1.alpha_real()


def calc_g_dot(jacobian, alphas, n):
    """ we just need to multiply them, but numpy doesn't want to do that"""
    row1 = sum([jacobian.item((0, i)) * alphas.item((0, i)) for i in range(n)])
    row2 = sum([jacobian.item((1, i)) * alphas.item((0, i)) for i in range(n)])
    row3 = sum([jacobian.item((2, i)) * alphas.item((0, i)) for i in range(n)])
    return [row1, row2, row3]


def system_body_velocity(link0, link1, link2, alpha0, alpha1, alpha0_dot, alpha1_dot):
    """"""
    d = link1.length * sin(alpha0, alpha1) - link0.length * sin(alpha0) + link2 * sin(alpha1)
    if d == 0:
        return -1  # error state
    row1 = -1 * link0.length * (link2.length + link1.length * cos(alpha1))

#
# def body_velocity_g1()
