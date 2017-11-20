from numpy import matrix
from math import sin, cos
import SE3


ONE_THIRD = 0.333333333333


def se3_mult(g, h):
    """
    multiplies two SE3 objects, performing g composed with h
    :param g: the left SE3 object
    :param h: the right SE3 object
    :return: a new SE3 object that combines the two
    """
    return g.matrix * h.matrix


def adjoint_inverse(ga, gb, ha):
    """ all parameters are numpy.matrix objects"""
    gai = ga.getI()
    gbi = gb.getI()
    gbi_ga = gbi * ga
    gbi_ga_ha = gbi_ga * ha
    gbi_ga_ha_gai = gbi_ga_ha * gai
    return gbi_ga_ha_gai * gb


def frame_difference(ga, gb):
    """ all parameters are numpy.matrix objects"""
    return ga.getI() * gb


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


def g_left(g, joint_left):
    """ calculates the SE(3) matrix for the left link, g_{-1}"""
    x = -1 * ((g.length * 0.5) + (g.length * 0.5) * cos(-1 * joint_left))
    y = g.y()  # everything should have the same y values, all movement happens in the xz plane
    z = -1 * (g.length * 0.5) * sin(-1 * joint_left)
    multiplier = SE3.SE3(x, y, z, quat_from_theta(-1 * joint_left))
    product = se3_mult(g.gmp, multiplier)
    temp = SE3.SE3(0, 0, 0, quat_from_theta(0))
    temp.hard_reset(product)
    return temp


def g_right(g, joint_right):
    """ calculates the SE(3) matrix for the left link, g_{-1}"""
    x = (g.length * 0.5) + (g.length * 0.5) * cos(joint_right)
    y = g.y()  # everything should have the same y values, all movement happens in the xz plane
    z = (g.length * 0.5) * sin(joint_right)
    multiplier = SE3.SE3(x, y, z, quat_from_theta(joint_right))
    product = se3_mult(g.gmp, multiplier)
    temp = SE3.SE3(0, 0, 0, quat_from_theta(0))
    temp.hard_reset(product)
    return temp


def calc_beta(g, joint_left, joint_right):
    """"""
    x = (g.length * (cos(joint_right) - cos(joint_left))) * ONE_THIRD
    y = g.y()  # everything happens in the xz plane, so the y doesn't change
    z = (g.length * (sin(joint_right) + sin(joint_left))) * ONE_THIRD
    theta = (joint_right - joint_left) * ONE_THIRD
    return SE3.SE3(x, y, z, quat_from_theta(theta))


def system_body_velocity(link0, link1, link2, alpha0, alpha1, alpha0_dot, alpha1_dot):
    """"""
    d = link1.length * sin(alpha0, alpha1) - link0.length * sin(alpha0) + link2 * sin(alpha1)
    if d == 0:
        return -1  # error state
    row1 = -1 * link0.length * (link2.length + link1.length * cos(alpha1))
