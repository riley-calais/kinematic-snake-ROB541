from numpy import matrix
from math import sin, cos, acos
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
    return [cos(theta * 0.5), 0.0, sin(theta * 0.5), 0.0]


def quat_from_theta_prime(theta):
    """ the first derivative of the quaternion"""
    return [-0.5 * sin(theta * 0.5), 0.0, 0.5 * cos(theta * 0.5), 0.0]


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
    z = (g.length * 0.5) * sin(-1 * joint_left)
    multiplier = SE3.SE3(x, y, z, quat_from_theta(-1 * joint_left))
    product = se3_mult(g.gmp, multiplier)
    temp = SE3.SE3(0, 0, 0, quat_from_theta(0))
    temp.hard_reset(product)
    return temp


def g_right(g, joint_right):
    """ calculates the SE(3) matrix for the left link, g_{-1}"""
    x = (g.length * 0.5) + (g.length * 0.5) * cos(joint_right)
    y = g.y()  # everything should have the same y values, all movement happens in the xz plane
    z = -1 * ((g.length * 0.5) * sin(joint_right))
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


def system_body_velocity(length, alpha0, alpha1, alpha0_dot, alpha1_dot):
    """"""
    # d = length * sin(alpha0 + alpha1) - length * sin(alpha0) + length * sin(alpha1)
    # if d == 0:
    #     return -1  # error state
    # d = -1 / d
    # row1 = ((-1 * length * (length + length * cos(alpha0)) * 0.5) * d * alpha0_dot) + \
    #        ((length * (length - length * cos(alpha1)) * 0.5) * d * alpha1_dot)
    # row2 = 0  # the y and z components will be zero for the body velocity of the system frame, so this still works
    # row3 = ((-1 * length * sin(alpha1)) * d * alpha0_dot) + ((length * sin(alpha1)) * d * alpha1_dot)
    # return [row1, row2, row3]  # add a scaling factor to x so we get something visible

    # try another approach:

    half_l = length * 0.5
    row1 = [sin(alpha0), 1, cos(alpha0), (cos(alpha0) - 1) * half_l]
    row2 = [0, 0, 1, 0]
    row3 = [-sin(alpha1), 1, cos(alpha1), (cos(alpha1) + 1) * half_l]
    m = matrix([row1, row2, row3])
    omega_g = -1 * m.getI()
    row1 = [-half_l, 0]
    row2 = [0, 0]
    row3 = [0, half_l]
    omega_b = matrix([row1, row2, row3])
    alpha_dot = matrix([[alpha0_dot], [alpha1_dot]])
    body_velocity = omega_g * omega_b * alpha_dot
    return body_velocity


def world_velocity(g_body_velocity, g):
    """"""
    # we can drop into SE(2) to do these calculations in the x-z plane
    rho_gbv = matrix([[0, -1 * g_body_velocity[2], g_body_velocity[0]],
                      [g_body_velocity[2], 0, g_body_velocity[1]],
                      [0, 0, 0]])
    g_se2_mat = xyz_to_xz(g.gmp.matrix)
    rho_g_dot = g_se2_mat * rho_gbv
    theta = angle_from_mat(rho_g_dot)
    g_dot = SE3.SE3(rho_g_dot.item((0, 2)), 0, rho_g_dot.item((1, 2)), quat_from_theta(theta))
    return g_dot

    # if we have SE3 for everything, then just multiply:
    # rho_gbv = rho_body_velocity(g_body_velocity)
    # g_dot = g.gmp.matrix * rho_gbv
    # temp = SE3.SE3(0, 0, 0, quat_from_theta(0))
    # temp.hard_reset(g_dot)
    # return temp


def rho_body_velocity(body_velocity):
    row1 = [0, 0, 1, body_velocity[0]]
    row2 = [0, 0, 0, body_velocity[1]]
    row3 = [-1, 0, 0, body_velocity[2]]
    row4 = [0, 0, 0, 0]
    return matrix([row1, row2, row3, row4])


def xyz_to_xz(g_mat):
    # the tricky thing here is getting the angle out. We can use trace(R) = 1 + 2cos(theta), as long as -pi < theta < pi
    theta = angle_from_mat(g_mat)
    row1 = [cos(theta), -1 * sin(theta), x_from_matrix(g_mat)]
    row2 = [sin(theta), cos(theta), z_from_matrix(g_mat)]
    row3 = [0, 0, 1]
    return matrix([row1, row2, row3])


def angle_from_mat(m):
    trace = m.item((0, 0)) + m.item((1, 1)) + m.item((2, 2))
    theta = acos((trace - 1) * 0.5)
    return theta




