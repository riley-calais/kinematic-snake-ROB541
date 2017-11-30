from numpy import matrix
from math import sin, cos, acos
import SE3


ONE_THIRD = 0.333333333333


def quat_from_theta(theta):
    """ our axis is always (0, 1, 0) ,so q1 and q3 are always 0"""
    return [cos(theta * 0.5), 0.0, sin(theta * 0.5), 0.0]


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
    multiplier = SE3.SE3(x, y, z, -joint_left)
    temp = SE3.SE3(0, 0, 0, 0)
    temp.reset(g.gmp)
    temp.mult_right(multiplier)
    return temp


def g_right(g, joint_right):
    """ calculates the SE(3) matrix for the left link, g_{-1}"""
    x = (g.length * 0.5) + (g.length * 0.5) * cos(joint_right)
    y = g.y()  # everything should have the same y values, all movement happens in the xz plane
    z = -((g.length * 0.5) * sin(joint_right))
    multiplier = SE3.SE3(x, y, z, joint_right)
    temp = SE3.SE3(0, 0, 0, 0)
    temp.reset(g.gmp)
    temp.mult_right(multiplier)
    return temp


def calc_beta(length, joint_left, joint_right):
    """"""
    x = (length * (cos(joint_right) - cos(joint_left))) * ONE_THIRD
    y = 0  # everything happens in the xz plane, so the y doesn't change
    z = (length * (sin(joint_right) + sin(joint_left))) * -ONE_THIRD  # signs are flipped because negative z is away from camera
    theta = (joint_right - joint_left) * ONE_THIRD
    return SE3.SE3(x, y, z, theta)


def system_body_velocity(length, alpha0, alpha1, alpha0_dot, alpha1_dot):
    """ calculates the body velocity for the CoM frame for the system"""
    half_l = length * 0.5
    row1 = [sin(alpha0), 0.4, cos(alpha1), (cos(alpha0) - 1) * half_l]
    row2 = [0, 0.4, 1, 0]
    row3 = [-sin(alpha1), 0.4, cos(alpha0), (cos(alpha1) + 1) * half_l]
    m = matrix([row1, row2, row3])
    omega_g = -1 * m.getI()
    row1 = [-half_l, 0]
    row2 = [0, 0]
    row3 = [0, half_l]
    omega_b = matrix([row1, row2, row3])
    alpha_dot = matrix([[alpha0_dot], [alpha1_dot]])
    omegas = omega_g * omega_b
    body_velocity = omegas * alpha_dot
    return body_velocity
    # return matrix([[body_velocity[0]], [0], [body_velocity[1]], [body_velocity[2]]])


def world_velocity(g_body_velocity, g):
    """ converts the body velocity of the CoM frame to its world velocity"""
    # we can drop into SE(2) to do these calculations in the x-z plane
    rho_gbv = matrix([[0, -1 * g_body_velocity.item(3), g_body_velocity.item(0)],
                      [g_body_velocity.item(3), 0, -g_body_velocity.item(2)],
                      [0, 0, 0]])
    g_se2_mat = xyz_to_xz(g.gmp.matrix, g.gmp.theta)
    rho_g_dot = g_se2_mat * rho_gbv
    theta = g_body_velocity.item(3)  # just add the thetas together
    g_dot = SE3.SE3(rho_g_dot.item((0, 2)), 0, rho_g_dot.item((1, 2)), theta)
    return g_dot
    # return cap_vel(g_dot)


def cap_vel(g_dot):
    if g_dot.x() > 0.3:
        x = 0.3
    elif g_dot.x() < -0.3:
        x = -0.3
    else:
        x = g_dot.x()

    if g_dot.z() > 0.3:
        z = 0.3
    elif g_dot.z() < -0.3:
        z = -0.3
    else:
        z = g_dot.z()

    return SE3.SE3(x, 0, z, g_dot.theta)


def xyz_to_xz(g_mat, theta):
    """ converts an SE3 matrix with 0 y component, to an SE2 matrix in the x-z plane"""
    row1 = [cos(theta), -1 * sin(theta), x_from_matrix(g_mat)]
    row2 = [sin(theta), cos(theta), -z_from_matrix(g_mat)]
    row3 = [0, 0, 1]
    return matrix([row1, row2, row3])

# this is dangerous because it requires arccos, which has a limited domain
# def angle_from_mat(m):
#     if m.item((0, 0)) == 0 and m.item((0, 1)) == 0:
#         return 0  # this happens when velocity is zero
#     trace = m.item((0, 0)) + m.item((1, 1)) + m.item((2, 2))
#     if trace > 3:
#         trace = 3  # technically dangerous, but it's just to account for rounding errors
#     theta = acos((trace - 1) * 0.5)
#     return theta




