"""3d robot: three-arm delta (parallel) robot.

Closed-form solution after the well-known delta-robot derivation (Trossen Robotics
community tutorial "Delta robot kinematics"), adapted to this robot's geometry.
Notation: docs/notation/3d-robot-delta.png. z points up, so the effector has z < 0.
"""
from math import atan, cos, degrees, pi, radians, sin, sqrt

E = 69.28  # end-effector triangle side
F = 178.78  # base triangle side
RE = 150.0  # forearm (parallelogram) length
RF = 96.1  # upper-arm length

SQRT3 = sqrt(3.0)
SIN120, COS120 = SQRT3 / 2.0, -0.5
TAN60, SIN30, TAN30 = SQRT3, 0.5, 1 / SQRT3
ARM_ANGLES = (-pi / 2, pi / 6, 5 * pi / 6)  # direction of arms 1, 2, 3 in the xy plane


def _elbows(theta1, theta2, theta3):
    """Elbow points shifted towards the centre by the effector radius (sphere centres)."""
    t = (F - E) * TAN30 / 2
    th1, th2, th3 = radians(theta1), radians(theta2), radians(theta3)
    y1 = -(t + RF * cos(th1))
    z1 = -RF * sin(th1)
    y2 = (t + RF * cos(th2)) * SIN30
    x2 = y2 * TAN60
    z2 = -RF * sin(th2)
    y3 = (t + RF * cos(th3)) * SIN30
    x3 = -y3 * TAN60
    z3 = -RF * sin(th3)
    return (0.0, y1, z1), (x2, y2, z2), (x3, y3, z3)


def forward_kinematics(theta1, theta2, theta3):
    """(theta1, theta2, theta3) -> (x, y, z): intersection of three spheres."""
    (_, y1, z1), (x2, y2, z2), (x3, y3, z3) = _elbows(theta1, theta2, theta3)
    dnm = (y2 - y1) * x3 - (y3 - y1) * x2
    w1 = y1 * y1 + z1 * z1
    w2 = x2 * x2 + y2 * y2 + z2 * z2
    w3 = x3 * x3 + y3 * y3 + z3 * z3
    # x = (a1*z + b1)/dnm
    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0
    # y = (a2*z + b2)/dnm
    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0
    # a*z^2 + b*z + c = 0
    a = a1 * a1 + a2 * a2 + dnm * dnm
    b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
    c = (b2 - y1 * dnm) ** 2 + b1 * b1 + dnm * dnm * (z1 * z1 - RE * RE)
    d = b * b - 4.0 * a * c
    if d < 0:
        raise ValueError("Point unreachable")
    z0 = -0.5 * (b + sqrt(d)) / a
    return ((a1 * z0 + b1) / dnm, (a2 * z0 + b2) / dnm, z0)


def _angle_yz(x0, y0, z0):
    """Angle of one arm, solved in its own YZ plane."""
    y1 = -0.5 * TAN30 * F  # base joint
    y0 -= 0.5 * TAN30 * E  # shift centre to edge
    a = (x0 * x0 + y0 * y0 + z0 * z0 + RF * RF - RE * RE - y1 * y1) / (2 * z0)
    b = (y1 - y0) / z0
    d = -(a + b * y1) ** 2 + RF * (b * b * RF + RF)
    if d < 0:
        raise ValueError("Point unreachable")
    yj = (y1 - a * b - sqrt(d)) / (b * b + 1)  # outer solution
    zj = a + b * yj
    return degrees(atan(-zj / (y1 - yj)) + (pi if yj > y1 else 0.0))


def inverse_kinematics(x, y, z):
    """(x, y, z) -> (theta1, theta2, theta3); arms 2 and 3 via a +-120 deg rotation."""
    return (
        _angle_yz(x, y, z),
        _angle_yz(x * COS120 + y * SIN120, y * COS120 - x * SIN120, z),
        _angle_yz(x * COS120 - y * SIN120, y * COS120 + x * SIN120, z),
    )


def joint_positions(theta1, theta2, theta3):
    """For each arm: [base joint, elbow, effector joint] as (x, y, z), for plotting."""
    x, y, z = forward_kinematics(theta1, theta2, theta3)
    rb, re_ = F * TAN30 / 2, E * TAN30 / 2  # in-radius of base / effector triangle
    arms = []
    for ang, th in zip(ARM_ANGLES, (theta1, theta2, theta3)):
        u = (cos(ang), sin(ang))
        reach = rb + RF * cos(radians(th))
        arms.append([
            (rb * u[0], rb * u[1], 0.0),
            (reach * u[0], reach * u[1], -RF * sin(radians(th))),
            (x + re_ * u[0], y + re_ * u[1], z),
        ])
    return arms
