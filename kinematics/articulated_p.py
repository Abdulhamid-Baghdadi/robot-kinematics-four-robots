"""p robot: 3-DOF articulated arm (base yaw + shoulder + elbow).

Notation: docs/notation/p-robot.png. theta2 is measured from the vertical and
theta3 carries a 90 deg offset, so (0, 0, 0) is the robot's zero pose
(upper arm vertical, forearm horizontal).
"""
from math import acos, asin, atan2, cos, degrees, pi, radians, sin, sqrt

L1 = 88.2  # base height
L2 = 120.23  # upper arm
L3 = 150.0  # forearm


def forward_kinematics(theta1, theta2, theta3):
    th1, th2, th3 = radians(theta1), radians(theta2), radians(theta3 + 90)
    r = L2 * sin(th2) + L3 * sin(th2 + th3)
    return (r * cos(th1), r * sin(th1), L2 * cos(th2) + L3 * cos(th2 + th3) + L1)


def inverse_kinematics(x, y, z):
    theta1 = atan2(y, x)
    r = sqrt(x ** 2 + y ** 2)
    D = sqrt((z - L1) ** 2 + r ** 2)
    try:
        alpha = acos((L3 ** 2 + L2 ** 2 - D ** 2) / (2 * L2 * L3))  # cosine rule
        theta3 = pi - alpha
        a, b = L3 * cos(theta3), L3 * sin(theta3)
        cos_beta = ((L2 + a) ** 2 + D ** 2 - b ** 2) / (2 * (L2 + a) * D)
        theta2 = pi / 2 - asin((z - L1) / D) - acos(cos_beta)
    except (ValueError, ZeroDivisionError):
        raise ValueError("Point unreachable") from None
    return (degrees(theta1), degrees(theta2), degrees(theta3) - 90)


def joint_positions(theta1, theta2, theta3):
    """[base, shoulder, elbow, tool] as (x, y, z)."""
    th1, th2 = radians(theta1), radians(theta2)
    r_e, z_e = L2 * sin(th2), L1 + L2 * cos(th2)
    return [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, L1),
        (r_e * cos(th1), r_e * sin(th1), z_e),
        forward_kinematics(theta1, theta2, theta3),
    ]
