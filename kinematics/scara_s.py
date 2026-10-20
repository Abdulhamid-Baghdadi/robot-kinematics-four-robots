"""s robot: SCARA-type arm (two revolute joints) whose third axis drives a wheel.

Notation: docs/notation/s-robot.png. The third joint rolls a wheel of radius R,
so its output is the travelled arc length z = R * theta3.
The IK derivation follows lfelipev/scara_trajectory_planning (scara_node.py).
"""
from math import acos, atan2, cos, degrees, pi, radians, sin, sqrt

L1 = 92.75
L2 = 70.72
R = 20.0  # wheel radius


def forward_kinematics(theta1, theta2, theta3):
    th1, th2 = radians(theta1), radians(theta2)
    x = L1 * sin(th1) + L2 * sin(th1 + th2)
    y = L1 * cos(th1) + L2 * cos(th1 + th2)
    return (x, y, R * radians(theta3))


def inverse_kinematics(x, y, z):
    r = sqrt(x ** 2 + y ** 2)
    try:
        alpha = acos((L1 ** 2 + L2 ** 2 - r ** 2) / (2 * L1 * L2))  # cosine rule
        theta2 = pi - alpha
        a, b = L2 * cos(theta2), L2 * sin(theta2)
        cos_beta = ((L1 + a) ** 2 + r ** 2 - b ** 2) / (2 * (L1 + a) * r)
        # bearing of the target from the y axis; the 2021 code used pi/2 - asin(y/r),
        # which is only correct for x >= 0 (targets left of the base came out wrong)
        theta1 = atan2(x, y) - acos(cos_beta)
    except (ValueError, ZeroDivisionError):
        raise ValueError("Point unreachable") from None
    return (degrees(theta1), degrees(theta2), degrees(z / R))


def joint_positions(theta1, theta2, theta3=0.0):
    """[base, elbow, tool] as (x, y)."""
    th1 = radians(theta1)
    x, y, _ = forward_kinematics(theta1, theta2, theta3)
    return [(0.0, 0.0), (L1 * sin(th1), L1 * cos(th1)), (x, y)]
