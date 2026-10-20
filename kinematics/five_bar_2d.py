"""2d robot: planar five-bar parallel linkage driven by two base servos.

Notation (docs/notation/2d-robot-*.png): P0 and P1 are the servos, L0 apart;
L1 are the driven cranks, L2 the two equal distal links meeting at the tool point P4.
In code the distal link length is called L3/L4 (as in the 2021 source).
"""
from math import acos, atan2, cos, degrees, pi, radians, sin, sqrt

L0 = 78.08  # distance between the two servos
L1 = 100.0  # crank length
L3 = 180.0  # distal link length (both distal links are equal)
L4 = L3


def _elbows(theta0, theta1):
    xa = -L1 * sin(radians(theta0)) - L0 / 2  # left crank tip (P2)
    ya = L1 * cos(radians(theta0))
    xb = L1 * sin(radians(theta1)) + L0 / 2  # right crank tip (P3)
    yb = L1 * cos(radians(theta1))
    return (xa, ya), (xb, yb)


def forward_kinematics(theta0, theta1, delta_sign=+1):
    """(theta0, theta1) -> (x, y). delta_sign picks the assembly mode."""
    (xa, ya), (xb, yb) = _elbows(theta0, theta1)
    l23 = sqrt((xb - xa) ** 2 + (yb - ya) ** 2)
    alpha = atan2(yb - ya, xb - xa)
    try:
        delta = delta_sign * acos(l23 / (2 * L4))
    except ValueError:
        raise ValueError("Point unreachable") from None
    return (xa + L4 * cos(alpha + delta), ya + L4 * sin(alpha + delta))


def inverse_kinematics(x, y):
    """(x, y) -> (theta0, theta1), elbows-out solution."""
    try:
        l04 = sqrt((x + L0 / 2) ** 2 + y ** 2)
        # atan2 instead of the 2021 atan(y/dx): same result between the servos, but
        # targets outside the servo span were 180 deg off
        alpha = pi / 2 - atan2(y, x + L0 / 2)
        beta = acos((L1 * L1 - L4 * L4 + l04 * l04) / (2 * L1 * l04))
        theta0 = degrees(beta - alpha)

        l14 = sqrt((x - L0 / 2) ** 2 + y ** 2)
        gamma = atan2(y, x - L0 / 2) - pi / 2
        delta = acos((L1 * L1 - L4 * L4 + l14 * l14) / (2 * L1 * l14))
        theta1 = degrees(delta - gamma)
    except (ValueError, ZeroDivisionError):
        raise ValueError("Point unreachable") from None
    return (theta0, theta1)


def joint_positions(theta0, theta1, delta_sign=+1):
    """Closed chain P0 -> P2 -> P4 -> P3 -> P1 as a list of (x, y)."""
    p2, p3 = _elbows(theta0, theta1)
    p4 = forward_kinematics(theta0, theta1, delta_sign)
    return [(-L0 / 2, 0.0), p2, p4, p3, (L0 / 2, 0.0)]
