"""Forward and inverse kinematics for the four robots of the Chocko-station (2021).

Each module exposes ``forward_kinematics``, ``inverse_kinematics`` and
``joint_positions`` (the points of the kinematic chain, used for plotting).
Angles are in degrees, lengths in millimetres. Unreachable poses raise ValueError.
"""
from . import articulated_p, delta_3d, five_bar_2d, scara_s

ROBOTS = {
    "2d": five_bar_2d,
    "3d": delta_3d,
    "p": articulated_p,
    "s": scara_s,
}
