"""Print FK -> IK -> FK for one pose of each robot.  Run: python examples/roundtrip_demo.py"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from kinematics import articulated_p, delta_3d, five_bar_2d, scara_s  # noqa: E402

POSES = [
    ("2d  five-bar ", five_bar_2d, (20.0, 35.0)),
    ("3d  delta    ", delta_3d, (10.0, 25.0, 40.0)),
    ("p   articul. ", articulated_p, (30.0, 20.0, -15.0)),
    ("s   SCARA    ", scara_s, (-40.0, 70.0, 90.0)),
]


def fmt(v):
    return "(" + ", ".join(f"{x:8.3f}" for x in v) + ")"


for name, robot, q in POSES:
    p = robot.forward_kinematics(*q)
    q_back = robot.inverse_kinematics(*p)
    err = max(abs(a - b) for a, b in zip(q, q_back))
    print(f"{name} q={fmt(q)} deg -> p={fmt(p)} mm -> IK q={fmt(q_back)}  |dq|max={err:.1e}")
