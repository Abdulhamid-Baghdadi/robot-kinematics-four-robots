"""IK(FK(q)) == q for every robot, over random joint values inside each working range."""
import math
import random

import pytest

from kinematics import articulated_p, delta_3d, five_bar_2d, scara_s

N = 500
TOL = 1e-6

# Joint ranges sampled for each robot. Every closed-form IK returns one branch
# (elbow up/down, elbows out ...); BRANCH filters samples to that branch.
RANGES = {
    five_bar_2d: [(-60, 90), (-60, 90)],
    delta_3d: [(-30, 80)] * 3,
    articulated_p: [(-180, 180), (-80, 80), (-89, 89)],
    scara_s: [(-120, 120), (1, 179), (-360, 360)],
}


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _five_bar_elbows_out(q):
    p0, p2, p4, p3, p1 = five_bar_2d.joint_positions(*q)
    return _cross(p0, p4, p2) > 0 and _cross(p1, p4, p3) < 0


def _p_reaches_forward(q):
    th2, th3 = math.radians(q[1]), math.radians(q[2] + 90)
    return articulated_p.L2 * math.sin(th2) + articulated_p.L3 * math.sin(th2 + th3) > 1


BRANCH = {
    five_bar_2d: _five_bar_elbows_out,
    articulated_p: _p_reaches_forward,  # otherwise (theta1 + 180, ...) is the same pose
}


def _samples(module, rng):
    keep = BRANCH.get(module, lambda q: True)
    while True:
        q = tuple(rng.uniform(lo, hi) for lo, hi in RANGES[module])
        try:
            p = module.forward_kinematics(*q)
        except ValueError:
            continue  # not assemblable (five-bar / delta), draw again
        if keep(q):
            yield q, p


@pytest.mark.parametrize("module", list(RANGES), ids=lambda m: m.__name__.split(".")[-1])
def test_ik_inverts_fk(module):
    rng = random.Random(2021)
    gen = _samples(module, rng)
    for _ in range(N):
        q, p = next(gen)
        q_back = module.inverse_kinematics(*p)
        wrapped = [(b - a + 180) % 360 - 180 for a, b in zip(q, q_back)]  # angles mod 360
        assert wrapped == pytest.approx([0] * len(q), abs=TOL), (q, p, q_back)


@pytest.mark.parametrize("module, point", [
    (five_bar_2d, (0, 400)),
    (delta_3d, (0, 0, -400)),
    (articulated_p, (500, 0, 88.2)),
    (scara_s, (200, 0, 0)),
])
def test_unreachable_raises(module, point):
    with pytest.raises(ValueError):
        module.inverse_kinematics(*point)


def test_zero_poses():
    assert articulated_p.forward_kinematics(0, 0, 0) == pytest.approx(
        (articulated_p.L3, 0, articulated_p.L1 + articulated_p.L2))
    assert scara_s.forward_kinematics(0, 0, 0) == pytest.approx((0, scara_s.L1 + scara_s.L2, 0))
    x, y = five_bar_2d.forward_kinematics(0, 0)
    assert x == pytest.approx(0) and y > five_bar_2d.L1
    x, y, z = delta_3d.forward_kinematics(0, 0, 0)
    assert math.hypot(x, y) == pytest.approx(0, abs=1e-9) and z < 0
