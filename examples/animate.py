"""Render one GIF per robot: the tool follows a path, IK gives the joints, FK draws the chain.

Run: python examples/animate.py   (writes docs/images/<robot>.gif)
"""
import os
import sys
from math import cos, pi, sin

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT)

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

from kinematics import articulated_p, delta_3d, five_bar_2d, scara_s  # noqa: E402

OUT = os.path.join(ROOT, "docs", "images")
FRAMES = 60
LINK, JOINT, TRACE, TARGET = "#1f4e79", "#e07b00", "#9bbbd4", "#c0392b"


def circle(cx, cy, r, k, z=None):
    a = 2 * pi * k / FRAMES
    pt = (cx + r * cos(a), cy + r * sin(a))
    return pt if z is None else pt + (z,)


def save(fig, update, name):
    anim = FuncAnimation(fig, update, frames=FRAMES, blit=False)
    anim.save(os.path.join(OUT, name), writer=PillowWriter(fps=20), dpi=80)
    plt.close(fig)
    print("wrote", name)


def planar(name, title, path, ik, chain, lim):
    fig, ax = plt.subplots(figsize=(5, 5))
    trace = []

    def update(k):
        target = path(k)
        q = ik(*target)
        pts = chain(*q)
        trace.append(target[:2])
        ax.clear()
        ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1]); ax.set_aspect("equal")
        ax.grid(alpha=0.3); ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm]")
        ax.plot(*zip(*trace), color=TRACE, lw=1.5)
        ax.plot(*zip(*pts), "-o", color=LINK, lw=3, mfc=JOINT, mec=JOINT, ms=7)
        ax.plot(*target[:2], "x", color=TARGET, ms=10, mew=2)
        ax.set_title(title + "\n" + ", ".join(f"{a:.0f}°" for a in q[:2]), fontsize=10)

    save(fig, update, name)


def spatial(name, title, path, ik, chains, lim, view=(25, -60)):
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, projection="3d")
    trace = []

    def update(k):
        target = path(k)
        q = ik(*target)
        trace.append(target)
        ax.clear()
        ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1]); ax.set_zlim(*lim[2])
        ax.view_init(*view)
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
        ax.plot(*zip(*trace), color=TRACE, lw=1.5)
        for pts in chains(*q):
            ax.plot(*zip(*pts), "-o", color=LINK, lw=3, mfc=JOINT, mec=JOINT, ms=5)
        ax.plot(*[[c] for c in target], "x", color=TARGET, ms=10, mew=2)
        ax.set_title(title + "\n" + ", ".join(f"{a:.0f}°" for a in q), fontsize=10)

    save(fig, update, name)


def delta_chains(*q):
    arms = delta_3d.joint_positions(*q)
    base = [a[0] for a in arms] + [arms[0][0]]
    eff = [a[2] for a in arms] + [arms[0][2]]
    return arms + [base, eff]


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    planar("2d-robot.gif", "2d robot (five-bar)",
           lambda k: circle(0, 190, 45, k), five_bar_2d.inverse_kinematics,
           five_bar_2d.joint_positions, ((-160, 160), (-40, 280)))
    planar("s-robot.gif", "s robot (SCARA)",
           lambda k: circle(0, 100, 45, k, 0.0), scara_s.inverse_kinematics,
           scara_s.joint_positions, ((-170, 170), (-90, 180)))
    spatial("p-robot.gif", "p robot (articulated)",
            lambda k: circle(150, 0, 60, k, 120 + 40 * sin(4 * pi * k / FRAMES)),
            articulated_p.inverse_kinematics,
            lambda *q: [articulated_p.joint_positions(*q)],
            ((-50, 250), (-150, 150), (0, 300)))
    spatial("3d-robot.gif", "3d robot (delta)",
            lambda k: circle(0, 0, 40, k, -170 + 15 * sin(4 * pi * k / FRAMES)),
            delta_3d.inverse_kinematics, delta_chains,
            ((-150, 150), (-150, 150), (-220, 20)), view=(15, -60))
