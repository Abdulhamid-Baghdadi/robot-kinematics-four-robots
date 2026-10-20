# Robot Kinematics: Four Small Robots

Closed-form **forward and inverse kinematics** for the four robots of *Chocko-station*, a 2021 group project in a mechatronics bachelor's course (group F): a small ROS-controlled station in which four servo-driven robots pick, move and place chocolate bars.

This repository is the kinematics part of that project, extracted and ported to Python 3, with round-trip tests and an animation for each robot.

| 2d robot: five-bar | 3d robot: delta | p robot: articulated arm | s robot: SCARA |
|:---:|:---:|:---:|:---:|
| ![2d](docs/images/2d-robot.gif) | ![3d](docs/images/3d-robot.gif) | ![p](docs/images/p-robot.gif) | ![s](docs/images/s-robot.gif) |

*In each animation the tool point follows a path (red ✕), the IK computes the joint angles and the FK draws the chain.*

## The four robots

| Robot | Type | Joints → output | Module |
|---|---|---|---|
| **2d** | planar five-bar parallel linkage | 2 base servos → (x, y) | [`five_bar_2d.py`](kinematics/five_bar_2d.py) |
| **3d** | delta (3 parallel arms) | 3 base servos → (x, y, z) | [`delta_3d.py`](kinematics/delta_3d.py) |
| **p** | articulated arm (yaw, shoulder, elbow) | 3 servos → (x, y, z) | [`articulated_p.py`](kinematics/articulated_p.py) |
| **s** | SCARA, two revolute joints + wheel axis | 2 servos + wheel → (x, y, z) | [`scara_s.py`](kinematics/scara_s.py) |

Angles are in degrees, lengths in mm. Every module has `forward_kinematics`, `inverse_kinematics` (raises `ValueError` for unreachable points) and `joint_positions` (for plotting).

### 2d robot: five-bar linkage

<img src="docs/notation/2d-robot-fk.png" width="45%"> <img src="docs/notation/2d-robot-ik.png" width="45%">

Servos P0, P1 at (∓L0/2, 0), cranks L1 = 100, distal links L2 = 180, L0 = 78.08.

- **FK:** crank tips P2 = (−L1 sin θ0 − L0/2, L1 cos θ0), P3 = (L1 sin θ1 + L0/2, L1 cos θ1). With l23 = |P3 − P2| and α the angle of P2→P3, the tool is P4 = P2 + L2 (cos(α+δ), sin(α+δ)) with δ = ±acos(l23 / 2L2) (the sign picks the assembly mode).
- **IK:** each side on its own. Left: l04 = |P4 − P0|, α the angle of P0→P4 from the vertical, β from the cosine rule in the triangle (L1, L2, l04): β = acos((L1² − L2² + l04²) / (2 L1 l04)), so θ0 = β − α. The right side works the same way (γ, δ), elbows out.

### 3d robot: delta

<img src="docs/notation/3d-robot-delta.png" width="70%">

Base triangle f = 178.78, effector triangle e = 69.28, upper arms rf = 96.1, forearms re = 150.

- **IK:** each arm is solved in its own YZ plane: the elbow J1 is where the circle of the upper arm (radius rf around F1) meets the sphere of the forearm (radius re around the shifted effector joint E1′), giving θ = atan(−zJ / (yF − yJ)). Arms 2 and 3 use the same function after rotating the target by ±120°.
- **FK:** the effector is where three spheres (radius re, centred at the elbows shifted inward by the effector radius) meet. Eliminating x and y gives a quadratic a z² + b z + c = 0, and the lower root is taken.

The derivation follows the well-known *Delta robot kinematics* tutorial from the Trossen Robotics community, applied to this robot's dimensions.

### p robot: articulated arm

<img src="docs/notation/p-robot.png" width="70%">

Base height L1 = 88.2, upper arm L2 = 120.23, forearm L3 = 150. θ2 is measured from the vertical; θ3 has a +90° offset so that (0, 0, 0) is the zero pose (upper arm up, forearm horizontal).

- **FK:** r = L2 sin θ2 + L3 sin(θ2 + θ3′), x = r cos θ1, y = r sin θ1, z = L1 + L2 cos θ2 + L3 cos(θ2 + θ3′), with θ3′ = θ3 + 90°.
- **IK:** θ1 = atan2(y, x). In the arm plane D = √(r² + (z − L1)²); the cosine rule gives the elbow α = acos((L2² + L3² − D²) / 2L2L3), θ3′ = π − α, and the shoulder is θ2 = π/2 − asin((z − L1)/D) − β, with β the angle between D and the upper arm.

### s robot: SCARA

<img src="docs/notation/s-robot.png" width="70%">

L1 = 92.75, L2 = 70.72; the third axis drives a wheel of radius R = 20, so z = R θ3 is the distance rolled.

- **FK:** x = L1 sin θ1 + L2 sin(θ1 + θ2), y = L1 cos θ1 + L2 cos(θ1 + θ2), z = R θ3.
- **IK:** r = √(x² + y²); cosine rule: θ2 = π − acos((L1² + L2² − r²) / 2L1L2); θ1 = atan2(x, y) − β, where β is the angle between r and L1. The IK layout follows [lfelipev/scara_trajectory_planning](https://github.com/lfelipev/scara_trajectory_planning).

## Run it

```bash
pip install -r requirements.txt
python -m pytest -q                 # IK(FK(q)) == q for all four robots
python examples/roundtrip_demo.py   # one pose per robot, FK -> IK -> FK
python examples/animate.py          # re-renders the GIFs in docs/images
```

```
2d  five-bar  q=(  20.000,   35.000) deg -> p=(  22.823,  246.191) mm -> IK q=(  20.000,   35.000)  |dq|max=1.4e-14
3d  delta     q=(  10.000,   25.000,   40.000) deg -> p=(  17.121,  -27.281, -128.096) mm -> IK q=(  10.000,   25.000,   40.000)  |dq|max=1.4e-14
p   articul.  q=(  30.000,   20.000,  -15.000) deg -> p=( 165.021,   95.275,  188.106) mm -> IK q=(  30.000,   20.000,  -15.000)  |dq|max=3.6e-15
s   SCARA     q=( -40.000,   70.000,   90.000) deg -> p=( -24.259,  132.296,   31.416) mm -> IK q=( -40.000,   70.000,   90.000)  |dq|max=2.8e-14
```

The tests sample 500 random joint vectors per robot. Each closed-form IK returns one solution branch (five-bar elbows out, p-robot elbow in [−90°, 90°] reaching forward, SCARA elbow in (0°, 180°)), so the samples are limited to that branch.

## Changes from the 2021 code

- Ported from Python 2 / ROS Indigo to plain Python 3 with no ROS dependency. Plotting moved out of the kinematics modules.
- **Two bugs found by the new round-trip tests and fixed:**
  - *2d robot IK* used `atan(y/dx)`, which is 180° off for targets outside the span between the two servos. It now uses `atan2`, which gives the same result inside that span.
  - *s robot IK* used `π/2 − asin(y/r)` as the target bearing, which is only correct for x ≥ 0. It now uses `atan2(x, y)`.
- The equations and the robot dimensions are unchanged.

## Team & my contribution

Chocko-station was built by a team (group F). The full project included the ROS nodes, a teaching GUI, a camera, pumps, Arduino firmware and the station itself. It stays in a private repository. Teammate **Moritz Abdank** wrote a large share of the overall project (about 63 % of all commits).

**My part (Abdulhamid Baghdadi):** the robot kinematics in this repository. According to the git history, I wrote all four kinematics files and the four notation slide decks. I worked out the five-bar and articulated-arm models, adapted published delta and SCARA solutions to our geometry, and corrected the zero points during commissioning. In this extraction I also wrote the round-trip tests, the animations and the two bug fixes above.
