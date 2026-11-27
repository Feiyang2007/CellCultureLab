"""用有限差分标定 PhysX 雅可比：link 索引、行序、是否需要按 base 姿态旋转。"""
import os
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd
import omni.timeline
from pxr import Gf, UsdGeom

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ctx = omni.usd.get_context()
assert ctx.open_stage(os.path.join(PROJ, "assets", "scenes", "biolab_main_v10.usda"))
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(30):
    app.update()

from isaacsim.core.prims import Articulation
a = Articulation("/World/Robot/Robot1Mount/ur5e")
a.initialize()
view = a._physics_view
a.set_gains(kps=np.array([[2000.0] * 6]), kds=np.array([[100.0] * 6]),
            joint_indices=list(range(6)))
a.set_max_efforts(np.array([[150.0] * 6]), joint_indices=list(range(6)))
for _ in range(5):
    app.update()


def link_pose(idx):
    T = np.asarray(view.get_link_transforms(), dtype=np.float64)[0, idx]
    return T[:3], T[3:7]


q0 = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1).copy()
T0 = np.asarray(view.get_link_transforms(), dtype=np.float64)[0]
print("base pos", np.round(T0[0, :3], 3).tolist(), "quat xyzw", np.round(T0[0, 3:7], 3).tolist())
p0, _ = link_pose(6)
print("wrist0", np.round(p0, 4).tolist())

num = np.zeros((3, 6))
for j in range(6):
    q = q0.copy()
    q[j] += 0.02
    a.set_joint_positions(q.reshape(1, -1))
    for _ in range(3):
        app.update()
    pj, _ = link_pose(6)
    num[:, j] = (pj - p0) / 0.02
    a.set_joint_positions(q0.reshape(1, -1))
    for _ in range(3):
        app.update()
print("numeric dEEF/dq (wrist, world):")
print(np.round(num, 3))

J = np.asarray(a.get_jacobians(), dtype=np.float64)[0]
for idx in (5, 6):
    for rows, tag in ((slice(0, 3), "rows0:3"), (slice(3, 6), "rows3:6")):
        print("J link%d %s:" % (idx, tag))
        print(np.round(J[idx, rows, :6], 3))

# base 旋转
q_base = T0[0, 3:7]
x, y, z, w = q_base
R = np.array([
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
])
for idx in (5, 6):
    print("Rbase @ J link%d rows0:3:" % idx)
    print(np.round(R @ J[idx, 0:3, :6], 3))
tl.stop()
app.close()
