"""判定雅可比的行序与 link 索引：四种组合各跑一次 IK，看哪个收敛。"""
import os
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd
import omni.timeline
from pxr import Usd

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ctx = omni.usd.get_context()
assert ctx.open_stage(os.path.join(PROJ, "assets", "scenes", "biolab_main_v10.usda"))
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(30):
    app.update()

from isaacsim.core.prims import Articulation
path = "/World/Robot/Robot1Mount/ur5e"
a = Articulation(path)
a.initialize()
view = a._physics_view
q0 = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
a.set_gains(kps=np.array([[1000.0] * 6]),
            kds=np.array([[50.0] * 6]), joint_indices=list(range(6)))
a.set_max_efforts(np.array([[150.0] * 6]), joint_indices=list(range(6)))
for _ in range(5):
    app.update()

print("body names", list(a.body_names)[:18])
print("link idx wrist", a.get_link_index("wrist_3_link"))

target = np.array([-1.60, -1.30, 1.20])


def run(link_idx, rows, tag):
    a.set_joint_positions(q0.reshape(1, -1))
    for _ in range(10):
        app.update()
    for i in range(120):
        q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
        T = np.asarray(view.get_link_transforms(), dtype=np.float64)[0, link_idx]
        err = target - T[:3]
        J = np.asarray(a.get_jacobians(), dtype=np.float64)
        Jw = J[0, link_idx, rows, :6]
        A = Jw @ Jw.T + 0.08 ** 2 * np.eye(3)
        dq = Jw.T @ np.linalg.solve(A, err)
        n = np.linalg.norm(dq)
        if n > 0.02:
            dq *= 0.02 / n
        qt = q.copy()
        qt[:6] = q[:6] + dq
        a.set_joint_position_targets(qt.reshape(1, -1))
        app.update()
        if i % 40 == 39:
            T = np.asarray(view.get_link_transforms(), dtype=np.float64)[0, link_idx]
            print("%-10s f%3d err %.4f eef %s" % (tag, i + 1, np.linalg.norm(target - T[:3]),
                                                  np.round(T[:3], 3).tolist()))


for link_idx in (6, 5):
    for rows, tag in ((slice(0, 3), "lin"), (slice(3, 6), "ang")):
        run(link_idx, rows, "L%d-%s" % (link_idx, tag))

tl.stop()
app.close()
