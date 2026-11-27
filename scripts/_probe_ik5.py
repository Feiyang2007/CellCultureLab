"""看手臂是不是被碰撞/限位卡住：试竖直上举、同高小位移、高位目标，并打印 qd。"""
import os
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd
import omni.timeline

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
a.set_gains(kps=np.array([[1000.0] * 6]), kds=np.array([[50.0] * 6]),
            joint_indices=list(range(6)))
a.set_max_efforts(np.array([[150.0] * 6]), joint_indices=list(range(6)))
for _ in range(5):
    app.update()
print("gains now", np.asarray(a.get_gains()[0]).round(1).tolist()[:6])

targets = [
    ("up", (-1.90, -1.14, 1.55)),
    ("lateral", (-1.60, -1.05, 1.35)),
    ("high", (-1.60, -1.30, 1.45)),
]


def run(tag, target):
    a.set_joint_positions(q0.reshape(1, -1))
    for _ in range(10):
        app.update()
    target = np.array(target)
    for i in range(120):
        q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
        T = np.asarray(view.get_link_transforms(), dtype=np.float64)[0, 6]
        err = target - T[:3]
        J = np.asarray(a.get_jacobians(), dtype=np.float64)
        Jw = J[0, 6, 0:3, :6]
        A = Jw @ Jw.T + 0.08 ** 2 * np.eye(3)
        dq = Jw.T @ np.linalg.solve(A, err)
        n = np.linalg.norm(dq)
        if n > 0.02:
            dq *= 0.02 / n
        qt = q.copy()
        qt[:6] = q[:6] + dq
        a.set_joint_position_targets(qt.reshape(1, -1))
        app.update()
    q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
    qd = np.asarray(a.get_joint_velocities(), dtype=np.float64).reshape(-1)
    T = np.asarray(view.get_link_transforms(), dtype=np.float64)[0, 6]
    print("%-8s err %.4f eef %s qdmax %.3f q %s" % (
        tag, np.linalg.norm(target - T[:3]), np.round(T[:3], 3).tolist(),
        float(np.abs(qd[:6]).max()), np.round(q[:6], 2).tolist()))


for tag, t in targets:
    run(tag, t)
tl.stop()
app.close()
