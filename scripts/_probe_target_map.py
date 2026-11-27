"""验证 set_joint_position_targets 的顺序/符号：一次只推一个关节。"""
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
a = Articulation("/World/Robot/Robot1Mount/ur5e")
a.initialize()
a.set_gains(kps=np.array([[1000.0] * 6]), kds=np.array([[50.0] * 6]),
            joint_indices=list(range(6)))
a.set_max_efforts(np.array([[150.0] * 6]), joint_indices=list(range(6)))
for _ in range(10):
    app.update()
q0 = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1).copy()
print("q0", np.round(q0[:6], 3).tolist())


def hold(tag, delta, n=25):
    a.set_joint_positions(q0.reshape(1, -1))
    for _ in range(5):
        app.update()
    qt = q0.copy()
    qt[delta[0]] += delta[1]
    a.set_joint_position_targets(qt.reshape(1, -1))
    for _ in range(n):
        app.update()
    q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
    print("%-18s target j%d %+0.2f -> q[:6] %s" % (
        tag, delta[0], delta[1], np.round(q[:6], 3).tolist()))


for j in range(6):
    hold("joint%d +0.1" % j, (j, 0.1))
for j in (1, 2):
    hold("joint%d -0.1" % j, (j, -0.1))

a.set_joint_positions(q0.reshape(1, -1))
for _ in range(5):
    app.update()
qt = q0.copy()
qt[6] = 0.5
a.set_joint_position_targets(qt.reshape(1, -1))
for _ in range(25):
    app.update()
q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
print("gripper 0.5 -> q[6:]", np.round(q[6:], 3).tolist())
tl.stop()
app.close()
