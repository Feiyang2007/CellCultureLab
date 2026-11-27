"""隔离测试：UR5e 的 set_joint_position_targets 哪一步开始坏。"""
import os
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd
import omni.timeline

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ctx = omni.usd.get_context()
assert ctx.open_stage(os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda"))
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(30):
    app.update()

from isaacsim.core.prims import Articulation
a = Articulation("/World/Robot/Robot1Mount/ur5e")
a.initialize()


def show(tag, n=5):
    for _ in range(n):
        app.update()
    q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
    t = a.get_joint_position_targets()
    t = np.asarray(t, dtype=np.float64).reshape(-1) if t is not None else None
    print("%-18s q[:6] %s" % (tag, np.round(q[:6], 4).tolist()))
    print("%-18s tgt[:6] %s" % ("", np.round(t[:6], 4).tolist() if t is not None else None))


q0 = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
print("q0", np.round(q0, 4).tolist())
print("kps", np.asarray(a.get_gains()[0]).round(1).tolist()[:12])

print("--- 1) set targets = q0 ---")
a.set_joint_position_targets(q0.reshape(1, -1))
show("after target=q0", 5)

print("--- 2) set gains ---")
a.set_gains(kps=np.array([[1000.0, 1000.0, 800.0, 400.0, 300.0, 200.0]]),
            kds=np.array([[50.0, 50.0, 40.0, 20.0, 15.0, 10.0]]),
            joint_indices=list(range(6)))
a.set_max_efforts(np.array([[150.0] * 6]), joint_indices=list(range(6)))
show("after gains", 5)

print("--- 3) tiny move joint0 +0.01 ---")
q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
qt = q.copy()
qt[0] += 0.01
a.set_joint_position_targets(qt.reshape(1, -1))
show("after tiny move", 10)

print("--- 4) gripper 0.5 ---")
q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
qt = q.copy()
qt[6] = 0.5
a.set_joint_position_targets(qt.reshape(1, -1))
show("after grip", 10)

print("--- 5) set_joint_positions (teleport) q0 ---")
a.set_joint_positions(q0.reshape(1, -1))
show("after teleport", 5)

tl.stop()
app.close()
