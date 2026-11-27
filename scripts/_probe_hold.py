"""确认手臂驱动到底有没有力矩：读回 max effort + 保持初始位 120 帧。"""
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
q0 = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
print("q0", np.round(q0[:6], 4).tolist())
try:
    me = np.asarray(a.get_max_efforts(), dtype=np.float64).reshape(-1)
    print("max_efforts before", np.round(me[:6], 2).tolist())
except Exception as e:
    print("get_max_efforts failed", repr(e)[:200])
a.set_gains(kps=np.array([[1000.0] * 6]), kds=np.array([[50.0] * 6]),
            joint_indices=list(range(6)))
a.set_max_efforts(np.array([[150.0] * 6]), joint_indices=list(range(6)))
print("gains now", np.asarray(a.get_gains()[0]).round(1).tolist()[:6])
try:
    me = np.asarray(a.get_max_efforts(), dtype=np.float64).reshape(-1)
    print("max_efforts after", np.round(me[:6], 2).tolist())
except Exception as e:
    print("get_max_efforts(2) failed", repr(e)[:200])

a.set_joint_position_targets(q0.reshape(1, -1))
for i in range(120):
    app.update()
    if i % 40 == 39:
        q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
        print("hold f%3d %s" % (i + 1, np.round(q[:6], 4).tolist()))
tl.stop()
app.close()
