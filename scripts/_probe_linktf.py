"""查 PhysX articulation view 能不能直接给 link transforms（避免另建 RigidPrim 视图）。"""
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
print("view type", type(view))
for name in ("get_link_transforms", "get_link_poses", "get_world_transforms",
             "get_articulation_body_transforms", "get_body_transforms"):
    print(name, hasattr(view, name))
q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
print("q0", np.round(q[:6], 3).tolist())
if hasattr(view, "get_link_transforms"):
    try:
        t = view.get_link_transforms()
        print("link transforms shape", np.asarray(t).shape)
        arr = np.asarray(t)
        print("sample link 6", arr[0, 6, :7] if arr.ndim == 3 else arr[0, :7])
    except Exception as e:
        print("get_link_transforms failed", repr(e)[:300])

# 再试一次设增益 + 写目标，确认不另建视图时是否稳定
a.set_gains(kps=np.array([[1000.0, 1000.0, 800.0, 400.0, 300.0, 200.0]]),
            kds=np.array([[50.0, 50.0, 40.0, 20.0, 15.0, 10.0]]),
            joint_indices=list(range(6)))
for _ in range(5):
    app.update()
q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
print("after gains", np.round(q[:6], 4).tolist())
q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
qt = q.copy()
qt[0] += 0.02
a.set_joint_position_targets(qt.reshape(1, -1))
for _ in range(20):
    app.update()
q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
print("after move", np.round(q[:6], 4).tolist())
tl.stop()
app.close()
