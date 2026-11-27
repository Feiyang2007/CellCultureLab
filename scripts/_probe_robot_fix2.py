"""变体测试：只停用坏关节（不动 articulation 根），看 Robot1 能否写目标。

用法：python _probe_robot_fix2.py [a|b|c]
  a = 保留 root_joint 的 API，只 active=false 坏关节
  b = API 移到 base_link，保留坏关节
  c = API 移到 ur5e Xform，active=false 坏关节
"""
import os
import sys
import numpy as np
from isaacsim import SimulationApp

variant = (sys.argv[1] if len(sys.argv) > 1 else "a").lower()
app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdPhysics

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ctx = omni.usd.get_context()
assert ctx.open_stage(os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda"))
stage = ctx.get_stage()

base = "/World/Robot/Robot1Mount/ur5e"
if variant in ("b", "c"):
    rj = stage.GetPrimAtPath(base + "/root_joint")
    if rj and rj.HasAPI(UsdPhysics.ArticulationRootAPI):
        rj.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    target = base + "/base_link" if variant == "b" else base
    p = stage.GetPrimAtPath(target)
    UsdPhysics.ArticulationRootAPI.Apply(p)
if variant in ("a", "c"):
    bad = stage.GetPrimAtPath(base + "/joints/robot_gripper_joint")
    if bad:
        bad.SetActive(False)

import omni.timeline
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(30):
    app.update()

from isaacsim.core.prims import Articulation
a = Articulation(base)
a.initialize()
q0 = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
print("variant", variant, "valid", a.is_physics_handle_valid(), "dof", a.num_dof)
print("q0", np.round(q0[:6], 3).tolist())
a.set_joint_position_targets(q0.reshape(1, -1))
for _ in range(10):
    app.update()
q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
print("after target=q0", np.round(q[:6], 3).tolist())
a.set_gains(kps=np.array([[1000.0, 1000.0, 800.0, 400.0, 300.0, 200.0]]),
            kds=np.array([[50.0, 50.0, 40.0, 20.0, 15.0, 10.0]]),
            joint_indices=list(range(6)))
a.set_max_efforts(np.array([[150.0] * 6]), joint_indices=list(range(6)))
for _ in range(10):
    app.update()
q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
print("after gains", np.round(q[:6], 3).tolist())
qt = q.copy()
qt[0] += 0.05
a.set_joint_position_targets(qt.reshape(1, -1))
for _ in range(30):
    app.update()
q2 = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
print("after move", np.round(q2[:6], 3).tolist())
tl.stop()
app.close()
