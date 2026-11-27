"""在内存里修 UR5e articulation 拓扑并验证能不能写关节目标（不落盘）。"""
import os
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdPhysics

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ctx = omni.usd.get_context()
assert ctx.open_stage(os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda"))
stage = ctx.get_stage()

for side in ("Robot1Mount", "Robot2Mount"):
    base = "/World/Robot/%s/ur5e" % side
    rj = stage.GetPrimAtPath(base + "/root_joint")
    if rj and rj.HasAPI(UsdPhysics.ArticulationRootAPI):
        rj.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        print("remove API from", rj.GetPath())
    bl = stage.GetPrimAtPath(base + "/base_link")
    if bl and not bl.HasAPI(UsdPhysics.ArticulationRootAPI):
        UsdPhysics.ArticulationRootAPI.Apply(bl)
        print("apply API to", bl.GetPath())
    bad = stage.GetPrimAtPath(base + "/joints/robot_gripper_joint")
    if bad:
        bad.SetActive(False)
        print("deactivate", bad.GetPath())

import omni.timeline
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(30):
    app.update()

from isaacsim.core.prims import Articulation
for path in ("/World/Robot/Robot1Mount/ur5e",):
    a = Articulation(path)
    a.initialize()
    print("valid", a.is_physics_handle_valid())
    q0 = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
    print("===", path, "dof", a.num_dof, "q0", np.round(q0[:6], 3).tolist())
    a.set_joint_position_targets(q0.reshape(1, -1))
    for i in range(10):
        app.update()
    q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
    print("  after target=q0 finite", np.isfinite(q).all(), np.round(q[:6], 3).tolist())
    a.set_gains(kps=np.array([[1000.0, 1000.0, 800.0, 400.0, 300.0, 200.0]]),
                kds=np.array([[50.0, 50.0, 40.0, 20.0, 15.0, 10.0]]),
                joint_indices=list(range(6)))
    a.set_max_efforts(np.array([[150.0] * 6]), joint_indices=list(range(6)))
    for i in range(10):
        app.update()
    q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
    print("  after gains finite", np.isfinite(q).all(), np.round(q[:6], 3).tolist())
    qt = q.copy()
    qt[0] += 0.02
    a.set_joint_position_targets(qt.reshape(1, -1))
    for i in range(20):
        app.update()
    q2 = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
    print("  after move finite", np.isfinite(q2).all(), np.round(q2[:6], 3).tolist())

tl.stop()
app.close()
