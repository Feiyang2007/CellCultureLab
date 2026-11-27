"""诊断：哪个 prim 才是 PhysX 认的 articulation 根，关节读数是否有效。"""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd
import omni.timeline
import numpy as np

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ctx = omni.usd.get_context()
assert ctx.open_stage(os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda"))
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(30):
    app.update()

from isaacsim.core.prims import Articulation

cands = [
    "/World/Robot/Robot1Mount/ur5e",
    "/World/Robot/Robot1Mount/ur5e/root_joint",
    "/World/Robot/Robot1Mount/ur5e/base_link",
    "/World/Robot/Robot1Mount",
]
for path in cands:
    print("===", path)
    try:
        a = Articulation(path)
        a.initialize()
        print("  valid", a.is_physics_handle_valid(), "n_dof", a.num_dof)
        q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
        qd = np.asarray(a.get_joint_velocities(), dtype=np.float64).reshape(-1)
        print("  q finite", np.isfinite(q).all(), "q[:12]", np.round(q[:12], 3).tolist())
        print("  qd finite", np.isfinite(qd).all(), "qd max", float(np.abs(qd).max()) if qd.size else None)
        J = a.get_jacobians()
        if J is not None:
            J = np.asarray(J, dtype=np.float64)
            print("  J shape", J.shape, "finite", np.isfinite(J).all(), "max", float(np.abs(J).max()))
    except Exception as e:
        print("  FAILED", repr(e)[:300])

tl.stop()
app.close()
