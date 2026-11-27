"""最小 IK 诊断：纯 +Z 10 cm，打印 dq / 实际 q / EEF，判断驱动是否跟指令。"""
import os
import sys
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd
import omni.timeline

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ctx = omni.usd.get_context()
assert ctx.open_stage(os.path.join(PROJ, "assets", "scenes", "biolab_main_v10.usda"))
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(30):
    app.update()

from _robot_ik import ArmFollower
arm = ArmFollower("/World/Robot/Robot1Mount/ur5e")
q0, qd0, p0, _ = arm.state()
print("p0", np.round(p0, 4).tolist(), "q0", np.round(q0[:6], 3).tolist())
target = p0 + np.array([0.0, 0.0, 0.10])
print("target", np.round(target, 4).tolist())

for i in range(80):
    q, qd, p, _ = arm.state()
    err = target - p
    J = np.asarray(arm.art.get_jacobians(), dtype=np.float64)
    Jw = J[0, arm.jac_idx, 0:3, :6]
    A = Jw @ Jw.T + arm.damping ** 2 * np.eye(3)
    dq = Jw.T @ np.linalg.solve(A, err)
    n = np.linalg.norm(dq)
    if n > arm.max_step:
        dq *= arm.max_step / n
    arm.step(target, 0.0)
    if i < 5:
        print("  set target q[:6]", np.round(arm.q_target[:6], 4).tolist())
    app.update()
    if i < 5:
        qa = np.asarray(arm.art.get_joint_positions(), dtype=np.float64).reshape(-1)
        print("  -> q[:6]", np.round(qa[:6], 4).tolist(), "eef", np.round(arm.state()[2], 4).tolist())
    if i % 20 == 19:
        q, qd, p, _ = arm.state()
        print("f%3d err %.4f eef %s\n     dq %s\n     q  %s\n     qd %s"
              % (i + 1, np.linalg.norm(target - p), np.round(p, 4).tolist(),
                 np.round(dq, 4).tolist(), np.round(q[:6], 3).tolist(),
                 np.round(qd[:6], 3).tolist()))
tl.stop()
app.close()
