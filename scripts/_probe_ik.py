"""验证 UR5e 位置 IK 能不能把腕部拉到目标点（只读式：不改场景文件）。"""
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd
import omni.timeline
from pxr import Gf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _robot_ik import ArmFollower

ctx = omni.usd.get_context()
assert ctx.open_stage(os.path.join(PROJ, "assets", "scenes", "biolab_main_v10.usda"))
stage = ctx.get_stage()
print("bad joint active?", stage.GetPrimAtPath("/World/Robot/Robot1Mount/ur5e/joints/robot_gripper_joint").IsActive())
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(30):
    app.update()

arm = ArmFollower("/World/Robot/Robot1Mount/ur5e")
q, qd, p, quat = arm.state()
print("init eef", [round(v, 4) for v in p])
print("init quat", [round(v, 4) for v in quat])
print("dof", arm.n_dof, "link_idx", arm.link_idx)
print("gains kps", arm.art.get_gains()[0].round(2).tolist()[:12] if hasattr(arm.art, "get_gains") else "n/a")

targets = [
    (-1.60, -1.20, 1.30),
    (-1.60, -1.30, 1.20),
    (-1.70, -1.25, 1.10),
]
for ti, tgt in enumerate(targets):
    print("== target", ti, tgt)
    for i in range(90):
        arm.step(tgt, gripper=(0.8 if ti == 1 else 0.0))
        app.update()
        if i % 30 == 29:
            q, qd, p, quat = arm.state()
            print("  f%3d err %.4f eef %s q %s" % (
                i + 1, arm.last_err, [round(v, 3) for v in p],
                [round(float(q[j]), 3) for j in range(6)]))

q, qd, p, quat = arm.state()
print("final eef", [round(v, 4) for v in p])
print("final grip", arm.grip_target, "dof6..", [round(float(v), 3) for v in q[6:]])
tl.stop()
app.close()
