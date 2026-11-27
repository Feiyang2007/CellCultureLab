"""验证 tl.stop()/play() 之后 articulation 张量视图的 link transforms 是否变旧。"""
import os
import sys
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd
import omni.timeline
from pxr import UsdGeom

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ctx = omni.usd.get_context()
assert ctx.open_stage(os.path.join(PROJ, "assets", "scenes", "biolab_main_v10.usda"))
stage = ctx.get_stage()
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(30):
    app.update()

from _robot_ik import ArmFollower
arm = ArmFollower("/World/Robot/Robot1Mount/ur5e")
wrist = "/World/Robot/Robot1Mount/ur5e/wrist_3_link"


def usd_pos():
    m = UsdGeom.Xformable(stage.GetPrimAtPath(wrist)).ComputeLocalToWorldTransform(0.0)
    t = m.ExtractTranslation()
    return np.array([t[0], t[1], t[2]])


def show(tag):
    p = arm.state()[2]
    print("%-16s tensor %s | usd %s" % (tag, np.round(p, 4).tolist(),
                                        np.round(usd_pos(), 4).tolist()))


show("after 30f")
tl.stop()
for _ in range(6):
    app.update()
arm.reset()
tl.play()
for _ in range(3):
    app.update()
show("after restart")
from _robot_ik import ArmFollower as AF2
arm = AF2("/World/Robot/Robot1Mount/ur5e")
show("after recreate")

target = usd_pos() + np.array([0.0, 0.0, 0.12])
for i in range(40):
    arm.step(target, 0.0)
    app.update()
    if i % 10 == 9:
        show("f%d" % (i + 1))
print("target", np.round(target, 4).tolist())
tl.stop()
app.close()
