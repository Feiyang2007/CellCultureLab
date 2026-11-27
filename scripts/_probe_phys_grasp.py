"""物理夹持探针：FixedJoint(wrist, object) + jointEnabled 运行期开关。"""
import os
import sys
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd
import omni.timeline
from pxr import Usd, UsdPhysics, Sdf, Gf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ctx = omni.usd.get_context()
assert ctx.open_stage(os.path.join(PROJ, "assets", "scenes", "biolab_main_v10.usda"))
stage = ctx.get_stage()

WRIST = "/World/Robot/Robot1Mount/ur5e/wrist_3_link"
OBJ = "/World/TaskArea/Stations/WellPlate96"

# 预建固定关节（默认关），play 之前
jpath = "/World/GraspJoint"
jp = UsdPhysics.FixedJoint.Define(stage, jpath)
jp.CreateBody0Rel().SetTargets([WRIST])
jp.CreateBody1Rel().SetTargets([OBJ])
jp.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
jp.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
jp.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
jp.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
en = jp.GetPrim().CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool)
en.Set(False)

tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(30):
    app.update()

from _robot_ik import ArmFollower
arm = ArmFollower("/World/Robot/Robot1Mount/ur5e")
from pxr import UsdGeom


def objpos():
    m = UsdGeom.Xformable(stage.GetPrimAtPath(OBJ)).ComputeLocalToWorldTransform(0.0)
    t = m.ExtractTranslation()
    return np.array([t[0], t[1], t[2]])


start = objpos()
print("obj start", np.round(start, 3).tolist())
target = start + np.array([0.0, 0.11, 0.0])
hover = 0.14

# 接近
grasp = start + np.array([0, 0, hover])
w0 = arm.state()[2]
for i in range(1, 13):
    t = i / 12.0
    arm.step(w0 + (grasp - w0) * t, 0.0)
    app.update()
for _ in range(5):
    arm.step(grasp, 0.9)
    app.update()

# 算 wrist 系偏移，开关节
q, qd, w, quat = arm.state()
o = objpos()
x, y, z, ww = quat
R = np.array([
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * ww), 2 * (x * z + y * ww)],
    [2 * (x * y + z * ww), 1 - 2 * (x * x + z * z), 2 * (y * z - x * ww)],
    [2 * (x * z - y * ww), 2 * (y * z + x * ww), 1 - 2 * (x * x + y * y)],
])
off_local = R.T @ (o - w)
jp.GetLocalPos0Attr().Set(Gf.Vec3f(*off_local))
en.Set(True)
for _ in range(5):
    app.update()
print("attached obj", np.round(objpos(), 3).tolist(), "wrist", np.round(w, 3).tolist())

# 带着走
for i in range(1, 40):
    t = i / 39.0
    arm.step(grasp + (target + np.array([0, 0, hover]) - grasp) * t, 0.9)
    app.update()
print("carried obj", np.round(objpos(), 3).tolist())

# 松手
en.Set(False)
for _ in range(30):
    app.update()
print("released obj", np.round(objpos(), 3).tolist(), "target", np.round(target, 3).tolist())
tl.stop()
app.close()
