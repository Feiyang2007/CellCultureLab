"""只读探针：UR5e + Robotiq 在 v09 里到底能不能驱动。

输出四件事：
1) 带 PhysicsArticulationRootAPI 的 prim（含 payload 加载后的机器人）；
2) /World/Robot 的子树（两层）；
3) 每台机器人下的 joint（类型/轴/限位/body0/body1）；
4) play 60 帧前后 wrist_3_link 的位姿，判断它是不是掉下去/被焊住。
"""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
scene = os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda")
ctx = omni.usd.get_context()
assert ctx.open_stage(scene), scene
stage = ctx.get_stage()
import omni.timeline
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(60):
    app.update()


def pose(path):
    prim = stage.GetPrimAtPath(path)
    if not prim:
        return None
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.0)
    t = m.ExtractTranslation()
    return tuple(round(v, 4) for v in t)


print("=== articulation roots ===")
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(" ", prim.GetPath(), prim.GetTypeName())

print("=== /World/Robot subtree ===")
root = stage.GetPrimAtPath("/World/Robot")
for child in root.GetChildren():
    print(" ", child.GetPath(), child.GetTypeName(),
          "| payload:", bool(child.GetPayloads()))
    for c2 in child.GetChildren():
        print("    ", c2.GetPath(), c2.GetTypeName())

print("=== joints under /World/Robot ===")
n = 0
for prim in stage.Traverse():
    if not str(prim.GetPath()).startswith("/World/Robot"):
        continue
    if prim.IsA(UsdPhysics.Joint):
        n += 1
        b0 = prim.GetRelationship("physics:body0").GetTargets()
        b1 = prim.GetRelationship("physics:body1").GetTargets()
        ax = prim.GetAttribute("physics:axis").Get()
        lo = prim.GetAttribute("physics:lowerLimit").Get()
        hi = prim.GetAttribute("physics:upperLimit").Get()
        print(" ", prim.GetPath(), prim.GetTypeName(), "axis", ax,
              "limit", lo, hi)
        print("      body0", [str(x) for x in b0], "body1", [str(x) for x in b1])
print("joints:", n)

print("=== wrist poses (should stay if articulated/fixed) ===")
for p in ("/World/Robot/Robot1Mount/ur5e/base_link",
          "/World/Robot/Robot1Mount/ur5e/wrist_3_link",
          "/World/Robot/Robot2Mount/ur5e/base_link",
          "/World/Robot/Robot2Mount/ur5e/wrist_3_link"):
    print(" ", p, pose(p))

print("=== articulation DOF probe ===")
try:
    from isaacsim.core.prims import Articulation
    for path in ("/World/Robot/Robot1Mount/ur5e", "/World/Robot/Robot2Mount/ur5e"):
        try:
            art = Articulation(path)
            art.initialize()
            print(" ", path, "n_dof", art.num_dof, "dof_names", list(art.dof_names or []))
        except Exception as e:
            print(" ", path, "Articulation FAILED:", repr(e)[:200])
except Exception as e:
    print(" Articulation API unavailable:", repr(e)[:200])

tl.stop()
app.close()
