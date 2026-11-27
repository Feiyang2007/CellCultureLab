import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env
_pxr_env.bootstrap()
from pxr import Usd

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
stage = Usd.Stage.Open(os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda"))
root = "/World/TaskArea/Stations/AClass"
print("exists:", stage.GetPrimAtPath(root).IsValid())
n_rb = n_col = n_jt = n_slot = 0
for prim in stage.Traverse():
    p = str(prim.GetPath())
    if not p.startswith(root):
        continue
    if "PhysicsRigidBodyAPI" in prim.GetAppliedSchemas():
        n_rb += 1
    if "PhysicsCollisionAPI" in prim.GetAppliedSchemas():
        n_col += 1
    if prim.GetTypeName() in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
        n_jt += 1
    if "/SlotAnchors/" in p:
        n_slot += 1
print("AClass 子树：刚体 %d 碰撞 %d 关节 %d 锚点 %d" % (n_rb, n_col, n_jt, n_slot))
d = stage.GetPrimAtPath(root + "/Co2Incubator")
print("Co2Incubator valid:", d.IsValid(), "type:", d.GetTypeName(),
      "schemas:", d.GetAppliedSchemas())
for c in d.GetChildren():
    print("   ", c.GetPath(), c.GetTypeName())
j = stage.GetPrimAtPath(root + "/Co2Incubator/Joints/door")
print("door joint:", j.IsValid(), j.GetTypeName(),
      "body1:", j.GetRelationship("physics:body1").GetTargets() if j.IsValid() else None,
      "limits:", (j.GetAttribute("physics:lowerLimit").Get(), j.GetAttribute("physics:upperLimit").Get()) if j.IsValid() else None)
b = stage.GetPrimAtPath(root + "/Co2Incubator/door")
print("door body:", b.IsValid(), b.GetTypeName(), b.GetAppliedSchemas())
print("first 12 prims under CryoBox25:")
for i, prim in enumerate(stage.Traverse()):
    p = str(prim.GetPath())
    if p.startswith(root + "/LN2Dewar"):
        print("   ", p, prim.GetTypeName())
