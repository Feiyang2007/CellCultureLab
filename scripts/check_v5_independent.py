"""Independent v5 checks against original MuJoCo vectors and USD schema types."""
import json
import math
from pathlib import Path
import sys
from mjcf_instrument_to_usd_v5 import san
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from runtime_joint_math import quat_mul, quat_rotate

ROOT = Path(__file__).resolve().parents[1]
results = []
for source in sorted((ROOT / "upstream/truth").glob("*.json")):
    truth = json.loads(source.read_text(encoding="utf-8"))
    stage = Usd.Stage.Open(str(ROOT / "assets/instruments5" / (source.stem+".usda")))
    bodies = {b["id"]:b for b in truth["bodies"]}
    holders = {p.GetName():p for p in stage.Traverse() if p.GetTypeName()=="Xform"}
    cache = UsdGeom.XformCache()
    def record_for_path(path, fallback):
        p = stage.GetPrimAtPath(path)
        if not p.GetCustomDataByKey("auxiliaryDofLink"):
            return fallback
        m = cache.GetLocalToWorldTransform(p)
        q = m.ExtractRotationQuat()
        return dict(world_pos=list(m.ExtractTranslation()),world_quat_wxyz=[q.GetReal(),*q.GetImaginary()])
    errors = []
    max_geom = max_axis = max_anchor = max_frame = 0.0
    checked = 0
    for g in truth["geoms"]:
        if g["body_id"] == 0:
            continue
        holder = holders.get(san("%s_%04d"%(g["name"],g["id"])))
        if not holder:
            errors.append("Missing geom "+str(g["id"]))
            continue
        shape = stage.GetPrimAtPath(str(holder.GetPath())+"/shape")
        if not shape.IsA(UsdGeom.Gprim):
            errors.append("Invalid geometry schema "+str(shape.GetPath()))
        for name in ("radius","height","size"):
            a = shape.GetAttribute(name)
            if a and str(a.GetTypeName()) != "double":
                errors.append("Incorrect numeric schema type "+str(shape.GetPath())+":"+name)
        world = cache.GetLocalToWorldTransform(holder)
        R, p = g["world_xmat_rowmajor"],g["world_pos"]
        for local in ([0,0,0],[.01,0,0],[0,.01,0],[0,0,.01]):
            actual = world.Transform(Gf.Vec3d(*local))
            expected = [p[i]+sum(R[i*3+k]*local[k] for k in range(3)) for i in range(3)]
            max_geom = max(max_geom,math.dist(actual,expected))
        # Box dimensions must be actual Cube scale, not a misleading extent.
        if int(g["type"]) == 6:
            scale = shape.GetAttribute("xformOp:scale").Get()
            if not shape.IsA(UsdGeom.Cube) or any(abs(scale[i]-g["size"][i])>1e-6 for i in range(3)):
                errors.append("Wrong box size/schema "+str(shape.GetPath()))
        checked += 1
    movable = [j for j in truth["joints"] if int(j["type"]) in (2,3)]
    for j in movable:
        body1 = bodies[j["body_id"]]
        body0 = bodies[body1["parent_id"]]
        path = "/Instrument/Joints/"+san(body0["name"]+"_"+j["name"])
        prim = stage.GetPrimAtPath(path)
        expected_type = UsdPhysics.RevoluteJoint if int(j["type"])==3 else UsdPhysics.PrismaticJoint
        mode = "angular" if int(j["type"])==3 else "linear"
        if not prim or not prim.IsA(expected_type) or not prim.HasAPI(UsdPhysics.DriveAPI,mode):
            errors.append("Joint or drive schema invalid "+path)
            continue
        world_frames = []
        body0 = record_for_path(prim.GetRelationship("physics:body0").GetTargets()[0],body0)
        body1 = record_for_path(prim.GetRelationship("physics:body1").GetTargets()[0],body1)
        for index,b in enumerate((body0,body1)):
            lr = prim.GetAttribute("physics:localRot"+str(index)).Get()
            lr = [*lr.GetImaginary(),lr.GetReal()]
            q = b["world_quat_wxyz"]
            bq = q[1:]+q[:1]
            world_q = quat_mul(bq,lr)
            world_frames.append(world_q)
            axis = quat_rotate(world_q,[1,0,0])
            max_axis = max(max_axis,math.dist(axis,j["world_axis"]))
            lp = prim.GetAttribute("physics:localPos"+str(index)).Get()
            rotated = quat_rotate(bq,lp)
            anchor = [b["world_pos"][i]+rotated[i] for i in range(3)]
            max_anchor = max(max_anchor,math.dist(anchor,j["world_anchor"]))
        max_frame = max(max_frame,1-abs(sum(a*b for a,b in zip(*world_frames))))
    rb = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    mass_count = sum(p.HasAPI(UsdPhysics.MassAPI) for p in rb)
    for p in rb:
        if p.GetCustomDataByKey("auxiliaryDofLink"):
            source_joint = next(j for j in truth["joints"] if j["id"]==p.GetCustomDataByKey("sourceJointId"))
            m = cache.GetLocalToWorldTransform(p)
            max_geom = max(max_geom,math.dist(m.ExtractTranslation(),source_joint["world_anchor"]))
            max_axis = max(max_axis,math.dist(m.TransformDir(Gf.Vec3d(1,0,0)),source_joint["world_axis"]))
    pairs = [(str(p.GetRelationship("physics:body0").GetTargets()),str(p.GetRelationship("physics:body1").GetTargets()))
             for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)]
    if len(set(pairs))!=len(pairs):
        errors.append("Multiple joints still connect the same body pair")
    if mass_count != len(rb):
        errors.append("Missing MassAPI")
    fixed = sum(p.IsA(UsdPhysics.FixedJoint) for p in stage.Traverse())
    expected_fixed = sum(b["id"]!=0 and b["parent_id"]!=0 and
                         b["id"] not in {j["body_id"] for j in truth["joints"]} for b in truth["bodies"])
    if fixed != expected_fixed:
        errors.append("Wrong welded body count")
    if any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()):
        errors.append("Asset contains a PhysicsScene")
    if max_geom>1e-6 or max_axis>1e-6 or max_anchor>1e-6 or abs(max_frame)>1e-6:
        errors.append("Vector/frame tolerance exceeded")
    result = dict(asset=source.stem,checked_geoms=checked,movable_joints=len(movable),
                  fixed_joints=fixed,rigid_bodies=len(rb),mass_api=mass_count,
                  geom_error_mm=max_geom*1000,axis_error=max_axis,
                  anchor_error_mm=max_anchor*1000,frame_error=max_frame,
                  errors=errors,passed=not errors)
    results.append(result)
    print(json.dumps(result),flush=True)
mini = Usd.Stage.Open(str(ROOT / "assets/scenes/test_instruments5.usda"))
composition = [str(e) for p in mini.Traverse() for e in p.GetPrimIndex().localErrors]
report = dict(passed=bool(results) and all(r["passed"] for r in results) and not composition,
              results=results,scene_composition_errors=composition,
              scene_count=sum(p.IsA(UsdPhysics.Scene) for p in mini.Traverse()),
              recognized_movable_joints=sum(p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint) for p in mini.Traverse()))
out = ROOT / "outputs/runtime_validation/v5_independent.json"
out.write_text(json.dumps(report,indent=2),encoding="utf-8")
print("INDEPENDENT_PASS",report["passed"])
raise SystemExit(0 if report["passed"] else 1)

