"""Build a separate standard-structure mini pilot from verified v5 geometry.

Does not migrate inactive lock state machines. Refuses active equality, actuators,
plugins, explicit contact exceptions or mixed collision masks in this pilot.
These require task-level conversion, never silent omission.
"""
import hashlib
import argparse
import json
import math
from pathlib import Path
from validate_gpu_capacity import bare_usd

bare_usd()
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

ROOT = Path(__file__).resolve().parents[1]
NAME = "centrifuge_tiangen_tgear_mini"


def extra_api(prim, token):
    tokens = list(prim.GetAppliedSchemas())
    if token not in tokens:
        tokens.append(token)
    prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(tokens))


def build():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-dir",type=Path,default=ROOT/"upstream/physics_reference")
    parser.add_argument("--allow-candidate",action="store_true")
    args = parser.parse_args()
    reference_path = args.reference_dir.resolve()/(NAME+".json")
    reference = json.loads(reference_path.read_text(encoding="utf-8"))
    if not reference["original_version_matched"] and not args.allow_candidate:
        raise ValueError("Candidate version requires explicit --allow-candidate")
    old = json.loads((ROOT/"upstream/truth"/(NAME+".json")).read_text(encoding="utf-8"))
    if reference["plugins"] or reference["actuators"] or reference["contact_pairs"] or reference["contact_excludes"]:
        raise ValueError("Pilot requires explicit migration of plugins/actuators/contact exceptions")
    if any(e["active0"] for e in reference["equality"]):
        raise ValueError("Active equality cannot be omitted")
    if reference["options"]["disableflags"] or reference["options"]["enableflags"]:
        raise ValueError("Non-default contact flags need explicit conversion")
    old_bodies = {b["name"]:b for b in old["bodies"]}
    deviations = []
    for b in reference["bodies"]:
        prev = old_bodies[b["name"]]
        mass_relative = abs(b["mass"]-prev["mass"])/max(prev["mass"],1e-12)
        pose_error = math.dist(b["world_pos"], prev["world_pos"])
        rotation_error = 1-abs(sum(x*y for x,y in zip(b["world_quat_wxyz"],prev["world_quat_wxyz"])))
        if mass_relative > 1e-6 or pose_error > 1e-8 or abs(rotation_error) > 1e-8:
            raise ValueError("Candidate reference differs from original geometry/mass: " + b["name"])
        deviations.append(dict(name=b["name"],mass_relative=mass_relative,position_m=pose_error,quaternion_dot_error=rotation_error))
    source = ROOT/"assets/instruments5"/(NAME+".usda")
    stage = Usd.Stage.Open(Usd.Stage.Open(str(source)).Flatten())
    top = stage.GetDefaultPrim()
    top.SetCustomDataByKey("validationScope", "mini pilot; candidate MuJoCo version; lock behavior not migrated")
    for b in reference["bodies"]:
        if not b["id"]:
            continue
        prim = stage.GetPrimAtPath("/Instrument/Bodies/"+b["name"])
        if not prim or b["mass"] <= 0 or min(b["inertia"]) <= 0:
            raise ValueError("Missing body or invalid inertia: " + b["name"])
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(False)
        extra_api(prim,"PhysxRigidBodyAPI")
        prim.CreateAttribute("physxRigidBody:linearDamping",Sdf.ValueTypeNames.Float).Set(0)
        prim.CreateAttribute("physxRigidBody:angularDamping",Sdf.ValueTypeNames.Float).Set(0)
        mass = UsdPhysics.MassAPI(prim)
        mass.CreateMassAttr(b["mass"])
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*b["ipos"]))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*b["inertia"]))
        mass.CreatePrincipalAxesAttr(Gf.Quatf(b["iquat"][0],Gf.Vec3f(*b["iquat"][1:])))
    bases = [b for b in reference["bodies"] if b["id"] and b["parentid"] == 0]
    if len(bases) != 1:
        raise ValueError("Pilot requires one fixed base")
    base = bases[0]
    fixed = UsdPhysics.FixedJoint.Define(stage,"/Instrument/Joints/world_fixed")
    fixed.CreateBody1Rel().SetTargets(["/Instrument/Bodies/"+base["name"]])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(*base["world_pos"]))
    fixed.CreateLocalRot0Attr(Gf.Quatf(base["world_quat_wxyz"][0],Gf.Vec3f(*base["world_quat_wxyz"][1:])))
    root = fixed.GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(root)
    extra_api(root,"PhysxArticulationAPI")
    root.CreateAttribute("physxArticulation:enabledSelfCollisions",Sdf.ValueTypeNames.Bool).Set(True)
    for j in reference["joints"]:
        matched = [p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint) and str(p.GetPath()).endswith("_"+j["name"])]
        if len(matched) != 1:
            raise ValueError("Ambiguous joint " + j["name"])
        prim = matched[0]
        extra_api(prim,"PhysxJointAPI")
        prim.CreateAttribute("physxJoint:armature",Sdf.ValueTypeNames.Float).Set(j["armature"])
        if j["frictionloss"]:
            raise ValueError("Nonzero frictionloss needs verified friction conversion")
        js = UsdPhysics.RevoluteJoint(prim)
        if not j["limited"]:
            js.CreateLowerLimitAttr(float("-inf"))
            js.CreateUpperLimitAttr(float("inf"))
        drive = UsdPhysics.DriveAPI(prim,"angular")
        drive.CreateStiffnessAttr(j["stiffness"]*math.pi/180)
        drive.CreateDampingAttr(j["damping"]*math.pi/180)
        drive.CreateTargetPositionAttr(math.degrees(reference["qpos_spring"][j["qposadr"]]-reference["qpos0"][j["qposadr"]]))
    # All six source colliders have mask (0,1): no automatic mutual contacts.
    # Self-filter this collection, preserving contact with external default (1,1)
    # objects. Multiple imported instruments need host-level mask groups rebuilt.
    masks = {(g["contype"],g["conaffinity"]) for g in reference["geoms"] if g["contype"] or g["conaffinity"]}
    if masks != {(0,1)}:
        raise ValueError("Mixed masks require full task-level group conversion")
    shapes = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    expected = sum(bool(g["contype"] or g["conaffinity"]) for g in reference["geoms"])
    if len(shapes) != expected:
        raise ValueError("Collider coverage differs from source")
    group = UsdPhysics.CollisionGroup.Define(stage,"/Instrument/CollisionFilters/mask_0_1")
    collection = group.GetCollidersCollectionAPI()
    collection.CreateExpansionRuleAttr("explicitOnly")
    collection.CreateIncludesRel().SetTargets([p.GetPath() for p in shapes])
    group.CreateFilteredGroupsRel().SetTargets([group.GetPath()])
    dest = ROOT/"assets/instruments6_pilot"/(NAME+".usda")
    dest.parent.mkdir(parents=True,exist_ok=True)
    stage.GetRootLayer().Export(str(dest))
    scene = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(scene,1)
    UsdGeom.SetStageUpAxis(scene,UsdGeom.Tokens.z)
    physics = UsdPhysics.Scene.Define(scene,"/physicsScene")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1))
    physics.CreateGravityMagnitudeAttr(0)
    scene.DefinePrim("/Mini","Xform").GetReferences().AddReference("../instruments6_pilot/"+NAME+".usda")
    scene_path = ROOT/"assets/scenes/test_mini_articulation.usda"
    scene.GetRootLayer().Export(str(scene_path))
    report = dict(asset=str(dest),asset_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),
        source_v5_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),reference=str(reference_path),
        reference_sha256=hashlib.sha256(reference_path.read_bytes()).hexdigest(),
        mujoco_version=reference["mujoco_version"],original_version_matched=reference["original_version_matched"],
        geometry_mass_comparison=deviations,colliders=len(shapes),native_armature=True,
        articulation_root=str(root.GetPath()),kinematic_bodies=0,
        inactive_equality_preserved_as_metadata=reference["equality"],lock_behavior_migrated=False,
        collision_scope="single instrument internal pairs; external default (1,1) contact remains enabled",
        runtime_validated=False)
    output = ROOT/"outputs/runtime_validation/mini_articulation_build.json"
    output.write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report),flush=True)


if __name__ == "__main__":
    build()
