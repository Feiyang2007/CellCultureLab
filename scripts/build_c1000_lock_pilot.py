"""Build an isolated C1000 articulation and switchable physical latch pilot.

Uses verified v5 geometry and the explicit diagnostic bridge reference. The
regular FixedJoint latch closes an articulation loop and is excluded from the
articulation. It is a hard functional approximation to MuJoCo's soft scalar
equality. Knob dry friction is NOT silently mapped to a friction coefficient.
"""
import hashlib
import json
import math
from pathlib import Path
import re
from validate_gpu_capacity import bare_usd

bare_usd()
from pxr import Usd,UsdGeom,UsdPhysics,Sdf,Gf

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"outputs/runtime_validation/c1000_isaac_lock_pilot"
NAME="thermal_cycler_biorad_c1000"


def slug(name):return re.sub(r"[^A-Za-z0-9_]","_",name)


def extra_api(prim,token):
    tokens=list(prim.GetAppliedSchemas())
    if token not in tokens:tokens.append(token)
    prim.SetMetadata("apiSchemas",Sdf.TokenListOp.CreateExplicit(tokens))


def build():
    source=ROOT/"assets/instruments5"/(NAME+".usda")
    reference_path=ROOT/"upstream/physics_bridge_reference"/(NAME+".json")
    reference=json.loads(reference_path.read_text(encoding="utf-8"))
    if reference["mujoco_version"]!="3.3.0" or reference.get("original_plugin_native_platform") is not False:
        raise ValueError("Expected explicitly labeled diagnostic bridge reference")
    if reference["actuators"] or reference["contact_pairs"] or reference["contact_excludes"]:
        raise ValueError("Unsupported actuator/contact rules")
    eq=reference["equality"]
    if len(eq)!=1 or eq[0]["name"]!="lid-lock" or eq[0]["type"]!=2 or eq[0]["active0"]:
        raise ValueError("Unexpected source equality")
    if reference["options"]["disableflags"] or reference["options"]["enableflags"]:
        raise ValueError("Unsupported simulation flags")
    stage=Usd.Stage.Open(Usd.Stage.Open(str(source)).Flatten())
    stage.GetDefaultPrim().SetCustomDataByKey("validationScope","C1000 physical latch pilot; bridge reference; hard latch approximation; knob dry friction unported; not production")
    bodies={b["name"]:b for b in reference["bodies"]}
    old=json.loads((ROOT/"upstream/truth"/(NAME+".json")).read_text(encoding="utf-8"))
    old_bodies={b["name"]:b for b in old["bodies"]}
    mass_errors=[]
    for b in reference["bodies"]:
        if not b["id"]:continue
        previous=old_bodies[b["name"]]
        error=abs(b["mass"]-previous["mass"])/max(previous["mass"],1e-12)
        if error>1e-6 or math.dist(b["world_pos"],previous["world_pos"])>1e-8:
            raise ValueError("Reference/source mismatch: "+b["name"])
        mass_errors.append(dict(name=b["name"],mass_relative_error=error))
        prim=stage.GetPrimAtPath("/Instrument/Bodies/"+slug(b["name"]))
        if not prim or b["mass"]<=0 or min(b["inertia"])<=0:raise ValueError("Invalid body "+b["name"])
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(False)
        extra_api(prim,"PhysxRigidBodyAPI")
        prim.CreateAttribute("physxRigidBody:linearDamping",Sdf.ValueTypeNames.Float).Set(0)
        prim.CreateAttribute("physxRigidBody:angularDamping",Sdf.ValueTypeNames.Float).Set(0)
        mass=UsdPhysics.MassAPI(prim)
        mass.CreateMassAttr(b["mass"]);mass.CreateCenterOfMassAttr(Gf.Vec3f(*b["ipos"]))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*b["inertia"]))
        mass.CreatePrincipalAxesAttr(Gf.Quatf(b["iquat"][0],Gf.Vec3f(*b["iquat"][1:])))
    bases=[b for b in reference["bodies"] if b["id"] and b["parentid"]==0]
    if len(bases)!=1:raise ValueError("Expected one fixed base")
    base=bases[0]
    fixed=UsdPhysics.FixedJoint.Define(stage,"/Instrument/Joints/world_fixed")
    fixed.CreateBody1Rel().SetTargets(["/Instrument/Bodies/"+slug(base["name"])])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(*base["world_pos"]))
    fixed.CreateLocalRot0Attr(Gf.Quatf(base["world_quat_wxyz"][0],Gf.Vec3f(*base["world_quat_wxyz"][1:])))
    UsdPhysics.ArticulationRootAPI.Apply(fixed.GetPrim());extra_api(fixed.GetPrim(),"PhysxArticulationAPI")
    fixed.GetPrim().CreateAttribute("physxArticulation:enabledSelfCollisions",Sdf.ValueTypeNames.Bool).Set(True)
    paths={};unsupported=[]
    for j in reference["joints"]:
        candidates=[p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint) and str(p.GetPath()).endswith("_"+slug(j["name"]))]
        if len(candidates)!=1:raise ValueError("Ambiguous joint: "+j["name"])
        prim=candidates[0];paths[j["name"]]=str(prim.GetPath())
        extra_api(prim,"PhysxJointAPI")
        prim.CreateAttribute("physxJoint:armature",Sdf.ValueTypeNames.Float).Set(j["armature"])
        drive=UsdPhysics.DriveAPI(prim,"angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(j["stiffness"]*math.pi/180)
        drive.CreateDampingAttr(j["damping"]*math.pi/180)
        drive.CreateTargetPositionAttr(math.degrees(reference["qpos_spring"][j["qposadr"]]-reference["qpos0"][j["qposadr"]]))
        if j["frictionloss"]:
            unsupported.append(dict(joint=j["name"],source_coulomb_effort_nm=j["frictionloss"],mapped=False))
    source_lid=UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(paths["lid"]))
    latch=UsdPhysics.FixedJoint.Define(stage,"/Instrument/StateConstraints/lid_lock")
    latch.CreateBody0Rel().SetTargets(source_lid.GetBody0Rel().GetTargets())
    latch.CreateBody1Rel().SetTargets(source_lid.GetBody1Rel().GetTargets())
    latch.CreateLocalPos0Attr(source_lid.GetLocalPos0Attr().Get())
    latch.CreateLocalPos1Attr(source_lid.GetLocalPos1Attr().Get())
    target=float(eq[0]["data"][0])-reference["qpos0"][next(j["qposadr"] for j in reference["joints"] if j["name"]=="lid")]
    rotation=Gf.Quatf(math.cos(target/2),Gf.Vec3f(math.sin(target/2),0,0))
    latch.CreateLocalRot0Attr(source_lid.GetLocalRot0Attr().Get()*rotation)
    latch.CreateLocalRot1Attr(source_lid.GetLocalRot1Attr().Get())
    latch.CreateJointEnabledAttr(False);latch.CreateExcludeFromArticulationAttr(True)
    masks={(g["contype"],g["conaffinity"]) for g in reference["geoms"] if g["contype"] or g["conaffinity"]}
    if masks!={(0,1)}:raise ValueError("Unexpected internal collision masks")
    shapes=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    if len(shapes)!=sum(bool(g["contype"] or g["conaffinity"]) for g in reference["geoms"]):raise ValueError("Collider coverage mismatch")
    group=UsdPhysics.CollisionGroup.Define(stage,"/Instrument/CollisionFilters/mask_0_1")
    collection=group.GetCollidersCollectionAPI();collection.CreateExpansionRuleAttr("explicitOnly")
    collection.CreateIncludesRel().SetTargets([p.GetPath() for p in shapes])
    group.CreateFilteredGroupsRel().SetTargets([group.GetPath()])
    OUT.mkdir(parents=True,exist_ok=True)
    asset=OUT/"asset.usda";stage.GetRootLayer().Export(str(asset))
    scene=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(scene,1);UsdGeom.SetStageUpAxis(scene,UsdGeom.Tokens.z)
    physics=UsdPhysics.Scene.Define(scene,"/physicsScene")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));physics.CreateGravityMagnitudeAttr(0)
    extra_api(physics.GetPrim(),"PhysxSceneAPI")
    physics.GetPrim().CreateAttribute("physxScene:timeStepsPerSecond",Sdf.ValueTypeNames.UInt).Set(1000)
    physics.GetPrim().CreateAttribute("physxScene:enableGPUDynamics",Sdf.ValueTypeNames.Bool).Set(False)
    physics.GetPrim().CreateAttribute("physxScene:broadphaseType",Sdf.ValueTypeNames.Token).Set("MBP")
    host=scene.DefinePrim("/C1000","Xform");host.GetReferences().AddReference(str(asset))
    scene.SetDefaultPrim(host)
    scene.GetRootLayer().customLayerData={"validationScope":"C1000 native physical latch pilot; not complete task or source-equivalent dynamics"}
    dest=OUT/"scene.usda";scene.GetRootLayer().Export(str(dest))
    reopened=Usd.Stage.Open(str(dest))
    errors=[str(e) for p in reopened.Traverse() for e in p.GetPrimIndex().localErrors]
    if errors:raise RuntimeError(str(errors))
    report=dict(scene=str(dest),asset=str(asset),asset_sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),
        reference=str(reference_path),reference_sha256=hashlib.sha256(reference_path.read_bytes()).hexdigest(),
        source_v5_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),mass_checks=mass_errors,
        joints=paths,articulation_root="/C1000/Joints/world_fixed",latch_path="/C1000/StateConstraints/lid_lock",
        lock_target_source_rad=target,latch_backend="excluded regular FixedJoint, hard approximation to scalar soft equality",
        unresolved_frictionloss=unsupported,colliders=len(shapes),physics_hz=1000,
        detent_mapping="source passive spring with C-round nearest-step position target",
        composition_errors=errors,production_ready=False,runtime_validated=False)
    (OUT/"build.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report),flush=True)


if __name__=="__main__":build()
