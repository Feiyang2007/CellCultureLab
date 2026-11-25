"""Run contact-enabled native-articulation torque trials against candidate MuJoCo.

The report distinguishes candidate dynamics checks from original-version
acceptance; no drive injection or solver-iteration increase is used.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--reference",type=Path,help="Compare unchanged pilot against another same-source reference")
parser.add_argument("--report",type=Path,default=ROOT/"outputs/runtime_validation/mini_articulation_comparison.json")
parser.add_argument("--temp-dir",type=Path,default=ROOT/".runtime/tmp")
args = parser.parse_args()
TMP = args.temp_dir.resolve()
TMP.mkdir(parents=True,exist_ok=True)
# Fail before starting Isaac if the selected directory cannot hold temporary files.
import tempfile
with tempfile.TemporaryFile(dir=TMP):
    pass
os.environ["TEMP"] = os.environ["TMP"] = str(TMP)
build_metadata = json.loads((ROOT/"outputs/runtime_validation/mini_articulation_build.json").read_text(encoding="utf-8"))
REFERENCE = Path(build_metadata["reference"])
ref = json.loads(REFERENCE.read_text(encoding="utf-8"))
if hashlib.sha256(REFERENCE.read_bytes()).hexdigest() != build_metadata["reference_sha256"]:
    raise RuntimeError("Reference changed after building the pilot")
if args.reference:
    override = json.loads(args.reference.read_text(encoding="utf-8"))
    if override["source_xml_sha256"] != ref["source_xml_sha256"]:
        raise RuntimeError("Override reference is from a different source XML")
    if bool(override["original_version_matched"]) != (override["mujoco_version"] == "3.3.0"):
        raise RuntimeError("Inconsistent reference version label")
    ref = override
    REFERENCE = args.reference.resolve()
asset_path = ROOT/"assets/instruments6_pilot/centrifuge_tiangen_tgear_mini.usda"
if hashlib.sha256(asset_path.read_bytes()).hexdigest() != build_metadata["asset_sha256"]:
    raise RuntimeError("Pilot asset changed after building")
scene_path = ROOT/"assets/scenes/test_mini_articulation.usda"
from isaacsim import SimulationApp
app = SimulationApp({"headless":True,"extra_args":["--portable-root",str(Path.home()/"CellCultureLabRuntime"),
    "--/app/settings/persistent=false","--/renderer/multiGpu/enabled=false"]})
import numpy as np
import omni.usd
import omni.physx
import omni.physics.tensors as tensors
from pxr import UsdPhysics,UsdGeom,PhysxSchema,Gf
from runtime_joint_math import joint_constraint_error

report = dict(complete=False,scope="candidate cross-engine torque comparison, contact-enabled single mini",
    original_version_matched=ref["original_version_matched"],mujoco_version=ref["mujoco_version"],
    original_version_acceptance=False,reference_sha256=hashlib.sha256(REFERENCE.read_bytes()).hexdigest(),
    contacts_enabled=True,drive_injected=False,solver_iterations_overridden=False,rows=[])
report["reference_path"] = str(REFERENCE)
report["asset_build_mujoco_version"] = build_metadata["mujoco_version"]
report["reference_override_used"] = args.reference is not None
report["scope"] = "cross-engine torque comparison of unchanged pilot against selected reference; not complete task acceptance"
report["asset_sha256"] = hashlib.sha256((ROOT/"assets/instruments6_pilot/centrifuge_tiangen_tgear_mini.usda").read_bytes()).hexdigest()
report["physics_time_steps_per_second"] = 240
report["tolerances"] = dict(angle_deg=.5,velocity_rad_s=.1,forbidden_position_m=.0001,forbidden_rotation_deg=1)
ctx = omni.usd.get_context()
iface = omni.physx.get_physx_interface()


def trial(protocol):
    iface.reset_simulation()
    tensors.reset()
    if not ctx.open_stage(str(scene_path)):
        raise RuntimeError("Could not open pilot")
    stage = ctx.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    scene = UsdPhysics.Scene(stage.GetPrimAtPath("/physicsScene"))
    g = protocol["gravity"]
    mag = math.sqrt(sum(x*x for x in g))
    scene.CreateGravityDirectionAttr(Gf.Vec3f(*[x/mag for x in g]) if mag else Gf.Vec3f(0,0,-1))
    scene.CreateGravityMagnitudeAttr(mag)
    ps = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    ps.CreateEnableGPUDynamicsAttr(False)
    ps.CreateBroadphaseTypeAttr("MBP")
    ps.CreateTimeStepsPerSecondAttr(round(1/protocol["dt"]))
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    if protocol["test_armature_override"] is not None:
        joint = next(p for p in joints if str(p.GetPath()).endswith("_"+protocol["joint"]))
        PhysxSchema.PhysxJointAPI.Apply(joint).CreateArmatureAttr(protocol["test_armature_override"])
    iface.start_simulation()
    dt = protocol["dt"]
    iface.update_simulation(dt,0)
    view = tensors.create_simulation_view("numpy")
    try:
        art = view.create_articulation_view("/Mini/Joints/world_fixed")
        if art.count != 1 or not art.shared_metatype.fixed_base or art.max_dofs != len(ref["joints"]):
            raise RuntimeError("Expected exactly one fixed-base two-DOF native articulation")
        names = list(art.shared_metatype.dof_names)
        mapping = [next(i for i,n in enumerate(names) if n.endswith("_"+j["name"])) for j in ref["joints"]]
        indices = np.array([0],dtype=np.uint32)
        q = np.zeros((1,art.max_dofs),dtype=np.float32)
        v = q.copy()
        force = q.copy()
        for j,i in zip(ref["joints"],mapping):
            q[0,i] = protocol["initial_q"][j["qposadr"]]-ref["qpos0"][j["qposadr"]]
            if j["name"] == protocol["joint"]:
                force[0,i] = protocol["force"]
        art.set_dof_positions(q,indices)
        art.set_dof_velocities(v,indices)
        native_armatures = art.get_dof_armatures().copy()[0,mapping].tolist()
        expected = [protocol["test_armature_override"] if j["name"]==protocol["joint"] and protocol["test_armature_override"] is not None else j["armature"] for j in ref["joints"]]
        if any(abs(a-b)>1e-6 for a,b in zip(native_armatures,expected)):
            raise RuntimeError("Native armature runtime readback mismatch")
        bodies = view.create_rigid_body_view("/Mini/Bodies/*")
        constraints = []
        trajectory = []
        for step in range(protocol["steps"]+1):
            qnow = art.get_dof_positions().copy()[0,mapping]
            vnow = art.get_dof_velocities().copy()[0,mapping]
            trajectory.append(dict(step=step,q=qnow.tolist(),v=vnow.tolist()))
            if step:
                poses = dict(zip(bodies.prim_paths,bodies.get_transforms().copy().tolist()))
                for p in joints:
                    b0 = str(p.GetRelationship("physics:body0").GetTargets()[0])
                    b1 = str(p.GetRelationship("physics:body1").GetTargets()[0])
                    xyzw = lambda x:[*x.GetImaginary(),x.GetReal()]
                    constraints.append(joint_constraint_error("RevoluteJoint",poses[b0],poses[b1],
                        list(p.GetAttribute("physics:localPos0").Get()),list(p.GetAttribute("physics:localPos1").Get()),
                        xyzw(p.GetAttribute("physics:localRot0").Get()),xyzw(p.GetAttribute("physics:localRot1").Get()),"X"))
            if step < protocol["steps"]:
                art.set_dof_actuation_forces(force,indices)
                iface.update_simulation(dt,(step+1)*dt)
        dq = [abs(a-b) for x,y in zip(trajectory,protocol["trajectory"]) for a,b in zip(x["q"],y["q"])]
        dv = [abs(a-b) for x,y in zip(trajectory,protocol["trajectory"]) for a,b in zip(x["v"],y["v"])]
        max_pos = max(x["position_m"] for x in constraints)
        max_rot = max(x["angle_deg"] for x in constraints)
        passed = max(dq)<math.radians(.5) and max(dv)<.1 and max_pos<.0001 and max_rot<1
        return dict(joint=protocol["joint"],force=protocol["force"],gravity=g,
            test_armature_override=protocol["test_armature_override"],native_armatures=native_armatures,
            articulation_dof_names=names,max_angle_error_deg=math.degrees(max(dq)),max_velocity_error_rad_s=max(dv),
            max_forbidden_position_m=max_pos,max_forbidden_rotation_deg=max_rot,
            passed=passed,trajectory=trajectory)
    finally:
        view.invalidate()
        iface.reset_simulation()


def contact_filter_trial(enabled):
    """Positive/negative controls on the authored group, without disabling contacts.

    Remote test cubes overlap by 5 mm. A/B belong to the instrument group;
    C belongs to it and D is external. Contacts must displace D, but only
    displace B when the self-filter is deliberately removed.
    """
    iface.reset_simulation()
    tensors.reset()
    if not ctx.open_stage(str(scene_path)):
        raise RuntimeError("Could not open pilot for filter control")
    stage = ctx.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    scene = UsdPhysics.Scene(stage.GetPrimAtPath("/physicsScene"))
    scene.CreateGravityMagnitudeAttr(0)
    ps = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    ps.CreateEnableGPUDynamicsAttr(False)
    ps.CreateBroadphaseTypeAttr("MBP")
    ps.CreateTimeStepsPerSecondAttr(240)
    group = UsdPhysics.CollisionGroup(stage.GetPrimAtPath("/Mini/CollisionFilters/mask_0_1"))
    includes = group.GetCollidersCollectionAPI().GetIncludesRel()
    if not enabled:
        # ClearTargets removes the session opinion and reveals referenced targets;
        # an explicit empty list is required to block the source self-filter.
        group.GetFilteredGroupsRel().SetTargets([])
    initial = {}
    for label,x,kin,in_group in [("A",1,True,True),("B",1.015,False,True),
                                ("C",2,True,True),("D",2.015,False,False)]:
        cube = UsdGeom.Cube.Define(stage,"/Probes/"+label)
        cube.CreateSizeAttr(.02)
        cube.AddTranslateOp().Set(Gf.Vec3d(x,0,0))
        prim = cube.GetPrim()
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        rb.CreateKinematicEnabledAttr(kin)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(1)
        UsdPhysics.CollisionAPI.Apply(prim)
        PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateContactOffsetAttr(.0001)
        PhysxSchema.PhysxCollisionAPI(prim).CreateRestOffsetAttr(0)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateLinearDampingAttr(0)
        if in_group:
            includes.AddTarget(prim.GetPath())
        initial[str(prim.GetPath())] = [x,0,0]
    iface.start_simulation()
    dt = 1/240
    iface.update_simulation(dt,0)
    view = tensors.create_simulation_view("numpy")
    try:
        bodies = view.create_rigid_body_view("/Probes/*")
        for step in range(1,24):
            iface.update_simulation(dt,step*dt)
        poses = dict(zip(bodies.prim_paths,bodies.get_transforms().copy().tolist()))
        displacement = {label:math.dist(initial["/Probes/"+label],poses["/Probes/"+label][:3]) for label in ("B","D")}
        passed = displacement["D"]>.003 and (displacement["B"]<1e-6 if enabled else displacement["B"]>.003)
        return dict(self_filter_enabled=enabled,displacement_m=displacement,passed=passed,
            composed_filtered_groups=[str(x) for x in group.GetFilteredGroupsRel().GetTargets()],
            scope="runtime positive/negative filter controls; remote boxes, not insertion or grasp validation")
    finally:
        view.invalidate()
        iface.reset_simulation()


try:
    for protocol in ref["trials"]:
        row = trial(protocol)
        report["rows"].append(row)
        print("TRIAL_RESULT "+json.dumps({k:v for k,v in row.items() if k!="trajectory"}),flush=True)
    report["filter_controls"] = [contact_filter_trial(True),contact_filter_trial(False)]
    print("FILTER_RESULT "+json.dumps(report["filter_controls"]),flush=True)
    report["complete"] = True
    report["candidate_dynamics_passed"] = len(report["rows"])==8 and all(x["passed"] for x in report["rows"])
    report["filter_controls_passed"] = all(x["passed"] for x in report["filter_controls"])
    report["original_version_dynamics_passed"] = report["original_version_matched"] and report["candidate_dynamics_passed"]
except Exception as error:
    report["fatal_error"] = repr(error)
    traceback.print_exc()
finally:
    print("REPORT_JSON "+json.dumps(report,allow_nan=False),flush=True)
    try:
        args.report.write_text(json.dumps(report,indent=2,allow_nan=False),encoding="utf-8")
    except OSError as error:
        print("REPORT_FILE_ERROR "+str(error),flush=True)
    app.close(wait_for_replicator=False)
