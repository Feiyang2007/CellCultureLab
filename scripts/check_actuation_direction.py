"""PhysX tensor joint measurement. All edits are temporary session overrides.

Default: read runtime poses and inspect native DriveAPI.
--apply-drive: inject diagnostic drive (does not repair the source asset).
Each positive/negative/hold trial reloads the stage and settles at a legal center.
Missing data is never reported as zero or immobility.
"""
import argparse
from collections import Counter
import json
import math
import hashlib
from pathlib import Path
import sys
import traceback
from runtime_joint_math import joint_coordinate, joint_constraint_error, target_plan, validate_pose

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("scene", nargs="?", default=str(ROOT / "assets/scenes/test_instruments4.usda"))
parser.add_argument("--apply-drive", action="store_true")
parser.add_argument("--filter", default="", help="Joint path substrings separated by |")
parser.add_argument("--settle", type=int, default=360)
parser.add_argument("--run", type=int, default=240)
parser.add_argument("--dt", type=float, default=1/120)
parser.add_argument("--no-contacts", action="store_true", help="Isolation only, not task validation")
parser.add_argument("--zero-gravity", action="store_true")
parser.add_argument("--zero-initial-velocities", action="store_true", help="Explicitly zero all velocities after reference settling")
parser.add_argument("--angular-stiffness", type=float, default=3000.0)
parser.add_argument("--angular-damping", type=float, default=300.0)
parser.add_argument("--max-force", type=float, default=1e6)
parser.add_argument("--solver-iterations", type=int, default=0)
parser.add_argument("--output", default=str(ROOT / "outputs/runtime_validation/actuation.json"))
args = parser.parse_args()
if not math.isfinite(args.dt) or args.dt <= 0 or args.run < 1 or args.settle < 1:
    parser.error("dt, settle and run must be positive")
if abs(1/args.dt-round(1/args.dt)) > 1e-6:
    parser.error("dt must correspond to an integer PhysicsScene timeStepsPerSecond")
if any(not math.isfinite(v) or v < 0 for v in (args.angular_stiffness,args.angular_damping,args.max_force)):
    parser.error("Drive settings must be finite nonnegative numbers")
if not 0 <= args.solver_iterations <= 255:
    parser.error("solver-iterations must be between 0 and 255 (zero uses the authored setting)")
sys.argv = [sys.argv[0]]

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "extra_args": [
    "--portable-root", str(Path.home() / "CellCultureLabRuntime"),
    "--/app/settings/persistent=false", "--/renderer/multiGpu/enabled=false",
]})
import omni.usd
import omni.physx
import omni.physics.tensors as tensors
import numpy as np
from pxr import UsdPhysics, PhysxSchema, Gf

ctx = omni.usd.get_context()
iface = omni.physx.get_physx_interface()
rows = []
report = dict(scene=str(Path(args.scene).resolve()), sensor="RigidBodyView.get_transforms",
              quaternion_order="xyzw", mode="injected_drive" if args.apply_drive else "as_authored",
              dt=args.dt, settle_steps=args.settle, run_steps=args.run,
              contacts=not args.no_contacts, gravity="zero" if args.zero_gravity else "authored",
              physics_device="CPU", rows=rows, complete=False)
report["zero_velocities_after_settle"] = args.zero_initial_velocities
report["test_drive"] = dict(angular_stiffness=args.angular_stiffness,angular_damping=args.angular_damping,
                             max_force=args.max_force,solver_iterations=args.solver_iterations)

def attr(prim, name, default=None):
    value = prim.GetAttribute(name).Get()
    return default if value is None else value

def quat_xyzw(q):
    return [*q.GetImaginary(), q.GetReal()]

def limit(p, name):
    value = attr(p, name)
    return value if value is not None and math.isfinite(value) else None

def describe(p):
    kind = ("RevoluteJoint" if p.IsA(UsdPhysics.RevoluteJoint) else
            "PrismaticJoint" if p.IsA(UsdPhysics.PrismaticJoint) else p.GetTypeName())
    b0 = p.GetRelationship("physics:body0").GetTargets()
    b1 = p.GetRelationship("physics:body1").GetTargets()
    mode = "angular" if kind == "RevoluteJoint" else "linear"
    return dict(path=str(p.GetPath()), kind=kind, mode=mode,
                recognized_joint=bool(p.IsA(UsdPhysics.Joint)),
                body0=str(b0[0]) if b0 else None, body1=str(b1[0]) if b1 else None,
                local_pos0=list(attr(p, "physics:localPos0", Gf.Vec3f(0))),
                local_pos1=list(attr(p, "physics:localPos1", Gf.Vec3f(0))),
                local_rot0=quat_xyzw(attr(p, "physics:localRot0", Gf.Quatf(1))),
                local_rot1=quat_xyzw(attr(p, "physics:localRot1", Gf.Quatf(1))),
                axis=str(attr(p, "physics:axis", "X")),
                lower=limit(p, "physics:lowerLimit"), upper=limit(p, "physics:upperLimit"),
                authored_drive=bool(p.HasAPI(UsdPhysics.DriveAPI, mode)))

def open_stage():
    iface.reset_simulation()
    tensors.reset()
    if not ctx.open_stage(str(Path(args.scene).resolve())):
        raise RuntimeError("Cannot open stage: " + args.scene)
    st = ctx.get_stage()
    st.SetEditTarget(st.GetSessionLayer())
    scenes = [p for p in st.Traverse() if p.IsA(UsdPhysics.Scene)]
    if not scenes:
        scenes = [UsdPhysics.Scene.Define(st, "/physicsScene").GetPrim()]
    scene = scenes[0]
    for extra in scenes[1:]:
        extra.SetActive(False)
    ps = PhysxSchema.PhysxSceneAPI.Apply(scene)
    ps.CreateEnableGPUDynamicsAttr(False)
    ps.CreateBroadphaseTypeAttr("MBP")
    ps.CreateTimeStepsPerSecondAttr(round(1/args.dt))
    report["physics_time_steps_per_second"] = ps.GetTimeStepsPerSecondAttr().Get()
    if args.zero_gravity:
        UsdPhysics.Scene(scene).CreateGravityMagnitudeAttr(0.0)
    for p in st.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).CreateSimulationOwnerRel().SetTargets([scene.GetPath()])
            if args.solver_iterations:
                PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateSolverPositionIterationCountAttr(args.solver_iterations)
                PhysxSchema.PhysxRigidBodyAPI(p).CreateSolverVelocityIterationCountAttr(args.solver_iterations)
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateSimulationOwnerRel().SetTargets([scene.GetPath()])
            if args.no_contacts:
                UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
    report["active_scene"] = str(scene.GetPath())
    report["session_overrides"] = ["single scene with explicit owners", "CPU dynamics"]
    return st

def step(n, offset=0):
    for i in range(n):
        iface.update_simulation(args.dt, (offset+i)*args.dt)

def snapshot(bodies):
    data = bodies.get_transforms().copy()
    if data.shape != (bodies.count, 7) or bodies.count == 0:
        raise RuntimeError("Empty or malformed rigid body view")
    return {path: validate_pose(pose.tolist()) for path, pose in zip(bodies.prim_paths, data)}

def body_view(st, view):
    paths = [str(p.GetPath()) for p in st.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if not paths:
        raise RuntimeError("Stage has no rigid bodies")
    return view.create_rigid_body_view(paths)

def coordinate(desc, poses):
    identity = [0, 0, 0, 0, 0, 0, 1]
    return joint_coordinate(desc["kind"], poses[desc["body0"]] if desc["body0"] else identity,
                            poses[desc["body1"]] if desc["body1"] else identity,
                            desc["local_pos0"], desc["local_pos1"],
                            desc["local_rot0"], desc["local_rot1"], desc["axis"])

def delta(a, b, kind):
    return (a-b+180)%360-180 if kind == "RevoluteJoint" else a-b

def constraint(desc, poses):
    identity = [0,0,0,0,0,0,1]
    return joint_constraint_error(desc["kind"],poses[desc["body0"]] if desc["body0"] else identity,
        poses[desc["body1"]] if desc["body1"] else identity,desc["local_pos0"],desc["local_pos1"],
        desc["local_rot0"],desc["local_rot1"],desc["axis"])

def stable_velocity(run):
    return all(all(math.isfinite(x) for x in v)
               and math.sqrt(sum(x*x for x in v[:3])) < 0.001
               and math.sqrt(sum(x*x for x in v[3:])) < 0.01
               for v in run["baseline_velocities"])

def same_baseline(runs):
    first = next(iter(runs.values()))["baseline_poses"]
    for run in runs.values():
        for path, pose in run["baseline_poses"].items():
            before = first[path]
            if math.dist(before[:3], pose[:3]) > 0.0001:
                return False
            dot = min(1.0, abs(sum(a*b for a,b in zip(before[3:],pose[3:]))))
            if 2*math.acos(dot) > 0.001:
                return False
    return True

def trial(desc, plan, target):
    st = open_stage()
    p = st.GetPrimAtPath(desc["path"])
    drive = UsdPhysics.DriveAPI.Apply(p, desc["mode"]) if args.apply_drive else UsdPhysics.DriveAPI(p, desc["mode"])
    if args.apply_drive:
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(args.angular_stiffness if desc["mode"] == "angular" else 30000.0)
        drive.CreateDampingAttr(args.angular_damping if desc["mode"] == "angular" else 3000.0)
        drive.CreateMaxForceAttr(args.max_force)
    drive.CreateTargetVelocityAttr(0.0)
    drive.CreateTargetPositionAttr(plan["center"])
    iface.start_simulation()
    step(1)
    view = tensors.create_simulation_view("numpy")
    try:
        bodies = body_view(st, view)
        start_poses = snapshot(bodies)
        start = coordinate(desc, start_poses)
        step(args.settle, 1)
        if args.zero_initial_velocities:
            bodies.set_velocities(np.zeros((bodies.count,6),dtype=np.float32),
                                  np.arange(bodies.count,dtype=np.uint32))
        base_poses = snapshot(bodies)
        base = coordinate(desc, base_poses)
        # Record velocity BEFORE the command, not just the final state.
        base_vel = bodies.get_velocities().copy()
        drive.GetTargetPositionAttr().Set(target)
        step(args.run, args.settle+1)
        end_poses = snapshot(bodies)
        end = coordinate(desc, end_poses)
        indices = [i for i,path in enumerate(bodies.prim_paths) if path in (desc["body0"], desc["body1"])]
        return dict(target=target, start=start, baseline=base, final=end,
                    change=delta(end,base,desc["kind"]), body_count=bodies.count,
                    initial_poses={k:start_poses[k] for k in (desc["body0"],desc["body1"]) if k},
                    baseline_poses={k:base_poses[k] for k in (desc["body0"],desc["body1"]) if k},
                    final_poses={k:end_poses[k] for k in (desc["body0"],desc["body1"]) if k},
                    baseline_velocities=base_vel[indices].tolist(),
                    endpoint_constraints=[constraint(desc,base_poses),constraint(desc,end_poses)])
    finally:
        view.invalidate()
        iface.reset_simulation()

def flush_report():
    print("REPORT_JSON " + json.dumps(report, ensure_ascii=True, allow_nan=False), flush=True)
    try:
        dest = Path(args.output)
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    except OSError as error:
        print("REPORT_FILE_ERROR " + str(error), flush=True)

try:
    st = open_stage()
    report["source_layers_sha256"] = {
        layer.realPath:hashlib.sha256(Path(layer.realPath).read_bytes()).hexdigest()
        for layer in st.GetUsedLayers() if layer.realPath and Path(layer.realPath).is_file()
    }
    descriptions = [describe(p) for p in st.Traverse() if p.GetTypeName() in ("RevoluteJoint", "PrismaticJoint", "PhysicsRevoluteJoint", "PhysicsPrismaticJoint")
                    and any(term in str(p.GetPath()) for term in args.filter.split("|"))]
    report["joint_count"] = len(descriptions)
    if not descriptions:
        raise RuntimeError("No matching joints; zero coverage is not a pass")
    iface.start_simulation()
    step(1)
    view = tensors.create_simulation_view("numpy")
    bodies = body_view(st, view)
    poses = snapshot(bodies)
    report["runtime_body_count"] = bodies.count
    for d in descriptions:
        try:
            d["initial_body_poses"] = {path:poses[path] for path in (d["body0"],d["body1"]) if path}
            d["pose_status"] = "OK"
        except Exception as error:
            d["pose_status"] = "READ_ERROR"
            d["sensor_error"] = repr(error)
        try:
            d["initial_runtime_coordinate"] = coordinate(d, poses)
            d["sensor_status"] = "OK"
        except ValueError as error:
            d["sensor_status"] = "COORDINATE_UNDEFINED"
            d["sensor_error"] = repr(error)
        except Exception as error:
            d["sensor_status"] = "READ_ERROR"
            d["sensor_error"] = repr(error)
    view.invalidate()
    iface.reset_simulation()
    for desc in descriptions:
        row = dict(joint=desc)
        rows.append(row)
        try:
            if not desc["recognized_joint"]:
                row["status"] = "UNRECOGNIZED_JOINT_SCHEMA"
            elif desc.get("pose_status") == "READ_ERROR":
                row["status"] = "READ_ERROR"
            elif not args.apply_drive and not desc["authored_drive"]:
                row["status"] = "MISSING_DRIVE_API"
            elif desc["sensor_status"] != "OK":
                row["status"] = desc["sensor_status"]
            else:
                plan = target_plan(desc["kind"],desc["lower"],desc["upper"])
                row["plan"] = plan
                runs = {key:trial(desc,plan,plan[key]) for key in ("positive","negative","center")}
                row["trials"] = runs
                tol = 1.0 if desc["kind"] == "RevoluteJoint" else 0.001
                baselines = [v["baseline"] for v in runs.values()]
                if not same_baseline(runs) or max(abs(delta(v,baselines[0],desc["kind"])) for v in baselines) > tol/10:
                    row["status"] = "RESET_MISMATCH"
                elif any(abs(delta(v,plan["center"],desc["kind"])) > tol for v in baselines):
                    row["status"] = "REFERENCE_NOT_REACHED"
                elif not all(stable_velocity(r) for r in runs.values()) or abs(runs["center"]["change"]) > tol:
                    row["status"] = "UNSTABLE_REFERENCE"
                elif any(c["position_m"]>.0001 or c["angle_deg"]>1.0
                         for run in runs.values() for c in run["endpoint_constraints"]):
                    row["status"] = "CONSTRAINT_VIOLATION"
                elif runs["positive"]["change"] > tol and runs["negative"]["change"] < -tol:
                    row["status"] = "DIRECTION_CONFIRMED"
                else:
                    row["status"] = "NO_BIDIRECTIONAL_RESPONSE"
        except ValueError as error:
            row.update(status="INVALID_TARGET_OR_POSE",error=repr(error))
        except Exception as error:
            row.update(status="READ_ERROR",error=repr(error))
        print("JOINT_RESULT " + json.dumps(row,ensure_ascii=True,allow_nan=False),flush=True)
    report["summary"] = dict(Counter(r["status"] for r in rows))
    report["complete"] = True
except Exception as error:
    report["fatal_error"] = repr(error)
    traceback.print_exc()
finally:
    flush_report()
    app.close(wait_for_replicator=False)

