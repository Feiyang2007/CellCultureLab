"""Audit GPU capacity authoring; optionally test a session-only repair in Isaac.

Examples (use Isaac's python.exe):
    python -u scripts/validate_gpu_capacity.py v07
    python -u scripts/validate_gpu_capacity.py test --fix
    python -u scripts/validate_gpu_capacity.py v06 --fix --run --mode gpu --steps 240

Without --run this uses bare USD and does not launch Isaac. Source layers are
never saved. A repair is authored in an anonymous session and exported as a new
small override layer under outputs/runtime_validation. Composed USD values are
configuration evidence, not a readback of PhysX's allocated GPU buffers.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import traceback
import time
from runtime_resources import snapshot as resource_snapshot


ROOT = Path(__file__).resolve().parents[1]
ISAAC = Path("D:/IsaacSim5/env/Lib/site-packages/isaacsim")
OUTPUT = ROOT / "outputs" / "runtime_validation"
CAPACITY_NAMES = (
    "physxScene:gpuTotalAggregatePairsCapacity",
    "physxScene:gpuFoundLostAggregatePairsCapacity",
)
OBSERVED_NAMES = CAPACITY_NAMES + (
    "physxScene:gpuMaxRigidContactCount",
    "physxScene:gpuMaxRigidPatchCount",
    "physxScene:gpuCollisionStackSize",
    "physxScene:gpuMaxRigidContactCountEstimate",
    "physxScene:gpuMaxPatchCountEstimate",
    "physxScene:enableGPUDynamics",
    "physxScene:broadphaseType",
    "physics:gravityDirection",
    "physics:gravityMagnitude",
)
DLL_HANDLES = []


def json_value(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    try:
        return [json_value(v) for v in value]
    except TypeError:
        return str(value)


def bare_usd():
    usd_dir = next((ISAAC / "extscache").glob("omni.usd.libs-*"))
    sys.path.insert(0, str(usd_dir))
    for folder in (usd_dir / "bin", usd_dir / "bin/deps", ISAAC / "kit"):
        DLL_HANDLES.append(os.add_dll_directory(str(folder)))


def applied_tokens(prim):
    op = prim.GetMetadata("apiSchemas")
    return list(op.ApplyOperations([])) if op else []


def schema_defaults(Sdf):
    """Read this installation's schema without loading its native plugin."""
    schema_dir = next((ISAAC / "extscache").glob("omni.usd.schema.physx-*"))
    path = schema_dir / "plugins/PhysxSchema/resources/generatedSchema.usda"
    layer = Sdf.Layer.FindOrOpen(str(path))
    prim = layer.GetPrimAtPath("/PhysxSceneAPI")
    return str(path), {
        name: {"type": str(prim.attributes[name].typeName),
               "default": json_value(prim.attributes[name].default)}
        for name in OBSERVED_NAMES if name in prim.attributes
    }


def audit(stage, defaults, UsdPhysics):
    scenes, owners, owner_rows = [], Counter(), []
    composition_errors = []
    for prim in stage.Traverse():
        composition_errors.extend(str(error) for error in prim.GetPrimIndex().localErrors)
        if prim.GetTypeName() == "PhysicsScene":
            tokens = applied_tokens(prim)
            attrs = {}
            for name in OBSERVED_NAMES:
                attr = prim.GetAttribute(name)
                value = attr.Get() if attr else None
                fallback = defaults.get(name) if "PhysxSceneAPI" in tokens else None
                attrs[name] = {
                    "composed_type": str(attr.GetTypeName()) if attr else None,
                    "composed_value": json_value(value),
                    "authored": bool(attr and attr.HasAuthoredValueOpinion()),
                    "installed_schema": defaults.get(name),
                    "schema_fallback_if_plugin_loaded": fallback if value is None else None,
                    "type_matches_schema": (
                        str(attr.GetTypeName()) == defaults[name]["type"]
                        if attr and name in defaults else None
                    ),
                }
            scenes.append({"path": str(prim.GetPath()), "api_schemas": tokens,
                           "attributes": attrs})
        rigid = prim.HasAPI(UsdPhysics.RigidBodyAPI)
        collision = prim.HasAPI(UsdPhysics.CollisionAPI)
        if rigid or collision:
            rel = prim.GetRelationship("physics:simulationOwner")
            targets = [str(v) for v in rel.GetTargets()] if rel else []
            owners[",".join(targets) or "<default:first-scene>"] += 1
            owner_rows.append({"path": str(prim.GetPath()), "rigid_body": rigid,
                               "collider": collision, "targets": targets})
    return {
        "scene_count": len(scenes), "scenes": scenes,
        "default_scene_from_traversal": scenes[0]["path"] if scenes else None,
        "body_or_collider_count": len(owner_rows), "owner_counts": dict(owners),
        "owners": owner_rows,
        "composition_errors": list(dict.fromkeys(composition_errors)),
        "note": "USD composition/schema evidence; no native buffer-capacity readback.",
    }


def repair(stage, capacity, Usd, UsdPhysics, Sdf, PhysxSchema=None, extra_capacities=None):
    scenes = [p for p in stage.Traverse() if p.GetTypeName() == "PhysicsScene"]
    if not scenes:
        raise RuntimeError("No PhysicsScene: refusing to invent a different scene.")
    selected = scenes[0]
    scene_path = selected.GetPath()
    session = stage.GetSessionLayer()
    with Usd.EditContext(stage, session):
        for other in scenes[1:]:
            other.SetActive(False)
        if PhysxSchema is not None:
            PhysxSchema.PhysxSceneAPI.Apply(selected)
        else:
            tokens = applied_tokens(selected)
            if "PhysxSceneAPI" not in tokens:
                tokens.append("PhysxSceneAPI")
            selected.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(tokens))
        # Prim.CreateAttribute can retain a weaker layer's erroneous int type.
        # A stronger explicit Sdf spec overrides the type without touching it.
        spec = Sdf.CreatePrimInLayer(session, scene_path)
        values = {name:capacity for name in CAPACITY_NAMES}
        values.update(extra_capacities or {})
        for name, value in values.items():
            attribute = Sdf.AttributeSpec(spec, name, Sdf.ValueTypeNames.UInt,
                                          Sdf.VariabilityVarying, False)
            attribute.default = value
            composed = selected.GetAttribute(name)
            if composed.GetTypeName() != Sdf.ValueTypeNames.UInt or composed.Get() != value:
                raise RuntimeError("Session uint override did not compose: " + name)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.CreateRelationship("physics:simulationOwner", custom=False).SetTargets([scene_path])
    # Gravity is deliberately untouched, including its USD fallback values.
    return str(scene_path)


def export_overrides(stage, source, destination, Sdf, Usd):
    if destination.resolve() == source.resolve():
        raise RuntimeError("Refusing to overwrite input stage")
    layer = Sdf.Layer.CreateAnonymous("capacity_overrides.usda")
    layer.TransferContent(stage.GetSessionLayer())
    layer.subLayerPaths = [source.as_posix()]
    if not layer.Export(str(destination)):
        raise RuntimeError("Could not export validation overrides")
    reopened = Usd.Stage.Open(str(destination))
    if not reopened:
        raise RuntimeError("Exported validation stage did not reopen")
    scenes = [p for p in reopened.Traverse() if p.GetTypeName() == "PhysicsScene"]
    if len(scenes) != 1:
        raise RuntimeError("Exported stage does not have exactly one active PhysicsScene")
    for name in CAPACITY_NAMES:
        attribute = scenes[0].GetAttribute(name)
        expected = stage.GetPrimAtPath(scenes[0].GetPath()).GetAttribute(name).Get()
        if attribute.GetTypeName() != Sdf.ValueTypeNames.UInt or attribute.Get() != expected:
            raise RuntimeError("Exported stage lost capacity override: " + name)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", nargs="?", default="v06",
                        help="v06, v07, test, or a USD scene path")
    parser.add_argument("--fix", action="store_true", help="Repair only the anonymous session")
    parser.add_argument("--run", action="store_true", help="Start Isaac and step physics")
    parser.add_argument("--stdout-only", action="store_true", help="Emit a compact report without writing files")
    parser.add_argument("--mode", choices=("cpu", "gpu"), default="gpu")
    parser.add_argument("--steps", type=int, default=240)
    parser.add_argument("--dt", type=float, default=1.0 / 60.0)
    parser.add_argument("--capacity", type=int, default=8192)
    parser.add_argument("--patch-capacity", type=int)
    parser.add_argument("--collision-stack-size", type=int, help="Collision stack bytes; use measured overflow demand")
    parser.add_argument("--max-commit-fraction",type=float,default=.94,
                        help="Abort between progress samples if system commit reaches this fraction")
    parser.add_argument("--portable-root", type=Path,
                        default=Path.home() / "CellCultureLabRuntime")
    args = parser.parse_args()
    if args.steps < 1 or not math.isfinite(args.dt) or args.dt <= 0 or not 1 <= args.capacity <= 2**32 - 1:
        parser.error("steps, dt, capacity must be positive; capacity must fit uint32")
    if abs(1/args.dt-round(1/args.dt)) > 1e-6:
        parser.error("dt must correspond to an integer PhysicsScene timeStepsPerSecond")
    if any(v is not None and not 1 <= v <= 2**32-1 for v in (args.patch_capacity,args.collision_stack_size)):
        parser.error("Optional capacities must be positive uint32 values")
    if not args.fix and (args.patch_capacity is not None or args.collision_stack_size is not None):
        parser.error("Optional capacity overrides require --fix")
    if not 0 < args.max_commit_fraction <= 1:
        parser.error("max-commit-fraction must be between zero and one")
    aliases = {"v06": "biolab_main_v06.usda", "v07": "biolab_main_v07.usda",
               "test": "test_instruments4.usda"}
    args.source = (ROOT / "assets/scenes" / aliases[args.scene]
                   if args.scene in aliases else Path(args.scene).resolve())
    if not args.source.is_file():
        parser.error("Scene does not exist: " + str(args.source))
    return args


def main():
    args = parse_args()
    sys.argv = [sys.argv[0]]  # Validation flags must not be interpreted by Kit.
    OUTPUT.mkdir(parents=True, exist_ok=True)
    tag = "gpu_capacity_{}_{}_{}".format(
        args.source.stem, args.mode, "fixed" if args.fix else "baseline")
    if args.capacity != 8192 or args.patch_capacity or args.collision_stack_size:
        tag += "_cap{}_patch{}_stack{}".format(args.capacity,args.patch_capacity or 0,args.collision_stack_size or 0)
    report_path = OUTPUT / (tag + ".json")
    native_log_path = OUTPUT / (tag + ".kit.log")
    derived_path = OUTPUT / (tag + ".usda")
    report = {"source": str(args.source), "timestamp_utc": datetime.now(timezone.utc).isoformat(),
              "requested_mode": args.mode, "fix_requested": args.fix,
              "runtime_requested": args.run, "runtime_validated": False,
              "runtime_completed": False,
              "status": "starting", "steps_completed": 0, "steps_requested": args.steps,
              "dt": args.dt, "log_capture_active": False, "errors": [],
              "capacity_messages": [], "native_log": str(native_log_path) if args.run and not args.stdout_only else None}
    app = None
    iface = None
    exit_code = 0
    report["resource_samples"] = [resource_snapshot("before_startup")]
    report["physics_update_wall_seconds"] = 0.0
    report["max_commit_fraction"] = args.max_commit_fraction

    def save():
        if args.stdout_only:
            return
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False,
                                         allow_nan=False), encoding="utf-8")

    save()  # Remains inspectable if startup fails before Python regains control.
    try:
        if args.run:
            print("STARTING_ISAAC " + str(args.portable_root), flush=True)
            from isaacsim import SimulationApp
            app = SimulationApp({"headless": True, "extra_args": [
                "--portable-root", str(args.portable_root),
                "--/app/settings/persistent=false", "--/renderer/multiGpu/enabled=false",
                *([] if args.stdout_only else ["--/log/file=" + str(native_log_path)]),
            ]})
            print("RUNTIME_PHASE startup_complete", flush=True)
            report["resource_samples"].append(resource_snapshot("after_startup"))
            import omni.usd
            import omni.physx
            from pxr import PhysxSchema
            # Capture native logs in the parent process, not a Python callback.
            # PhysX worker logging during initialization can otherwise contend
            # with the Python call's GIL. Completion alone is not log validation.
            report["log_review_required"] = True
        else:
            bare_usd()
            PhysxSchema = None
        from pxr import Usd, UsdPhysics, Sdf
        schema_path, defaults = schema_defaults(Sdf)
        report["installed_physx_schema"] = schema_path
        if args.run:
            ctx = omni.usd.get_context()
            print("RUNTIME_PHASE open_stage_begin", flush=True)
            if not ctx.open_stage(str(args.source)):
                raise RuntimeError("Could not open source stage")
            stage = ctx.get_stage()
            print("RUNTIME_PHASE open_stage_complete", flush=True)
        else:
            session = Sdf.Layer.CreateAnonymous("capacity_session.usda")
            stage = Usd.Stage.Open(Sdf.Layer.FindOrOpen(str(args.source)), session)
        report["before"] = audit(stage, defaults, UsdPhysics)
        report["source_layer_metadata"] = dict(stage.GetRootLayer().customLayerData)
        if args.fix:
            extras = {}
            if args.patch_capacity:
                extras["physxScene:gpuMaxRigidPatchCount"] = args.patch_capacity
            if args.collision_stack_size:
                extras["physxScene:gpuCollisionStackSize"] = args.collision_stack_size
            report["selected_scene"] = repair(stage, args.capacity, Usd, UsdPhysics, Sdf, PhysxSchema,extras)
            if not args.stdout_only:
                export_overrides(stage, args.source, derived_path, Sdf, Usd)
                report["derived_stage"] = str(derived_path)
        if args.run:
            stage.SetEditTarget(stage.GetSessionLayer())
            report["physics_time_steps_per_second"] = {}
            for prim in stage.Traverse():
                if prim.IsA(UsdPhysics.Scene):
                    ps = PhysxSchema.PhysxSceneAPI.Apply(prim)
                    ps.CreateTimeStepsPerSecondAttr(round(1/args.dt))
                    report["physics_time_steps_per_second"][str(prim.GetPath())] = ps.GetTimeStepsPerSecondAttr().Get()
        report["after"] = audit(stage, defaults, UsdPhysics)
        report["status"] = "static_audit_complete"
        save()
        if args.run:
            iface = omni.physx.get_physx_interface()
            override = 1 if args.mode == "gpu" else 0
            iface.overwrite_gpu_setting(override)
            report["physics_gpu_override"] = iface.get_overwrite_gpu_setting()
            if report["physics_gpu_override"] != override:
                raise RuntimeError("PhysX CPU/GPU override readback mismatch")
            report["status"] = "simulating"
            save()
            print("RUNTIME_PHASE start_simulation_begin", flush=True)
            report["resource_samples"].append(resource_snapshot("before_start_simulation"))
            iface.start_simulation()
            print("RUNTIME_PHASE start_simulation_complete", flush=True)
            for step in range(args.steps):
                update_begin = time.perf_counter()
                iface.update_simulation(args.dt, step * args.dt)
                report["physics_update_wall_seconds"] += time.perf_counter()-update_begin
                report["steps_completed"] = step + 1
                if (step + 1) % 60 == 0:
                    sample=resource_snapshot("step_"+str(step+1))
                    report["resource_samples"].append(sample)
                    save()
                    print("PHYSICS_STEPS " + str(step + 1), flush=True)
                    if sample.get("system_commit_limit_bytes",0) and sample["system_commit_bytes"]/sample["system_commit_limit_bytes"] >= args.max_commit_fraction:
                        report["resource_guard_triggered"] = True
                        raise RuntimeError("System commit reached configured guard; run stopped before further steps")
            report["runtime_completed"] = True
            report["runtime_validated"] = False
            report["capacity_overflow_observed"] = None
            report["full_scene_validated"] = False
            report["status"] = "runtime_complete_external_log_review_required"
            report["interpretation"] = (
                "Completed the requested run; composed capacity values are not native buffer readback. "
                "Absence of overflow during this run does not validate all contacts or future tasks. "
                + ("CPU mode does not validate GPU capacity." if args.mode == "cpu" else "")
            )
            exit_code = 0 if report["runtime_validated"] else 2
    except Exception:
        report["status"] = "failed"
        report["exception"] = traceback.format_exc()
        traceback.print_exc()
        exit_code = 1
    finally:
        # Flush the artifact before shutdown, which may itself fail in this build.
        save()
        print("CAPACITY_REPORT " + ("stdout_only" if args.stdout_only else str(report_path)), flush=True)
        if iface is not None:
            try:
                iface.reset_simulation()
            except Exception as exc:
                report["reset_error"] = str(exc)
                exit_code = 1
        save()
        sys.stdout.flush()
        sys.stderr.flush()
        compact = dict(report)
        for key in ("before", "after"):
            if key in compact:
                compact[key] = {k:v for k,v in compact[key].items() if k != "owners"}
        print("REPORT_JSON " + json.dumps(compact, ensure_ascii=True, allow_nan=False), flush=True)
        if app is not None:
            app.close(wait_for_replicator=False)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
