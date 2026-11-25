"""Small independent calibration for the PhysX tensor pose sensor (CPU, no contacts)."""
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "runtime_validation"
OUT.mkdir(parents=True, exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "extra_args": [
    "--portable-root", str(Path.home() / "CellCultureLabRuntime"),
    "--/app/settings/persistent=false", "--/renderer/multiGpu/enabled=false",
]})
import omni.usd
import omni.physx
import omni.timeline
import omni.physics.tensors as tensors
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf

results = []
try:
    for drive_enabled, target in [(True, 20.0), (True, -20.0), (False, 20.0)]:
        omni.timeline.get_timeline_interface().stop()
        tensors.reset()
        omni.usd.get_context().new_stage()
        st = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(st, 1.0)
        UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
        scene = UsdPhysics.Scene.Define(st, "/physicsScene")
        scene.CreateGravityMagnitudeAttr(0.0)
        ps = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        ps.CreateEnableGPUDynamicsAttr(False)
        ps.CreateBroadphaseTypeAttr("MBP")
        for name, kin in [("Base", True), ("Arm", False)]:
            p = UsdGeom.Cube.Define(st, "/" + name)
            p.CreateSizeAttr(0.1)
            rb = UsdPhysics.RigidBodyAPI.Apply(p.GetPrim())
            rb.CreateKinematicEnabledAttr(kin)
            mass = UsdPhysics.MassAPI.Apply(p.GetPrim())
            mass.CreateMassAttr(1.0)
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(0.01))
        j = UsdPhysics.RevoluteJoint.Define(st, "/Hinge")
        j.CreateBody0Rel().SetTargets(["/Base"])
        j.CreateBody1Rel().SetTargets(["/Arm"])
        j.CreateAxisAttr("Z")
        j.CreateLowerLimitAttr(-60.0)
        j.CreateUpperLimitAttr(60.0)
        if drive_enabled:
            dr = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
            dr.CreateTypeAttr("force")
            dr.CreateStiffnessAttr(10.0)
            dr.CreateDampingAttr(2.0)
            dr.CreateTargetPositionAttr(target)
        iface = omni.physx.get_physx_interface()
        iface.start_simulation()
        iface.update_simulation(1.0 / 120.0, 0.0)
        view = tensors.create_simulation_view("numpy")
        bodies = view.create_rigid_body_view("/Arm")
        initial = bodies.get_transforms().copy().tolist()
        for i in range(1, 241):
            iface.update_simulation(1.0 / 120.0, i / 120.0)
        final = bodies.get_transforms().copy().tolist()
        q = final[0][3:]
        angle = math.degrees(2 * math.atan2(q[2], q[3]))
        result = dict(drive_enabled=drive_enabled, target_deg=target,
                      body_paths=bodies.prim_paths, initial=initial, final=final,
                      measured_deg=angle)
        results.append(result)
        print("CALIBRATION " + json.dumps(result), flush=True)
        view.invalidate()
        iface.reset_simulation()
    passed = (abs(results[0]["measured_deg"] - 20) < 1 and
              abs(results[1]["measured_deg"] + 20) < 1 and
              abs(results[2]["measured_deg"]) < 0.1)
    report = {"passed": passed, "quaternion_order": "xyzw", "results": results}
    print("REPORT_JSON " + json.dumps(report), flush=True)
    try:
        (OUT / "sensor_calibration.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    except OSError as error:
        print("REPORT_FILE_ERROR " + str(error), flush=True)
    print("CALIBRATION_PASS", passed, flush=True)
    sys.stdout.flush()
except Exception:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close(wait_for_replicator=False)
