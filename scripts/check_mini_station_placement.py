"""Verify native mini rigid poses after station translation/rotation.

Checks runtime tensor poses against composed USD poses, not just joint angles.
Source layers are never saved; CPU, authored collisions, zero gravity for isolation.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument("--scene",default=str(ROOT/"outputs/runtime_validation/v06_native_mini_pilot.usda"))
p.add_argument("--root-prim",default="/World/NativeMiniPilot")
a=p.parse_args()
sys.argv=[sys.argv[0]]
from isaacsim import SimulationApp
app=SimulationApp({"headless":True,"extra_args":["--portable-root",str(Path.home()/"CellCultureLabRuntime"),
    "--/app/settings/persistent=false","--/renderer/multiGpu/enabled=false"]})
import omni.usd
import omni.physx
import omni.physics.tensors as tensors
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf

report=dict(complete=False,passed=False,scope="transformed mini station placement; not task acceptance",
    source=str(Path(a.scene).resolve()),root=a.root_prim,physics_hz=120,steps=120,original_version_matched=False)
iface=omni.physx.get_physx_interface()
view=None
try:
    ctx=omni.usd.get_context()
    if not ctx.open_stage(str(Path(a.scene).resolve())):
        raise RuntimeError("Stage open failed")
    stage=ctx.get_stage()
    report["source_layers_sha256"]={layer.realPath:hashlib.sha256(Path(layer.realPath).read_bytes()).hexdigest()
        for layer in stage.GetUsedLayers() if layer.realPath and Path(layer.realPath).is_file()}
    stage.SetEditTarget(stage.GetSessionLayer())
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            scene=UsdPhysics.Scene(prim)
            scene.CreateGravityMagnitudeAttr(0)
            ps=PhysxSchema.PhysxSceneAPI.Apply(prim)
            ps.CreateEnableGPUDynamicsAttr(False)
            ps.CreateBroadphaseTypeAttr("MBP")
            ps.CreateTimeStepsPerSecondAttr(120)
    iface.overwrite_gpu_setting(0)
    xform=UsdGeom.XformCache()
    expected={}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(a.root_prim)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            m=xform.GetLocalToWorldTransform(prim)
            q=m.ExtractRotationQuat()
            expected[str(prim.GetPath())]=[*m.ExtractTranslation(),*q.GetImaginary(),q.GetReal()]
    if len(expected)!=3:
        raise RuntimeError("Expected exactly three mini source bodies")
    iface.start_simulation()
    iface.update_simulation(1/120,0)
    view=tensors.create_simulation_view("numpy")
    bodies=view.create_rigid_body_view(a.root_prim+"/Bodies/*")
    art=view.create_articulation_view(a.root_prim+"/Joints/world_fixed")
    if art.count!=1 or not art.shared_metatype.fixed_base:
        raise RuntimeError("Expected one native fixed-base articulation")
    rows=[]
    for step in range(1,121):
        if step>1:
            iface.update_simulation(1/120,(step-1)/120)
        poses=dict(zip(bodies.prim_paths,bodies.get_transforms().copy().tolist()))
        for path,want in expected.items():
            actual=poses[path]
            q=actual[3:]
            n=math.sqrt(sum(x*x for x in q))
            dot=min(1,abs(sum(x*y/n for x,y in zip(q,want[3:]))))
            rows.append(dict(step=step,path=path,position_error_m=math.dist(actual[:3],want[:3]),
                rotation_error_deg=math.degrees(2*math.acos(dot))))
    report.update(complete=True,body_count=bodies.count,
        max_position_error_m=max(r["position_error_m"] for r in rows),
        max_rotation_error_deg=max(r["rotation_error_deg"] for r in rows),
        final_poses=poses,expected_poses=expected)
    report["passed"]=report["max_position_error_m"]<.0001 and report["max_rotation_error_deg"]<.01
except Exception as error:
    report["fatal_error"]=repr(error)
    traceback.print_exc()
finally:
    if view is not None:
        view.invalidate()
    iface.reset_simulation()
    print("REPORT_JSON "+json.dumps(report,allow_nan=False),flush=True)
    try:
        (ROOT/"outputs/runtime_validation/mini_station_placement.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    except OSError as error:
        print("REPORT_FILE_ERROR "+str(error),flush=True)
    app.close(wait_for_replicator=False)
