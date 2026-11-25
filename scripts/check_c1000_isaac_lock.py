"""CPU-only physical C1000 latch pilot; never overwrites production assets.

Uses source-derived passive spring/damping and armature, tensor q/v reads,
and an excluded regular FixedJoint as a switchable hard latch. Source soft
equality and knob dry friction differences remain explicit quality gates.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import traceback
from instrument_behavior import C1000LockController,detent_index

ROOT=Path(__file__).resolve().parents[1]
PILOT=ROOT/"outputs/runtime_validation/c1000_isaac_lock_pilot"
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--case",choices=("locked","unlocked","matrix","cycle","diag_idle","diag_reverse","diag_lever"),default="matrix")
parser.add_argument("--output",type=Path,default=Path.home()/"AppData/LocalLow/CellCultureLabReference/c1000_isaac_lock.json")
parser.add_argument("--stdout-only",action="store_true")
parser.add_argument("--latch-backend",choices=("fixed","limits"),default="fixed")
parser.add_argument("--disable-viewport",action="store_true",help="Supported SimulationApp viewport switch; physics unchanged")
parser.add_argument("--physics-only",action="store_true",help="Minimal Kit dependency graph; no camera/render validation")
args=parser.parse_args()
home=Path.home()/"AppData/LocalLow/CellCultureLabReference/c1000_isaac_runtime"
home.mkdir(parents=True,exist_ok=True)
tmp=home/"tmp";tmp.mkdir(exist_ok=True)
os.environ["TEMP"]=os.environ["TMP"]=str(tmp)
with tempfile.TemporaryFile(dir=tmp):pass
meta=json.loads((PILOT/"build.json").read_text(encoding="utf-8"))
reference_path=Path(meta["reference"])
if hashlib.sha256(reference_path.read_bytes()).hexdigest()!=meta["reference_sha256"]:raise RuntimeError("Reference changed")
if hashlib.sha256(Path(meta["asset"]).read_bytes()).hexdigest()!=meta["asset_sha256"]:raise RuntimeError("Asset changed")
ref=json.loads(reference_path.read_text(encoding="utf-8"))
report=dict(runtime_completed=False,runtime_validated=False,case=args.case,scope="C1000 physical latch functional pilot, not complete source-equivalent dynamics or task",
    asset_sha256=meta["asset_sha256"],reference_sha256=meta["reference_sha256"],
    source_plugin_native_platform=False,latch_backend=meta["latch_backend"] if args.latch_backend=="fixed" else "tensor DOF 1e-5 rad limit band; hard scalar lock approximation",
    unresolved_frictionloss=meta["unresolved_frictionloss"],physics_hz=1000,gravity=[0,0,0],
    production_ready=False,full_task_validated=False,viewport_updates_disabled=args.disable_viewport,physics_only_app=args.physics_only,rows=[],errors=[],
    thresholds=dict(locked_max_displacement_rad=.005,unlocked_min_displacement_rad=.01,
        forbidden_position_m=.0001,forbidden_rotation_deg=1))
from isaacsim import SimulationApp
if args.physics_only:
    import isaacsim
    from isaacsim import AppFramework
    package=Path(isaacsim.__file__).parent
    argv=[str(ROOT/"scripts/c1000_physics_only.kit"),"--no-window","--portable-root",str(home)]
    for folder in (package/"exts",package/"extscache",package/"kit/exts",package/"kit/extscore"):
        if folder.is_dir():argv.extend(["--ext-folder",str(folder)])
    class PhysicsApp(AppFramework):
        def close(self,**kwargs):super().close()
    app=PhysicsApp(argv=argv)
    for _ in range(3):app.update()
else:
    app=SimulationApp({"headless":True,"disable_viewport_updates":args.disable_viewport,"extra_args":["--portable-root",str(home),
        "--/app/settings/persistent=false","--/renderer/multiGpu/enabled=false"]})
print("RUNTIME_PHASE startup_complete",flush=True)
import numpy as np
import psutil
import omni.usd
import omni.physx
import omni.physics.tensors as tensors
from pxr import UsdPhysics
from runtime_joint_math import joint_constraint_error, validate_pose, quat_mul, quat_normalize, quat_rotate

def fixed_constraint_error(kind, pose0, pose1, pos0, pos1, rot0, rot1, axis):
    if kind != "FixedJoint":
        return joint_constraint_error(kind,pose0,pose1,pos0,pos1,rot0,rot1,axis)
    a,b=validate_pose(pose0),validate_pose(pose1)
    f0=quat_normalize(quat_mul(a[3:],rot0));f1=quat_normalize(quat_mul(b[3:],rot1))
    o0=quat_rotate(a[3:],pos0);o1=quat_rotate(b[3:],pos1)
    return dict(position_m=math.dist([a[i]+o0[i] for i in range(3)],[b[i]+o1[i] for i in range(3)]),
        angle_deg=math.degrees(2*math.acos(min(1.,abs(sum(x*y for x,y in zip(f0,f1)))))))

ctx=omni.usd.get_context();iface=omni.physx.get_physx_interface();view=None
dt=.001


def trial(case):
    global view
    iface.reset_simulation();tensors.reset()
    if not ctx.open_stage(str(PILOT/"scene.usda")):raise RuntimeError("Scene open failed")
    stage=ctx.get_stage();stage.SetEditTarget(stage.GetSessionLayer())
    latch=stage.GetPrimAtPath(meta["latch_path"])
    latch.GetAttribute("physics:jointEnabled").Set(False)
    if args.physics_only:
        from pxr import UsdUtils
        cache=UsdUtils.StageCache.Get();cache.Insert(stage)
        omni.physx.get_physx_simulation_interface().attach_stage(cache.GetId(stage).ToLongInt())
    iface.start_simulation();iface.update_simulation(dt,0)
    view=tensors.create_simulation_view("numpy")
    art=view.create_articulation_view(meta["articulation_root"])
    if art is None or art.count!=1 or art.max_dofs!=3 or not art.shared_metatype.fixed_base:
        raise RuntimeError("Expected one fixed-base three-DOF articulation")
    names=list(art.shared_metatype.dof_names)
    slug=lambda x:x.replace('-','_')
    mapping={j["name"]:next(i for i,n in enumerate(names) if n.endswith('_'+slug(j["name"]))) for j in ref["joints"]}
    li=mapping["lid"];le=mapping["lid-lever"];kn=mapping["lid-force-knob"]
    lj=next(j for j in ref["joints"] if j["name"]=="lid-lever")
    low=next(j for j in ref["joints"] if j["name"]=="lid")["range"][0]
    indices=np.array([0],dtype=np.uint32)
    original_limits=art.get_dof_limits().copy()
    q=np.zeros((1,art.max_dofs),dtype=np.float32);v=q.copy();force=q.copy();targets=q.copy()
    diagnostic=case.startswith("diag_")
    q[0,li]=low;q[0,le]=lj["range"][1] if case=="unlocked" or diagnostic else lj["range"][0]
    art.set_dof_positions(q,indices);art.set_dof_velocities(v,indices)
    gains=dict(stiffnesses=art.get_dof_stiffnesses().copy()[0].tolist(),dampings=art.get_dof_dampings().copy()[0].tolist(),
        armatures=art.get_dof_armatures().copy()[0].tolist())
    for j in ref["joints"]:
        i=mapping[j["name"]]
        for key,field in (("stiffnesses","stiffness"),("dampings","damping"),("armatures","armature")):
            if abs(gains[key][i]-j[field])>1e-5:
                raise RuntimeError("Runtime source-unit parameter mismatch: "+key+" "+j["name"]+" "+str(gains[key][i])+" vs "+str(j[field]))
    controller=C1000LockController(low);enabled=False;events=[];trace=[];phases=[];global_step=0
    bodies=view.create_rigid_body_view("/C1000/Bodies/*")
    hinges=[p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    constraints=[]
    def read():
        qn=art.get_dof_positions().copy()[0];vn=art.get_dof_velocities().copy()[0]
        if not np.isfinite(qn).all() or not np.isfinite(vn).all():raise RuntimeError("Nonfinite state")
        return qn,vn
    def logic(reset=False):
        nonlocal enabled
        if diagnostic:return 1
        qn,_=read()
        state=detent_index(float(qn[le]),lj["range"][0],lj["range"][1],int(lj["user"][0]))
        decision=(controller.reset if reset else controller.update)(float(qn[li]),state,enabled)
        if decision.enabled!=enabled:
            enabled=decision.enabled
            if args.latch_backend=="fixed":
                latch.GetAttribute("physics:jointEnabled").Set(enabled)
            else:
                limits=original_limits.copy()
                if enabled:limits[0,li,:]=[low,low+1e-5]
                art.set_dof_limits(limits,indices)
            events.append(dict(step=global_step,enabled=enabled,lid_rad=float(qn[li]),lever_state=state))
        return state
    def passive_targets():
        qn,_=read()
        scaled=(float(qn[le])-lj["range"][0])/(lj["range"][1]-lj["range"][0])*(int(lj["user"][0])-1)
        index=math.floor(scaled+.5) if scaled>=0 else math.ceil(scaled-.5)
        index=min(int(lj["user"][0])-1,max(0,index))
        targets[0,le]=lj["range"][0]+index*(lj["range"][1]-lj["range"][0])/(int(lj["user"][0])-1)
        art.set_dof_position_targets(targets,indices)
    logic(True);passive_targets()
    def phase(name,torque,steps,lever_state=None,until=None):
        nonlocal global_step
        print("LATCH_PHASE begin "+name+" step="+str(global_step),flush=True)
        q0,v0=read();continuity=True
        if lever_state is not None:
            qp=art.get_dof_positions().copy();vp=art.get_dof_velocities().copy()
            qp[0,le]=lj["range"][lever_state];vp[0,le]=0
            art.set_dof_positions(qp,indices);art.set_dof_velocities(vp,indices)
            qr,vr=read();continuity=qr[li]==q0[li] and vr[li]==v0[li]
            logic();passive_targets()
        startq,_=read();worst=0.;reached=until is None
        for step in range(steps):
            force[:]=0;force[0,li]=torque
            passive_targets();art.set_dof_actuation_forces(force,indices)
            iface.update_simulation(dt,(global_step+1)*dt);global_step+=1
            state=logic();qn,vn=read();worst=max(worst,abs(float(qn[li])-float(startq[li])))
            row=dict(step=global_step,phase=name,q=qn.tolist(),v=vn.tolist(),enabled=enabled,
                bad_locking=controller.bad_locking,lever_state=state,torque_nm=torque)
            trace.append(row)
            if global_step%100==0:
                print("RUNTIME_PHASE "+json.dumps(dict(step=global_step,phase=name,
                    private_bytes=psutil.Process().memory_info().private,lid_q=float(qn[li]),lid_v=float(vn[li]))),flush=True)
            if global_step%25==0:
                poses=dict(zip(bodies.prim_paths,bodies.get_transforms().copy().tolist()))
                xyzw=lambda x:[*x.GetImaginary(),x.GetReal()]
                for prim in hinges+([latch] if enabled and args.latch_backend=="fixed" else []):
                    p0=str(prim.GetRelationship("physics:body0").GetTargets()[0]);p1=str(prim.GetRelationship("physics:body1").GetTargets()[0])
                    err=fixed_constraint_error("FixedJoint" if prim==latch else "RevoluteJoint",poses[p0],poses[p1],
                        list(prim.GetAttribute("physics:localPos0").Get()),list(prim.GetAttribute("physics:localPos1").Get()),
                        xyzw(prim.GetAttribute("physics:localRot0").Get()),xyzw(prim.GetAttribute("physics:localRot1").Get()),"X")
                    constraints.append(err)
            if until is not None and until(row):reached=True;break
        rec=dict(name=name,steps=step+1,condition_reached=reached,lid_qv_continuity_at_fixture_event=bool(continuity),
            max_lid_motion_rad=worst,last=trace[-1])
        phases.append(rec);print("LATCH_PHASE "+json.dumps({k:v for k,v in rec.items() if k!='last'}),flush=True)
        return rec
    if diagnostic:
        if case=="diag_idle":phase("no_lock_zero_input",0.,2400)
        else:
            phase("no_lock_positive",.05,500)
            phase("no_lock_reverse",-.05,1900,lever_state=0 if case=="diag_lever" else None)
        verdict=dict(diagnostic_completed=True,no_latch_activated=not events)
    elif case in ("locked","unlocked"):
        phase(case,.05,500)
        passed=phases[-1]["max_lid_motion_rad"]<.005 if case=="locked" else phases[-1]["max_lid_motion_rad"]>.01
        verdict={case+"_force_response":passed}
    else:
        phase("initial_locked_force",.05,500)
        phase("unlock_and_open",.05,600,lever_state=1,until=lambda r:r['q'][li]>low+.08)
        phase("try_lock_while_open",0.,20,lever_state=0)
        phase("close_after_bad_lock",-.05,1800,until=lambda r:r['q'][li]<=low+.005 and r['v'][li]<0)
        phase("settle_with_bad_lock",-.05,500)
        phase("probe_open_despite_returning_closed",.05,120)
        phase("unlock_to_clear_and_close",-.05,1800,lever_state=1,until=lambda r:r['q'][li]<=low+.005 and r['v'][li]<0)
        phase("settle_before_relock",-.05,500)
        phase("relock_and_force",.05,500,lever_state=0)
        ps={p['name']:p for p in phases};bad=ps['try_lock_while_open']['last'];closed=ps['settle_with_bad_lock']['last'];rel=ps['relock_and_force']['last']
        verdict=dict(no_lid_state_change_at_fixture_events=all(p['lid_qv_continuity_at_fixture_event'] for p in phases),
            conditions_reached=all(p['condition_reached'] for p in phases),
            initial_lock_holds=ps['initial_locked_force']['max_lid_motion_rad']<.005,
            unlock_opens=ps['unlock_and_open']['max_lid_motion_rad']>.01,
            bad_lock_latched_without_constraint=bad['bad_locking'] and not bad['enabled'],
            returning_closed_does_not_reset_bad_latch=closed['bad_locking'] and not closed['enabled'] and closed['q'][li]<=low+.005,
            bad_latch_still_allows_opening=ps['probe_open_despite_returning_closed']['max_lid_motion_rad']>.01,
            relock_after_clear_works=rel['enabled'] and not rel['bad_locking'] and ps['relock_and_force']['max_lid_motion_rad']<.005)
    verdict["joint_constraints_preserved"]=all(e['position_m']<.0001 and e['angle_deg']<1 for e in constraints)
    return dict(case=case,phases=phases,trajectory=trace,latch_events=events,verdict=verdict,
        passed=all(verdict.values()),native_gains=gains,dof_names=names,steps=global_step,
        max_forbidden_position_m=max((e['position_m'] for e in constraints),default=0),
        max_forbidden_rotation_deg=max((e['angle_deg'] for e in constraints),default=0),
        knob_max_abs_q_rad=max(abs(r['q'][kn]) for r in trace),
        knob_max_abs_v_rad_s=max(abs(r['v'][kn]) for r in trace))


try:
    for case in (["locked","unlocked"] if args.case=="matrix" else [args.case]):
        report["rows"].append(trial(case))
        view.invalidate();view=None;iface.reset_simulation()
    report["runtime_completed"]=True
    report["functional_verdict_passed"]=all(r["passed"] for r in report["rows"])
    report["status"]="functional_pass_requires_external_log_review" if report["functional_verdict_passed"] else "functional_checks_failed"
except Exception as error:
    report["errors"].append(repr(error));report["status"]="incomplete_run";traceback.print_exc()
finally:
    summary=dict(report)
    summary["rows"]=[{k:v for k,v in row.items() if k!="trajectory"} for row in report["rows"]]
    summary["full_report_path"]=str(args.output)
    print("REPORT_JSON "+json.dumps(summary,allow_nan=False),flush=True)
    if not args.stdout_only:
        try:
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(report,indent=2,allow_nan=False),encoding="utf-8")
        except OSError as error:print("REPORT_FILE_ERROR "+str(error),flush=True)
    if view is not None:
        try:view.invalidate()
        except Exception:pass
    try:iface.reset_simulation()
    except Exception:pass
    app.close(wait_for_replicator=False)
