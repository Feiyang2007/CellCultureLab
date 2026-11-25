"""Continuous C1000 latch cycle, original MuJoCo equality and detent bridge.

Only lever positions are imposed by the input fixture. Lid q/v are never set
after initialization. This validates behavior/control continuity in MuJoCo,
not robotic manipulation, an Isaac backend, or a native Linux plugin build.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from check_instrument_behavior import load_source
from instrument_behavior import C1000LockController,detent_index


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    home=Path.home()/"AppData/LocalLow/CellCultureLabReference"
    parser.add_argument("--dependency-dir",type=Path,default=home/"mujoco330")
    parser.add_argument("--bridge",type=Path,default=home/"detent_bridge/detent_binary_bridge.dll")
    parser.add_argument("--source-dir",type=Path,default=Path.home()/".codex/tmp-repos/AutoBio/autobio")
    parser.add_argument("--output",type=Path,default=home/"detent_bridge_validation/c1000_latch_cycle.json")
    args=parser.parse_args()
    sys.path.insert(0,str(args.dependency_dir))
    import mujoco as mj
    import numpy as np
    if mj.__version__!="3.3.0":raise RuntimeError("Requires MuJoCo3.3.0")
    mj.mj_loadPluginLibrary(str(args.bridge.resolve()))
    classes=load_source(args.source_dir,mj,np)
    xml=args.source_dir/"model/instrument/thermal_cycler_biorad_c1000.xml"
    thresholds=dict(locked_max_abs_lid_displacement_rad=.005,unlocked_min_abs_lid_displacement_rad=.01,
        closed_band_above_lower_limit_rad=.005,torque_nm=.05)
    report=dict(mujoco_version=mj.__version__,model_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(),
        bridge_sha256=hashlib.sha256(args.bridge.read_bytes()).hexdigest(),thresholds=thresholds,
        input_fixture="imposed lever detent positions; lid q/v never overwritten after initialization",
        lock_backend="original MuJoCo joint equality",isaac_backend_connected=False,
        robot_task_validated=False,linux_native_full_plugin_validated=False,runs={})
    for backend in ("upstream","portable"):
        model=mj.MjModel.from_xml_path(str(xml));model.opt.timestep=.001
        data=mj.MjData(model);system=classes["ThermalCyclerBioradC1000"]();system.reload(model)
        low=system.lid_qpos_min;li=system.lid_qposadr;le=system.lid_lever
        controller=C1000LockController(low)
        data.qpos[li]=low;data.qpos[system.lever_qposadr]=model.jnt_range[system.lever_joint,0]
        mj.mj_forward(model,data)
        dof=int(model.jnt_dofadr[system.lid_joint]);events=[];trajectory=[];phases=[]
        def read():
            return dict(time=float(data.time),q=list(map(float,data.qpos)),v=list(map(float,data.qvel)),
                lock=bool(data.eq_active[system.lid_lock]),bad_locking=bool(system._bad_locking if backend=="upstream" else controller.bad_locking))
        def update(reset=False):
            if backend=="upstream":
                (system.reset if reset else system.update)(data)
            else:
                state=detent_index(float(data.qpos[le.qposadr]),le.joint_low,le.joint_high,int(le.joint_num_steps))
                decision=(controller.reset if reset else controller.update)(float(data.qpos[li]),state,bool(data.eq_active[system.lid_lock]))
                data.eq_active[system.lid_lock]=decision.enabled
        update(True)
        def phase(name,torque,steps,lever_state=None,until=None):
            before=read()
            if lever_state is not None:
                data.qpos[system.lever_qposadr]=model.jnt_range[system.lever_joint,lever_state]
                data.qvel[int(model.jnt_dofadr[system.lever_joint])]=0
                mj.mj_forward(model,data);update()
            after_input=read()
            events.append(dict(phase=name,lever_state=lever_state,
                lid_qv_preserved=before["q"][li]==after_input["q"][li] and before["v"][li]==after_input["v"][li]))
            start=float(data.qpos[li]);worst=0.;reached=until is None
            for step in range(steps):
                data.qfrc_applied[dof]=torque;mj.mj_step(model,data);update()
                row=read();row.update(phase=name,torque_nm=torque);trajectory.append(row)
                if not all(math.isfinite(x) for x in row["q"]+row["v"]):raise RuntimeError("Nonfinite state")
                worst=max(worst,abs(float(data.qpos[li])-start))
                if until is not None and until(row):reached=True;break
            record=dict(name=name,torque_nm=torque,steps=step+1,condition_reached=reached,
                before=before,after_input=after_input,end=read(),max_lid_motion_from_start_rad=worst)
            phases.append(record);return record
        phase("initial_locked_force",.05,500)
        phase("unlock_and_open",.05,600,lever_state=1,until=lambda r:r["q"][li]>low+.08)
        phase("try_lock_while_open",0.,20,lever_state=0)
        phase("close_after_bad_lock",-.05,1800,until=lambda r:r["q"][li]<=low+.005 and r["v"][dof]<0)
        phase("settle_with_bad_lock",-.05,500)
        phase("probe_open_despite_returning_closed",.05,120)
        phase("unlock_to_clear_and_close",-.05,1800,lever_state=1,until=lambda r:r["q"][li]<=low+.005 and r["v"][dof]<0)
        phase("settle_before_relock",-.05,500)
        phase("relock_and_force",.05,500,lever_state=0)
        by_name={p["name"]:p for p in phases}
        bad=by_name["try_lock_while_open"]["end"]
        closed=by_name["settle_with_bad_lock"]["end"]
        cleared=by_name["unlock_to_clear_and_close"]["after_input"]
        relock=by_name["relock_and_force"]["after_input"]
        verdict=dict(no_lid_state_overwrite_at_inputs=all(e["lid_qv_preserved"] for e in events),
            phase_conditions_reached=all(p["condition_reached"] for p in phases),
            initial_locked_response_bounded=by_name["initial_locked_force"]["max_lid_motion_from_start_rad"]<thresholds["locked_max_abs_lid_displacement_rad"],
            unlock_allows_opening=by_name["unlock_and_open"]["max_lid_motion_from_start_rad"]>thresholds["unlocked_min_abs_lid_displacement_rad"],
            bad_lock_attempt_latches_without_enabling_lock=bad["bad_locking"] and not bad["lock"],
            returning_closed_does_not_clear_bad_lock=closed["bad_locking"] and not closed["lock"] and closed["q"][li]<=low+.005,
            bad_latch_allows_reopening=by_name["probe_open_despite_returning_closed"]["max_lid_motion_from_start_rad"]>thresholds["unlocked_min_abs_lid_displacement_rad"],
            unlock_clears_latch=not cleared["bad_locking"] and not cleared["lock"],
            relock_activates_only_after_clear=relock["lock"] and not relock["bad_locking"],
            final_locked_response_bounded=by_name["relock_and_force"]["max_lid_motion_from_start_rad"]<thresholds["locked_max_abs_lid_displacement_rad"])
        report["runs"][backend]=dict(phases=phases,events=events,trajectory=trajectory,verdict=verdict,
            gravity=list(map(float,model.opt.gravity)),dt=float(model.opt.timestep),midrun_resets=0)
    original=report["runs"]["upstream"]["trajectory"];ported=report["runs"]["portable"]["trajectory"]
    same_length=len(original)==len(ported)
    maximum=max(abs(a-b) for x,y in zip(original,ported) for key in ("q","v") for a,b in zip(x[key],y[key]))
    flags=same_length and all(x["phase"]==y["phase"] and x["lock"]==y["lock"] and x["bad_locking"]==y["bad_locking"] for x,y in zip(original,ported))
    report.update(source_port_trace_equal=same_length and maximum<1e-12 and flags,
        max_source_port_qv_difference=maximum,steps_per_backend=len(original),
        passed=same_length and maximum<1e-12 and flags and all(all(r["verdict"].values()) for r in report["runs"].values()))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False),encoding="utf-8")
    print(json.dumps({k:v for k,v in report.items() if k!="runs"}))
    print(json.dumps(report["runs"]["portable"]["verdict"]))
    print(json.dumps([{k:v for k,v in p.items() if k not in ("before","after_input","end")} for p in report["runs"]["portable"]["phases"]]))
    return 0 if report["passed"] else 1


if __name__=="__main__":raise SystemExit(main())
