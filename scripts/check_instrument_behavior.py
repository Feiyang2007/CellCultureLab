"""Compare portable decisions against the actual upstream class methods.

Only selected source class AST nodes are executed to avoid importing unrelated
rendering/UI/task dependencies. Methods themselves are not edited or rewritten.
Also compares C1000 force-response traces in the original MuJoCo model using
the experimental detent bridge. No Isaac task acceptance is implied.
"""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace
from typing import TypeVar
from instrument_behavior import C1000LockController,detent_index,vortex_command


def load_source(source,mj,np):
    namespace=dict(mujoco=mj,np=np,T=TypeVar("T"))
    for filename,names in [("simulation.py",{"System"}),
        ("instrument.py",{"Detent","ThermalCyclerBioradC1000","VortexMixerGenie2"})]:
        path=source/filename
        parsed=ast.parse(path.read_text(encoding="utf-8"))
        selected=[node for node in parsed.body if isinstance(node,ast.ClassDef) and node.name in names]
        if {n.name for n in selected}!=names:raise RuntimeError("Upstream classes changed")
        exec(compile(ast.Module(body=selected,type_ignores=[]),str(path),"exec"),namespace)
    return namespace


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    home=Path.home()/"AppData/LocalLow/CellCultureLabReference"
    parser.add_argument("--dependency-dir",type=Path,default=home/"mujoco330")
    parser.add_argument("--bridge",type=Path,default=home/"detent_bridge/detent_binary_bridge.dll")
    parser.add_argument("--source-dir",type=Path,default=Path.home()/".codex/tmp-repos/AutoBio/autobio")
    parser.add_argument("--output",type=Path,default=home/"detent_bridge_validation/behavior_comparison.json")
    args=parser.parse_args()
    sys.path.insert(0,str(args.dependency_dir))
    import mujoco as mj
    import numpy as np
    if mj.__version__!="3.3.0":raise RuntimeError("Requires MuJoCo3.3.0")
    mj.mj_loadPluginLibrary(str(args.bridge.resolve()))
    source=load_source(args.source_dir,mj,np)
    cmodel=mj.MjModel.from_xml_path(str(args.source_dir/"model/instrument/thermal_cycler_biorad_c1000.xml"))
    vmodel=mj.MjModel.from_xml_path(str(args.source_dir/"model/instrument/vortex_mixer_genie_2.xml"))
    report=dict(source_hashes={n:hashlib.sha256((args.source_dir/n).read_bytes()).hexdigest() for n in ("instrument.py","simulation.py")},
        mujoco_version=mj.__version__,bridge_sha256=hashlib.sha256(args.bridge.read_bytes()).hexdigest(),
        source_classes_executed_without_method_changes=True,source_ui_imports_excluded=True,
        isaac_backend_connected=False,isaac_task_validated=False,linux_native_full_plugin_validated=False,
        detent_samples=0,c1000_steps=0,vortex_cases=0,mismatches=[],force_controls=[])
    rng=random.Random(20261019)
    # Compare actual source detent subsystem on the three real detent joints.
    for model,names in [(cmodel,["lid-lever"]),(vmodel,["switch","knob"])]:
        for name in names:
            original=source["Detent"](name);original.reload(model)
            d=mj.MjData(model)
            low,high=map(float,model.jnt_range[original.joint_id]);n=int(model.jnt_user[original.joint_id,0])
            spacing=(high-low)/(n-1)
            values=[low+i*spacing for i in range(n)]
            values += [low+(i+.5)*spacing for i in range(n-1)]
            values += [float(np.nextafter(q,-math.inf)) for q in values]+[float(np.nextafter(q,math.inf)) for q in values]
            values += [rng.uniform(low-.1,high+.1) for _ in range(1000)]
            for q in values:
                d.qpos[original.qposadr]=q;original.update(d)
                actual=detent_index(q,low,high,n)
                report["detent_samples"]+=1
                if actual!=original.value:report["mismatches"].append(dict(type="detent",joint=name,q=q,source=float(original.value),port=actual))
    original=source["ThermalCyclerBioradC1000"]();original.reload(cmodel)
    port=C1000LockController(original.lid_qpos_min);d=mj.MjData(cmodel)
    low=original.lid_qpos_min;threshold=low+.01
    cases=[(0.,0),(low,0),(low,1),(low,0),(low+.05,0),(low,0),(low,1),(low,0)]
    cases += [(float(np.nextafter(threshold,-math.inf)),0),(threshold,0),(float(np.nextafter(threshold,math.inf)),0)]
    cases += [(rng.uniform(low,0),rng.randint(0,1)) for _ in range(2000)]
    enabled=bool(d.eq_active[original.lid_lock]);trace=[]
    for i,(lid,state) in enumerate(cases):
        d.qpos[original.lid_qposadr]=lid
        d.qpos[original.lever_qposadr]=float(cmodel.jnt_range[original.lever_joint,state])
        if i==0 or i%97==0:
            original.reset(d);decision=port.reset(lid,state,enabled)
        else:
            original.update(d);decision=port.update(lid,state,enabled)
        enabled=decision.enabled
        expected=(bool(d.eq_active[original.lid_lock]),bool(original._bad_locking))
        if (decision.enabled,decision.bad_locking)!=expected:
            report["mismatches"].append(dict(type="C1000",step=i,source=expected,port=[decision.enabled,decision.bad_locking]))
        trace.append(dict(step=i,lid_rad=lid,lever_state=state,enabled=enabled,bad_locking=decision.bad_locking))
    report["c1000_steps"]=len(cases);report["c1000_logic_trace"]=trace
    # Compare source contact membership on both geom1 and geom2 (no force gate).
    original_v=source["VortexMixerGenie2"]();original_v.reload(vmodel)
    geom=original_v.platform_geom_id_range.start
    touches=[([],False),([SimpleNamespace(geom1=geom,geom2=-1)],True),
        ([SimpleNamespace(geom1=-1,geom2=geom)],True),([SimpleNamespace(geom1=-2,geom2=-1)],False)]
    for switch in range(3):
        for knob in range(11):
            for contacts,touch in touches:
                fake=SimpleNamespace(qpos=np.zeros(vmodel.nq),ctrl=np.zeros(vmodel.nu),contact=contacts)
                for system,state in [(original_v.switch,switch),(original_v.knob,knob)]:
                    lo,hi=vmodel.jnt_range[system.joint_id]
                    fake.qpos[system.qposadr]=lo+(hi-lo)*state/(int(system.joint_num_steps)-1)
                original_v.reset(fake)
                # Vortex has no custom _reset: Manager applies its command only
                # on the subsequent update, unlike C1000._reset's delegation.
                original_v.update(fake)
                actual=vortex_command(switch,knob,touch)
                expected=float(fake.ctrl[original_v.platform_actuator_id])
                report["vortex_cases"]+=1
                if abs(actual.target_velocity_rad_s-expected)>1e-12:
                    report["mismatches"].append(dict(type="vortex",switch=switch,knob=knob,touch=touch,source=expected,port=actual.target_velocity_rad_s))
    # Physical controls: original equality, same torque, locked vs unlocked.
    cmodel.opt.timestep=.001
    for locked in (False,True):
        responses={}
        for backend in ("upstream","portable"):
            system=source["ThermalCyclerBioradC1000"]();system.reload(cmodel)
            controller=C1000LockController(system.lid_qpos_min);data=mj.MjData(cmodel)
            data.qpos[system.lid_qposadr]=system.lid_qpos_min
            state=0 if locked else 1
            data.qpos[system.lever_qposadr]=cmodel.jnt_range[system.lever_joint,state]
            mj.mj_forward(cmodel,data)
            if backend=="upstream":system.reset(data)
            else:
                decision=controller.reset(float(data.qpos[system.lid_qposadr]),state,bool(data.eq_active[system.lid_lock]))
                data.eq_active[system.lid_lock]=decision.enabled
            lid_dof=int(cmodel.jnt_dofadr[system.lid_joint]);rows=[]
            for step in range(500):
                data.qfrc_applied[lid_dof]=.05
                mj.mj_step(cmodel,data)
                if backend=="upstream":system.update(data)
                else:
                    lever=system.lid_lever
                    lever_state=detent_index(float(data.qpos[lever.qposadr]),lever.joint_low,lever.joint_high,int(lever.joint_num_steps))
                    decision=controller.update(float(data.qpos[system.lid_qposadr]),lever_state,bool(data.eq_active[system.lid_lock]))
                    data.eq_active[system.lid_lock]=decision.enabled
                rows.append(dict(step=step+1,q=list(map(float,data.qpos)),v=list(map(float,data.qvel)),lock=bool(data.eq_active[system.lid_lock])))
            responses[backend]=rows
        maximum=max(abs(a-b) for x,y in zip(responses["upstream"],responses["portable"]) for key in ("q","v") for a,b in zip(x[key],y[key]))
        record=dict(locked=locked,applied_lid_torque_nm=.05,steps=500,dt=.001,
            max_upstream_port_trajectory_difference=maximum,
            max_lid_displacement_rad=max(abs(x["q"][original.lid_qposadr]-low) for x in responses["portable"]),
            source_lock_trace_identical=all(x["lock"]==y["lock"] for x,y in zip(responses["upstream"],responses["portable"])),
            trajectories=responses)
        report["force_controls"].append(record)
    report["logic_equivalence_passed"]=not report["mismatches"]
    report["mujoco_force_trace_equivalence_passed"]=all(x["max_upstream_port_trajectory_difference"]<1e-12 and x["source_lock_trace_identical"] for x in report["force_controls"])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False),encoding="utf-8")
    print(json.dumps({k:v for k,v in report.items() if k not in ("c1000_logic_trace","force_controls")}))
    print(json.dumps([{k:v for k,v in r.items() if k!="trajectories"} for r in report["force_controls"]]))
    return 0 if report["logic_equivalence_passed"] and report["mujoco_force_trace_equivalence_passed"] else 1


if __name__=="__main__":raise SystemExit(main())
