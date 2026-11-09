"""Bounded CPU-only validation of the pinned original detent instruction bridge.

Does not change MJCF or USD. A successful result is diagnostic bridge evidence,
not native Linux plugin acceptance or an Isaac task validation.
"""
import argparse
import ctypes
import hashlib
import json
import math
from pathlib import Path
import random
import sys

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    home=Path.home()/"AppData/LocalLow/CellCultureLabReference"
    parser.add_argument("--dependency-dir",type=Path,default=home/"mujoco330")
    parser.add_argument("--bridge",type=Path,default=home/"detent_bridge/detent_binary_bridge.dll")
    parser.add_argument("--source-dir",type=Path,default=Path.home()/".codex/tmp-repos/AutoBio/autobio/model/instrument")
    parser.add_argument("--output-dir",type=Path,default=home/"detent_bridge_validation")
    args=parser.parse_args()
    sys.path.insert(0,str(args.dependency_dir))
    import mujoco as mj
    if mj.__version__!="3.3.0":raise RuntimeError("Requires isolated MuJoCo3.3.0")
    mj.mj_loadPluginLibrary(str(args.bridge.resolve()))
    library=ctypes.CDLL(str(args.bridge.resolve()))
    library.lab_detent_bridge_status.restype=ctypes.c_char_p
    status=library.lab_detent_bridge_status().decode()
    print("BRIDGE_STATUS "+status,flush=True)
    if status!="pinned_original_detent_code_ready":raise RuntimeError(status)
    report=dict(bridge=str(args.bridge.resolve()),bridge_sha256=hashlib.sha256(args.bridge.read_bytes()).hexdigest(),
        mujoco_version=mj.__version__,bridge_status=status,force_checks=[],invalid_models=[],models=[],
        linux_native_full_plugin_validated=False,isaac_task_validated=False,production_use_approved=False)
    def xml(steps,stiffness=2.3,limited="true",springref=0,two_joints=False):
        extra='<joint name="second" type="slide"/>' if two_joints else ''
        return f'''<mujoco><compiler angle="radian"/><extension><plugin plugin="mjlab.passive.detent"><instance name="detent"/></plugin></extension>
        <worldbody><body><joint name="j" type="hinge" range="-1 1" limited="{limited}" stiffness="{stiffness}" damping=".07" springref="{springref}" user="{steps}"/>{extra}<geom size=".01"/><plugin instance="detent"/></body></worldbody></mujoco>'''
    rng=random.Random(20261019)
    for steps in (2,3,11):
        model=mj.MjModel.from_xml_string(xml(steps));data=mj.MjData(model)
        low,high=map(float,model.jnt_range[0]);spacing=(high-low)/(steps-1)
        values=[low-.2,high+.2]+[low+i*spacing for i in range(steps)]
        values.extend(low+(i+.5)*spacing for i in range(steps-1))
        values.extend(rng.uniform(low-.3,high+.3) for _ in range(1000))
        maximum=0.
        for q in values:
            speed=rng.uniform(-2,2);data.qpos[0]=q;data.qvel[0]=speed
            mj.mj_forward(model,data)
            scaled=(q-low)/spacing
            index=math.floor(scaled+.5) if scaled>=0 else math.ceil(scaled-.5)
            index=min(steps-1,max(0,index))
            target=low+spacing*index
            expected=float(model.jnt_stiffness[0])*(target-q)-float(model.dof_damping[0])*speed
            error=abs(float(data.qfrc_passive[0])-expected);maximum=max(maximum,error)
            if not math.isfinite(error):raise RuntimeError("Nonfinite passive force")
        record=dict(steps=steps,samples=len(values),max_absolute_error=maximum,passed=maximum<1e-12)
        report["force_checks"].append(record)
        print("FORCE_CHECK "+json.dumps(record),flush=True)
    for label,parameters in [("one_step",dict(steps=1)),("zero_stiffness",dict(steps=3,stiffness=0)),
        ("unlimited",dict(steps=3,limited="false")),("nonzero_spring_reference",dict(steps=3,springref=.1)),
        ("multiple_dofs",dict(steps=3,two_joints=True))]:
        try:
            model=mj.MjModel.from_xml_string(xml(**parameters));data=mj.MjData(model)
            report["invalid_models"].append(dict(case=label,rejected=False))
        except Exception as error:
            report["invalid_models"].append(dict(case=label,rejected=True,error=repr(error)))
    from export_physics_reference import export_model
    args.output_dir.mkdir(parents=True,exist_ok=True)
    for name in ("thermal_cycler_biorad_c1000","vortex_mixer_genie_2"):
        source=args.source_dir/(name+".xml")
        model=mj.MjModel.from_xml_path(str(source));data=mj.MjData(model)
        # Exercise the original initialization and force calculation, then a
        # short zero-input trajectory without importing task Python state logic.
        for _ in range(100):mj.mj_step(model,data)
        finite=all(math.isfinite(float(x)) for x in data.qpos) and all(math.isfinite(float(x)) for x in data.qvel)
        reference=export_model(mj,__import__("numpy"),source,True)
        reference.update(original_version_matched=True,
            acceptance_scope="MuJoCo3.3.0 diagnostic reference using original detent machine-code ABI bridge; native Linux validation pending",
            plugin_execution_mode="original detent x86-64 functions with Windows/SysV ABI and CRT bridge",
            original_plugin_native_platform=False,bridge_sha256=report["bridge_sha256"],
            linux_native_full_plugin_validated=False)
        path=args.output_dir/(name+".json")
        path.write_text(json.dumps(reference,indent=2,allow_nan=False),encoding="utf-8")
        record=dict(name=name,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            joints=model.njnt,plugins=model.nplugin,short_simulation_steps=100,finite=finite,
            reference=str(path),trials=len(reference["trials"]),qpos=list(map(float,data.qpos)))
        report["models"].append(record);print("MODEL_CHECK "+json.dumps(record),flush=True)
    report["passed"]=all(r["passed"] for r in report["force_checks"]) and all(r["rejected"] for r in report["invalid_models"]) and all(r["finite"] for r in report["models"])
    (args.output_dir/"bridge_validation.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print("REPORT_JSON "+json.dumps(report),flush=True)
    return 0 if report["passed"] else 1


if __name__=="__main__":raise SystemExit(main())
