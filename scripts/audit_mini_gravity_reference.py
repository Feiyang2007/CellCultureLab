"""Read-only MuJoCo3.3.0 gravity oracle, including constrained endpoints.

Compares inverse-dynamics gravity with the independent sum of COM gravity
moments. No constraint impulse or measured acceleration is used as gravity.
Does not alter Isaac assets, Qwen runner, or any previous report.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    home=Path.home()/"AppData/LocalLow/CellCultureLabReference"
    parser.add_argument("--dependency-dir",type=Path,default=home/"mujoco330")
    parser.add_argument("--source",type=Path,default=Path.home()/".codex/tmp-repos/AutoBio/autobio/model/instrument/centrifuge_tiangen_tgear_mini.xml")
    parser.add_argument("--output",type=Path,default=home/"mini_lid_gravity_reference_audit.json")
    args=parser.parse_args()
    sys.path.insert(0,str(args.dependency_dir))
    import mujoco as mj
    import numpy as np
    if mj.__version__!="3.3.0":raise RuntimeError("Requires MuJoCo3.3.0")
    reference=json.loads((ROOT/"upstream/physics_reference/centrifuge_tiangen_tgear_mini.json").read_text(encoding="utf-8"))
    source_hash=hashlib.sha256(args.source.read_bytes()).hexdigest()
    if reference["source_xml_sha256"]!=source_hash:raise RuntimeError("Source differs from canonical reference")
    model=mj.MjModel.from_xml_path(str(args.source));data=mj.MjData(model)
    model.opt.gravity[:]=[0,0,-9.81]
    jid=mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,"lid")
    qadr=int(model.jnt_qposadr[jid]);dof=int(model.jnt_dofadr[jid]);body=int(model.jnt_bodyid[jid])
    descendants=[]
    for bid in range(1,model.nbody):
        ancestor=bid
        while ancestor and ancestor!=body:ancestor=int(model.body_parentid[ancestor])
        if ancestor==body:descendants.append(bid)
    def evaluate(q):
        mj.mj_resetData(model,data);data.qpos[qadr]=q;data.qvel[:]=0
        mj.mj_forward(model,data)
        gravity_torque=-float(data.qfrc_bias[dof])
        moment=sum(float(np.dot(data.xaxis[jid],np.cross(data.xipos[bid]-data.xanchor[jid],model.body_mass[bid]*model.opt.gravity))) for bid in descendants)
        return dict(q_rad=float(q),gravity_torque_nm=gravity_torque,com_moment_torque_nm=moment,
            difference=abs(gravity_torque-moment),acceleration_rad_s2=float(data.qacc[dof]),
            constraint_force_nm=float(data.qfrc_constraint[dof]))
    low,high=map(float,model.jnt_range[jid]);rows=[evaluate(q) for q in np.linspace(low,high,1001)]
    left,right=low,high
    if evaluate(left)["gravity_torque_nm"]*evaluate(right)["gravity_torque_nm"]>=0:
        raise RuntimeError("Expected interior gravity sign change")
    for _ in range(60):
        middle=(left+right)/2
        if evaluate(middle)["gravity_torque_nm"]>0:right=middle
        else:left=middle
    balance=(left+right)/2
    curve=json.loads((ROOT/"outputs/runtime_validation/mini_lid_gravity_torque_curve.json").read_text(encoding="utf-8"))
    compare=[]
    for old in curve["rows"]:
        if old["gravity"]!="real" or old["torque_Nm"]!=0:continue
        expected=evaluate(float(old["q_in_rad"]))
        compare.append(dict(q_rad=old["q_in_rad"],isaac_acceleration_inferred_net_torque_nm=old["implied_net_torque_Nm"],
            reference_gravity_torque_nm=expected["gravity_torque_nm"],
            delta=old["implied_net_torque_Nm"]-expected["gravity_torque_nm"],
            at_limit=abs(old["q_in_rad"]-low)<1e-8 or abs(old["q_in_rad"]-high)<1e-8))
    eqid=mj.mj_name2id(model,mj.mjtObj.mjOBJ_EQUALITY,"lid-lock")
    report=dict(mujoco_version=mj.__version__,source_xml=str(args.source),source_xml_sha256=source_hash,
        source_reference_sha256=hashlib.sha256((ROOT/"upstream/physics_reference/centrifuge_tiangen_tgear_mini.json").read_bytes()).hexdigest(),
        gravity=[0,0,-9.81],qvel_zero=True,methods=["negative qfrc_bias at zero velocity","sum of COM gravity moments about hinge axis"],
        max_method_difference=max(r["difference"] for r in rows),low_endpoint=evaluate(low),high_endpoint=evaluate(high),
        unstable_interior_equilibrium_rad=balance,unstable_interior_equilibrium_deg=math.degrees(balance),
        sampled_max_opposing_open_torque_nm=max(-r["gravity_torque_nm"] for r in rows),
        sampled_max_opposing_close_torque_nm=max(r["gravity_torque_nm"] for r in rows),
        threshold_scope="ideal source model, at-rest quasi-static breakaway against gravity only; not a universal trajectory torque lower bound",
        lid_lock=dict(type=int(model.eq_type[eqid]),type_name="joint polynomial equality" if model.eq_type[eqid]==mj.mjtEq.mjEQ_JOINT else "other",
            active_at_reset=bool(model.eq_active0[eqid]),target_constant=float(model.eq_data[eqid,0])),
        comparison_to_isaac_samples=compare,rows=rows,
        isaac_full_trajectory_validated=False,real_device_calibrated=False)
    if report["max_method_difference"]>1e-12:raise RuntimeError("Gravity oracles disagree")
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False),encoding="utf-8")
    print(json.dumps({k:v for k,v in report.items() if k not in ("rows","comparison_to_isaac_samples")}))
    print(json.dumps(compare[:2]))


if __name__=="__main__":main()
