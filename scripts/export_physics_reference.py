"""Compile source MJCF without editing it; export compiled physics and torque trials.

Use --dependency-dir for an isolated MuJoCo installation. Plugin models require
their original compatible plugin; failures are recorded, never silently stripped.
This supplements (does not overwrite) upstream/truth geometric snapshots.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def export_model(mj, np, source, trial_enabled):
    m = mj.MjModel.from_xml_path(str(source))
    d = mj.MjData(m)
    mj.mj_forward(m, d)
    name = lambda kind, i: mj.mj_id2name(m, kind, i) or f"unnamed_{i}"
    obj = mj.mjtObj
    arr = lambda value: np.asarray(value).tolist()
    fields = lambda prefix, names, i: {key: arr(getattr(m, prefix + key)[i]) for key in names}
    bodies = [dict(id=i, name=name(obj.mjOBJ_BODY, i),
                   **fields("body_", ["parentid", "weldid", "mass", "inertia", "ipos", "iquat", "plugin"], i),
                   world_pos=arr(d.xpos[i]), world_quat_wxyz=arr(d.xquat[i])) for i in range(m.nbody)]
    joints = []
    for i in range(m.njnt):
        dof = int(m.jnt_dofadr[i])
        dof_end = int(m.jnt_dofadr[i+1]) if i+1<m.njnt else m.nv
        joints.append(dict(id=i, name=name(obj.mjOBJ_JOINT, i),
            **fields("jnt_", ["type", "bodyid", "qposadr", "dofadr", "limited", "range", "stiffness", "solref", "solimp", "pos", "axis", "user"], i),
            armature=float(m.dof_armature[dof]), damping=float(m.dof_damping[dof]),
            frictionloss=float(m.dof_frictionloss[dof]),
            dofs=[dict(id=k,armature=float(m.dof_armature[k]),damping=float(m.dof_damping[k]),
                       frictionloss=float(m.dof_frictionloss[k])) for k in range(dof,dof_end)],
            world_anchor=arr(d.xanchor[i]), world_axis=arr(d.xaxis[i])))
    geoms = [dict(id=i, name=name(obj.mjOBJ_GEOM, i),
        **fields("geom_", ["bodyid", "type", "contype", "conaffinity", "condim", "friction", "margin", "gap", "solref", "solimp"], i))
        for i in range(m.ngeom)]
    pairs = [dict(id=i, **fields("pair_", ["geom1", "geom2", "dim", "friction", "margin", "gap", "solref", "solimp"], i)) for i in range(m.npair)]
    excludes = [dict(body1=int(x) >> 16, body2=int(x) & 65535) for x in m.exclude_signature]
    equality = [dict(id=i, name=name(obj.mjOBJ_EQUALITY, i),
        **fields("eq_", ["type", "obj1id", "obj2id", "active0", "data", "solref", "solimp"], i)) for i in range(m.neq)]
    actuators = [dict(id=i, name=name(obj.mjOBJ_ACTUATOR, i),
        **fields("actuator_", ["trntype", "trnid", "dyntype", "gaintype", "biastype", "dynprm", "gainprm", "biasprm", "gear", "ctrllimited", "ctrlrange", "forcelimited", "forcerange"], i))
        for i in range(m.nu)]
    xml = ET.parse(source).getroot()
    out = dict(schema_version=1, source_xml=str(source), source_xml_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        mujoco_version=mj.__version__, scope="standalone instrument; task assembly rules must be exported separately",
        bodies=bodies, joints=joints, geoms=geoms, contact_pairs=pairs, contact_excludes=excludes,
        equality=equality, actuators=actuators,
        plugins=[dict(tag=x.tag, attributes=x.attrib) for x in xml.iter("plugin")],
        qpos0=arr(m.qpos0), qpos_spring=arr(m.qpos_spring),
        options=dict(gravity=arr(m.opt.gravity), timestep=float(m.opt.timestep), integrator=int(m.opt.integrator),
                     disableflags=int(m.opt.disableflags), enableflags=int(m.opt.enableflags)),
        task_script_migrated=False, trials=[])
    out["trial_protocol_status"] = "not_requested"
    if trial_enabled and not joints:
        out["trial_protocol_status"] = "not_applicable_no_joints"
        return out
    if trial_enabled:
        out["trial_protocol_status"] = "scalar_joint_protocol"
        if any(j["type"] not in (2, 3) for j in joints) or m.nq != m.nv:
            raise ValueError("Torque protocol supports scalar hinge/slide joints only")
        dt, steps = 1/240, 60
        # Stable interior state avoids conflating soft limit differences with inertia.
        initial = m.qpos0.copy()
        for j in joints:
            if j["limited"] and j["range"][0] >= 0:
                initial[j["qposadr"]] = 0.7
        protocols = [(j["name"], force, [0, 0, 0], None) for j in joints for force in (-0.001, 0, 0.001)]
        protocols += [(joints[0]["name"], 0, [0, 0, -9.81], None)]
        protocols += [(joints[-1]["name"], 0.001, [0, 0, 0], 0.1)]
        authored_armature = m.dof_armature.copy()
        for target, force, gravity, test_armature in protocols:
            steps = 12 if any(gravity) else 60
            m.opt.timestep = dt
            m.opt.integrator = mj.mjtIntegrator.mjINT_EULER
            m.opt.gravity[:] = gravity
            mj.mj_resetData(m, d)
            d.qpos[:] = initial
            index = next(j["dofadr"] for j in joints if j["name"] == target)
            m.dof_armature[:] = authored_armature
            if test_armature is not None:
                m.dof_armature[index] = test_armature
            d.qfrc_applied[index] = force
            mj.mj_forward(m, d)
            initial_acc = arr(d.qacc)
            trajectory = [dict(step=0, q=arr(d.qpos), v=arr(d.qvel))]
            for step in range(1, steps+1):
                mj.mj_step(m, d)
                trajectory.append(dict(step=step, q=arr(d.qpos), v=arr(d.qvel)))
            out["trials"].append(dict(joint=target, force=force, gravity=gravity, dt=dt, steps=steps,
                test_armature_override=test_armature,
                initial_q=arr(initial), initial_v=[0.0]*m.nv, initial_acceleration=initial_acc,
                equality_active=arr(d.eq_active), trajectory=trajectory))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source-dir", type=Path, required=True)
    p.add_argument("--dependency-dir", type=Path)
    p.add_argument("--plugin", type=Path)
    p.add_argument("--names", nargs="+", default=["centrifuge_tiangen_tgear_mini"])
    p.add_argument("--trials", action="store_true")
    p.add_argument("--allow-version-drift", action="store_true", help="Candidate only; never original-version acceptance")
    p.add_argument("--output-dir", type=Path, default=ROOT/"upstream/physics_reference")
    a = p.parse_args()
    if a.dependency_dir:
        sys.path.insert(0, str(a.dependency_dir.resolve()))
    import mujoco as mj
    import numpy as np
    if mj.__version__ != "3.3.0" and not a.allow_version_drift:
        raise RuntimeError("Reference requires original MuJoCo 3.3.0, got " + mj.__version__)
    if a.plugin:
        mj.mj_loadPluginLibrary(str(a.plugin.resolve()))
    a.output_dir.mkdir(parents=True, exist_ok=True)
    summary = dict(mujoco_version=mj.__version__, results=[])
    for name in a.names:
        try:
            result = export_model(mj, np, a.source_dir/(name+".xml"), a.trials)
            result["original_version_matched"] = mj.__version__ == "3.3.0"
            result["acceptance_scope"] = "original-version reference" if result["original_version_matched"] else "candidate reference; requires 3.3.0 repeat"
            dest = a.output_dir/(name+".json")
            dest.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
            summary["results"].append(dict(name=name, status="EXPORTED", path=str(dest),
                bodies=len(result["bodies"]), equality=len(result["equality"]), trials=len(result["trials"])))
        except Exception as error:
            summary["results"].append(dict(name=name, status="FAILED", error=repr(error)))
    (a.output_dir/"export_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=True), flush=True)
    return 0 if all(r["status"] == "EXPORTED" for r in summary["results"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
