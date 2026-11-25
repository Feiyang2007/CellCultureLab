"""Read-only compiled original task options and position-actuator audit.

Apply only the exact source prepare helper (recursive robot gravcomp=1).
No expert trajectory or task success is claimed.
"""
import argparse,ast,hashlib,json,sys
from pathlib import Path
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
home=Path.home()/'AppData/LocalLow/CellCultureLabReference';sys.path.insert(0,str(home/'mujoco330'))
import mujoco
import numpy as np
source=Path.home()/'.codex/tmp-repos/AutoBio/autobio';task=source/'load_centrifuge_5430.py';scene=source/'model/scene/insert_centrifuge_5430.xml'
parsed=ast.parse(task.read_text(encoding='utf-8'))
helper=next(n for n in parsed.body if isinstance(n,ast.FunctionDef) and n.name=='set_gravcomp')
namespace={'mujoco':mujoco};exec(compile(ast.Module(body=[helper],type_ignores=[]),str(task),'exec'),namespace)
spec=mujoco.MjSpec.from_file(str(scene));namespace['set_gravcomp'](spec.body('/ur:world'));m=spec.compile();d=mujoco.MjData(m)
mujoco.mj_forward(m,d)
acts=[]
for i in range(m.nu):
    name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_ACTUATOR,i)
    acts.append(dict(name=name,gainprm=m.actuator_gainprm[i].tolist(),biasprm=m.actuator_biasprm[i].tolist(),
        trnid=m.actuator_trnid[i].tolist(),forcerange=m.actuator_forcerange[i].tolist(),forcelimited=bool(m.actuator_forcelimited[i]),ctrlrange=m.actuator_ctrlrange[i].tolist()))
contacts=[dict(geom1=int(c.geom1),geom2=int(c.geom2),distance=float(c.dist)) for c in d.contact]
report=dict(scope='compiled original full scene and exact robot gravity preparation; expert not executed',
    mujoco_version=mujoco.__version__,scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
    task_sha256=hashlib.sha256(task.read_bytes()).hexdigest(),ngeom=m.ngeom,nbody=m.nbody,nq=m.nq,nv=m.nv,nkey=m.nkey,
    options=dict(timestep=float(m.opt.timestep),gravity=m.opt.gravity.tolist(),integrator=int(m.opt.integrator),impratio=float(m.opt.impratio),cone=int(m.opt.cone),noslip_iterations=int(m.opt.noslip_iterations),enableflags=int(m.opt.enableflags)),
    robot_gravity_compensated_body_count=int(np.count_nonzero(m.body_gravcomp)),actuators=acts,
    robot_controller='joint position targets, updated by source TOPPRA path follower at20Hz; not constant Cartesian force',
    expert_planner=dict(joint_velocity_limit=.8,joint_acceleration_limit=.6,frequency_hz=20),
    insertion_path='source distal localZ +.005 to proximal -.03 with fixed grasp-relative transform, then loosen gripper240->190 and wait200steps',
    initial_contact_count=int(d.ncon),initial_contacts=contacts,
    full_task_validated=False,expert_executed=False)
args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('actuators','initial_contacts')}),flush=True)
