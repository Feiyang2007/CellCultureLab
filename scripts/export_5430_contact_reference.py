"""Compile untouched 3.3.0 source models for an isolated single-tube contact fixture.

No robot, table or second tube: this is not the upstream insertion task. Export
all instrument collision shapes, slot frames and the original simple tube.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.sax.saxutils as xml

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
home=Path.home()/'AppData/LocalLow/CellCultureLabReference'
sys.path.insert(0,str(home/'mujoco330'))
import mujoco as mj
import numpy as np
if mj.__version__!='3.3.0':raise RuntimeError('Requires original MuJoCo 3.3.0')
source=Path.home()/'.codex/tmp-repos/AutoBio/autobio'
instrument=source/'model/instrument/centrifuge_eppendorf_5430.xml'
tube=source/'model/object/centrifuge_1-5ml_screw-simple.xml'
document=f'''<mujoco model="5430 contact fixture">
<option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"/>
<asset><model name="instrument" file={xml.quoteattr(str(instrument))}/>
<model name="tube" file={xml.quoteattr(str(tube))}/></asset>
<worldbody><attach model="instrument" body="world" prefix="inst/"/>
<body name="tube_root" pos="0 0 1"><freejoint name="tube_free"/>
<attach model="tube" body="world" prefix="tube/"/></body></worldbody></mujoco>'''
m=mj.MjModel.from_xml_string(document);d=mj.MjData(m);mj.mj_forward(m,d)
name=lambda kind,i:mj.mj_id2name(m,kind,i) or f'unnamed_{i}'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
shapes=[];meshes={}
for i in range(m.ngeom):
    if not(m.geom_contype[i] or m.geom_conaffinity[i]):continue
    mid=int(m.geom_dataid[i]);kind=int(m.geom_type[i])
    if kind==int(mj.mjtGeom.mjGEOM_MESH) and mid not in meshes:
        va=int(m.mesh_vertadr[mid]);vn=int(m.mesh_vertnum[mid]);fa=int(m.mesh_faceadr[mid]);fn=int(m.mesh_facenum[mid])
        meshes[mid]=dict(points=m.mesh_vert[va:va+vn].tolist(),faces=m.mesh_face[fa:fa+fn].tolist())
    shapes.append(dict(id=i,name=name(mj.mjtObj.mjOBJ_GEOM,i),body_id=int(m.geom_bodyid[i]),
        body_name=name(mj.mjtObj.mjOBJ_BODY,int(m.geom_bodyid[i])),type=kind,size=m.geom_size[i].tolist(),
        position=d.geom_xpos[i].tolist(),rotation=d.geom_xmat[i].reshape(3,3).tolist(),
        mesh_id=mid if kind==7 else None,contype=int(m.geom_contype[i]),conaffinity=int(m.geom_conaffinity[i]),
        friction=m.geom_friction[i].tolist(),margin=float(m.geom_margin[i]),gap=float(m.geom_gap[i])))
slots=[]
for i in range(m.nsite):
    n=name(mj.mjtObj.mjOBJ_SITE,i)
    if n.startswith('inst/slot'):
        slots.append(dict(name=n,position=d.site_xpos[i].tolist(),rotation=d.site_xmat[i].reshape(3,3).tolist()))
if len(slots)!=30:raise RuntimeError(f'Expected 30 slots, got {len(slots)}')
root=m.body('tube_root').id
ids=[i for i in range(m.nbody) if m.body_weldid[i]==root]
mass=float(sum(m.body_mass[i] for i in ids))
com=sum(m.body_mass[i]*d.xipos[i] for i in ids)/mass
inertia=np.zeros((3,3))
for i in ids:
    r=d.ximat[i].reshape(3,3)
    delta=d.xipos[i]-com
    inertia+=r@np.diag(m.body_inertia[i])@r.T+m.body_mass[i]*(np.dot(delta,delta)*np.eye(3)-np.outer(delta,delta))
com_local=com-d.xpos[root]
vals,axes=np.linalg.eigh(inertia)
if np.linalg.det(axes)<0:axes[:,0]*=-1
quat=np.zeros(4);mj.mju_mat2Quat(quat,axes.reshape(-1))
instrument_ids={g['id'] for g in shapes if not g['body_name'].startswith('tube/')}
tube_ids={g['id'] for g in shapes if g['body_name'].startswith('tube/')}
if len(tube_ids)!=4:raise RuntimeError('Expected four simple tube colliders')
slot=slots[0];R=np.array(slot['rotation']);p=np.array(slot['position'])
sq=np.zeros(4);mj.mju_mat2Quat(sq,R.reshape(-1));relq=np.array([1.,0,0,-1])/np.sqrt(2)
tq=np.zeros(4);mj.mju_mulQuat(tq,sq,relq)
adr=int(m.jnt_qposadr[m.joint('tube_free').id]);scans=[]
for offset in (0.,.003,.006):
    rows=[]
    for z in np.linspace(.020,-.040,121):
        pos=p+R@np.array([offset,0,z]);d.qpos[adr:adr+3]=pos;d.qpos[adr+3:adr+7]=tq
        mj.mj_forward(m,d)
        contacts=[c for c in d.contact if (int(c.geom1) in tube_ids and int(c.geom2) in instrument_ids) or
                  (int(c.geom2) in tube_ids and int(c.geom1) in instrument_ids)]
        rows.append(dict(slot_axis_position_m=float(z),contacts=len(contacts),
            min_distance_m=min((float(c.dist) for c in contacts),default=None)))
    scans.append(dict(lateral_offset_m=offset,rows=rows))
report=dict(mujoco_version=mj.__version__,scope='isolated static geometry/contact fixture; not robot or full task',
    instrument_xml=str(instrument),instrument_sha256=sha(instrument),tube_xml=str(tube),tube_sha256=sha(tube),
    source_task=str(source/'load_centrifuge_5430.py'),source_task_sha256=sha(source/'load_centrifuge_5430.py'),
    instrument_qpos=m.qpos0[:2].tolist(),instrument_pose='source open lid and zero rotor; rigid fixture in USD pilot',
    shapes=shapes,meshes=meshes,slots=slots,selected_slot=slot['name'],tube_rotation_wxyz=tq.tolist(),
    tube_initial_root_position=[0,0,1],tube_mass_kg=mass,tube_com_local=com_local.tolist(),
    tube_principal_inertia=vals.tolist(),tube_principal_axes_wxyz=quat.tolist(),
    task_target_local=dict(distal=[0,0,.005],proximal=[0,0,-.03],relative_quaternion_wxyz=relq.tolist()),
    scans=scans,production_ready=False,isaac_contact_validated=False)
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(report,allow_nan=False),encoding='utf-8')
print(json.dumps(dict(output=str(args.output),slots=len(slots),instrument_colliders=len(instrument_ids),tube_colliders=len(tube_ids),
    mass=mass,scan_summary=[dict(offset=s['lateral_offset_m'],first_contact=next((r for r in s['rows'] if r['contacts']),None),
        at_target=min(s['rows'],key=lambda r:abs(r['slot_axis_position_m']+.03))) for s in scans])),flush=True)
