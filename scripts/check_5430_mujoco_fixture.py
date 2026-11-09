"""Same static geometry, axial guide and declared force protocol in MuJoCo3.3.

This reconstruction is an explicit fixture, not the original robot task. It
keeps original per-geom contact parameters and compiled collider geometry.
"""
import argparse,json,sys,hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--offset',type=float,default=0)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--protocol',choices=('force','servo'),default='force')
parser.add_argument('--friction',type=float,default=1.)
parser.add_argument('--evaluate-isaac',type=Path)
parser.add_argument('--solver-profile',choices=('fixture_default','source_task'),default='fixture_default')
args=parser.parse_args();home=Path.home()/'AppData/LocalLow/CellCultureLabReference'
sys.path.insert(0,str(home/'mujoco330'))
import mujoco as mj
import numpy as np
from contact_fixture_protocol import command,schedule,design
if mj.__version__!='3.3.0':raise RuntimeError('Wrong engine version')
refpath=ROOT/'outputs/runtime_validation/5430_single_slot/reference.json'
ref=json.loads(refpath.read_text(encoding='utf-8'))
source=mj.MjModel.from_xml_path(ref['instrument_xml']);tube_source=mj.MjModel.from_xml_path(ref['tube_xml'])
fmt=lambda a:' '.join(format(float(v),'.17g') for v in a)
qfrom=lambda r:np.array(r).reshape(-1)
def quat(r):
    q=np.zeros(4);mj.mju_mat2Quat(q,np.array(r).reshape(-1));return q
tree=ET.Element('mujoco',model='fixed source geometry axial fixture')
ET.SubElement(tree,'compiler',angle='radian')
ET.SubElement(tree,'option',timestep='.001',gravity='0 0 -9.81',integrator='implicitfast')
asset=ET.SubElement(tree,'asset');world=ET.SubElement(tree,'worldbody')
for mid,mesh in ref['meshes'].items():
    ET.SubElement(asset,'mesh',name='mesh'+mid,vertex=fmt([v for p in mesh['points'] for v in p]),face=' '.join(str(v) for f in mesh['faces'] for v in f))
R=np.array(ref['slots'][0]['rotation']);slot=np.array(ref['slots'][0]['position']);start=slot+.02*R[:,2]+args.offset*R[:,0]
tube=ET.SubElement(world,'body',name='tube',pos=fmt(start),quat=fmt(ref['tube_rotation_wxyz']))
ET.SubElement(tube,'joint',name='guide',type='slide',axis='0 0 1',range='-.10 .03',limited='true')
ET.SubElement(tube,'inertial',mass=str(ref['tube_mass_kg']),pos=fmt(ref['tube_com_local']),diaginertia=fmt(ref['tube_principal_inertia']),quat=fmt(ref['tube_principal_axes_wxyz']))
tube_geom_ids=[i for i in range(tube_source.ngeom) if tube_source.geom_contype[i] or tube_source.geom_conaffinity[i]]
tidx=0
for g in ref['shapes']:
    istube=g['body_name'].startswith('tube/');position=np.array(g['position']);parent=tube if istube else world
    if istube:
        position-=ref['tube_initial_root_position'];sm=tube_source;si=tube_geom_ids[tidx];tidx+=1
    else:sm=source;si=g['id']
    attrs=dict(name=f"g{g['id']}",pos=fmt(position),quat=fmt(quat(g['rotation'])),
        contype=str(g['contype']),conaffinity=str(g['conaffinity']),density='0',
        friction=fmt(sm.geom_friction[si]),condim=str(sm.geom_condim[si]),solref=fmt(sm.geom_solref[si]),
        solimp=fmt(sm.geom_solimp[si]),margin=str(sm.geom_margin[si]),gap=str(sm.geom_gap[si]))
    if g['type']==7:attrs.update(type='mesh',mesh='mesh'+str(g['mesh_id']))
    elif g['type']==6:attrs.update(type='box',size=fmt(g['size']))
    elif g['type']==5:attrs.update(type='cylinder',size=fmt(g['size'][:2]))
    else:raise RuntimeError('Unsupported type')
    ET.SubElement(parent,'geom',attrs)
document=ET.tostring(tree,encoding='unicode');m=mj.MjModel.from_xml_string(document);d=mj.MjData(m);mj.mj_forward(m,d)
if args.solver_profile=='source_task':
    audited=json.loads((ROOT/'outputs/runtime_validation/5430_single_slot/source_task_audit.json').read_text(encoding='utf-8'))['options']
    for field in ('integrator','impratio','cone','noslip_iterations','enableflags'):setattr(m.opt,field,audited[field])
    # Keep diagnostic timestep fixed to isolate solver-option changes.
m.geom_friction[:,0]=args.friction
# Verify recompiled world collider origins/axes, including compiler mesh recentering.
max_pose_error=0.;max_mesh_vertex_error=0.
for g in ref['shapes']:
    if g['body_name'].startswith('tube/'):continue
    i=m.geom('g'+str(g['id'])).id
    if g['type']!=7:max_pose_error=max(max_pose_error,float(np.linalg.norm(d.geom_xpos[i]-g['position'])))
    else:
        mid=int(m.geom_dataid[i]);va=m.mesh_vertadr[mid];vn=m.mesh_vertnum[mid]
        actual=m.mesh_vert[va:va+vn]@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i]
        expected=np.array(ref['meshes'][str(g['mesh_id'])]['points'])@np.array(g['rotation']).T+g['position']
        if len(actual)!=len(expected):raise RuntimeError('Recompiled mesh vertex count differs')
        # Mesh vertex order is preserved by this compiler for these sources.
        max_mesh_vertex_error=max(max_mesh_vertex_error,float(np.max(np.linalg.norm(actual-expected,axis=1))))
if max_pose_error>1e-7 or max_mesh_vertex_error>1e-6:raise RuntimeError(str((max_pose_error,max_mesh_vertex_error)))
if args.evaluate_isaac:
    native=json.loads(args.evaluate_isaac.read_text(encoding='utf-8'));samples=[]
    if abs(native['offset_m']-args.offset)>1e-9:raise RuntimeError('Offset mismatch')
    for row in native['rows']:
        if row['step']%20 and row['step']!=native['rows'][-1]['step']:continue
        d.qpos[0]=row['slot_local'][2]-.02;d.qvel[0]=0;mj.mj_forward(m,d)
        contacts=[c for c in d.contact]
        samples.append(dict(step=row['step'],phase=row['phase'],z=row['slot_local'][2],
            min_source_contact_distance_m=min((float(c.dist) for c in contacts),default=None),contacts=len(contacts)))
    report=dict(scope='source-geometry penetration evaluated at recorded Isaac axial poses; sampled each20steps',
        native_report=str(args.evaluate_isaac),offset=args.offset,samples=samples,
        max_penetration_m=max((max(0,-r['min_source_contact_distance_m']) for r in samples if r['min_source_contact_distance_m'] is not None),default=0))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'}));sys.exit(0)
phases=[];rows=[];step=0
for phase,steps in schedule(args.protocol):
    peak=0.;minaxis=1.
    saturation=0
    for phase_step in range(steps):
        prelocal=R.T@(d.xpos[m.body('tube').id]-slot)
        force,zref,vref,saturated=command(args.protocol,phase,phase_step,steps,float(prelocal[2]),float(d.qvel[0]),ref['tube_mass_kg'],-9.81*R[2,2]);saturation+=int(saturated)
        d.qfrc_applied[0]=force;mj.mj_step(m,d);mj.mj_forward(m,d);step+=1
        pos=d.xpos[m.body('tube').id];local=R.T@(pos-slot);minaxis=min(minaxis,float(local[2]))
        dist=min((float(c.dist) for c in d.contact),default=None)
        rows.append(dict(step=step,phase=phase,q=float(d.qpos[0]),v=float(d.qvel[0]),slot_local=local.tolist(),
            contacts=int(d.ncon),min_contact_distance_m=dist,axial_input_n=force,z_ref=zref,v_ref=vref))
    phases.append(dict(phase=phase,end_slot_local=local.tolist(),target_distance_m=float(np.linalg.norm(local-[0,0,-.03])),min_axis_m=minaxis,saturation_steps=saturation))
end_insert=next(p for p in phases if p['phase']==('insert' if args.protocol=='force' else 'hold_insert'))
report=dict(mujoco_version=mj.__version__,scope='same geometry guided fixture, not original task',offset_m=args.offset,
    source_reference_sha256=hashlib.sha256(refpath.read_bytes()).hexdigest(),dt=.001,gravity=[0,0,-9.81],
    geometry_check=dict(max_nonmesh_origin_error_m=max_pose_error,max_mesh_vertex_error_m=max_mesh_vertex_error),
    protocol=design(args.protocol,ref['tube_mass_kg']),friction=args.friction,changed_friction_for_diagnosis=args.friction!=1,
    solver_profile=args.solver_profile,actual_options=dict(integrator=int(m.opt.integrator),impratio=float(m.opt.impratio),cone=int(m.opt.cone),noslip_iterations=int(m.opt.noslip_iterations),enableflags=int(m.opt.enableflags)),
    phases=phases,rows=rows,observations=dict(target_reached=end_insert['target_distance_m']<.005,
        withdrawn=phases[-1]['end_slot_local'][2]>.015),runtime_completed=True,full_task_validated=False)
args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
