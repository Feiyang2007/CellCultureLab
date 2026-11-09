"""Isolated source-geometry contact test: static instrument and axial tube guide."""
import argparse,hashlib,json,os,tempfile,traceback
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];PILOT=ROOT/'outputs/runtime_validation/5430_single_slot'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--offset',type=float,default=0)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--protocol',choices=('force','servo'),default='force')
parser.add_argument('--friction',type=float,default=1.)
parser.add_argument('--contact-profile',choices=('rigid','source_timeconstant_candidate'),default='rigid')
args=parser.parse_args()
meta=json.loads((PILOT/'build.json').read_text(encoding='utf-8'))
if hashlib.sha256((PILOT/'scene.usda').read_bytes()).hexdigest()!=meta['scene_sha256']:raise RuntimeError('Scene changed')
home=Path.home()/'AppData/LocalLow/CellCultureLabReference/5430_runtime';home.mkdir(parents=True,exist_ok=True)
os.environ['TEMP']=os.environ['TMP']=str(home)
with tempfile.TemporaryFile(dir=home):pass
from isaacsim import AppFramework
import isaacsim
package=Path(isaacsim.__file__).parent
argv=[str(ROOT/'scripts/c1000_physics_only.kit'),'--no-window','--portable-root',str(home)]
for folder in (package/'exts',package/'extscache',package/'kit/exts',package/'kit/extscore'):
    if folder.is_dir():argv.extend(['--ext-folder',str(folder)])
app=AppFramework(argv=argv)
for _ in range(3):app.update()
import numpy as np
from contact_fixture_protocol import command,schedule,design
import omni.usd,omni.physx,omni.physics.tensors as tensors
from pxr import Gf,UsdUtils,UsdGeom
report=dict(runtime_completed=False,runtime_validated=False,scope=meta['fixture'],offset_m=args.offset,
    scene_sha256=meta['scene_sha256'],dt=.001,gravity=[0,0,-9.81],physics_only=True,
    force_protocol_newton=[dict(phase='insert',axial_force=-.05,steps=1800),dict(phase='withdraw',axial_force=.3,steps=500)],
    thresholds_declared_before_run=dict(target_distance_m=.005,withdraw_axis_min_m=.015,contact_force_detect_n=.001),
    full_task_validated=False,production_ready=False,rows=[],errors=[])
ref=json.loads((PILOT/'reference.json').read_text(encoding='utf-8'));mass=ref['tube_mass_kg']
report.update(protocol=design(args.protocol,mass),friction=args.friction,changed_friction_for_diagnosis=args.friction!=1)
report['contact_profile']=args.contact_profile
iface=omni.physx.get_physx_interface();view=None
try:
    ctx=omni.usd.get_context()
    if not ctx.open_stage(str(PILOT/'scene.usda')):raise RuntimeError('Stage open failed')
    stage=ctx.get_stage();stage.SetEditTarget(stage.GetSessionLayer())
    material=stage.GetPrimAtPath('/World/ContactMaterial')
    for name in ('physics:staticFriction','physics:dynamicFriction'):material.GetAttribute(name).Set(args.friction)
    if args.contact_profile=='source_timeconstant_candidate':
        from pxr import PhysxSchema
        original=json.loads((ROOT/'upstream/physics_reference/centrifuge_eppendorf_5430.json').read_text(encoding='utf-8'))
        parameters={(tuple(g['solref']),tuple(g['solimp'])) for g in original['geoms'] if g['contype'] or g['conaffinity']}
        if len(parameters)!=1:raise RuntimeError('Mixed source parameters require per-material mapping')
        solref,solimp=parameters.pop();tc,ratio=solref;dw=solimp[1]
        if min(tc,ratio,dw)<=0:raise RuntimeError('Unsupported source parameter format')
        k=1/(dw*tc*tc*ratio*ratio);damping=2/(dw*tc)
        ma=PhysxSchema.PhysxMaterialAPI.Apply(material)
        ma.CreateCompliantContactAccelerationSpringAttr(True)
        ma.CreateCompliantContactStiffnessAttr(k);ma.CreateCompliantContactDampingAttr(damping)
        report['compliant_contact_candidate']=dict(source_solref=solref,source_solimp=solimp,
            acceleration_stiffness_per_s2=k,acceleration_damping_per_s=damping,
            native_readback=dict(stiffness=ma.GetCompliantContactStiffnessAttr().Get(),damping=ma.GetCompliantContactDampingAttr().Get(),acceleration=ma.GetCompliantContactAccelerationSpringAttr().Get()),
            basis='MuJoCo reference-acceleration coefficients at d=d_width only; impedance and multipoint force law NOT equivalent',
            source_contact_equivalence_validated=False)
    R=np.array(meta['slot_rotation']);axis=R[:,2];slot=np.array(meta['slot_position'])
    start=np.array(meta['tube_start_position'])+args.offset*R[:,0]
    UsdGeom.Xformable(stage.GetPrimAtPath('/World/Tube')).GetOrderedXformOps()[0].Set(Gf.Vec3d(*start))
    stage.GetPrimAtPath('/World/Guide').GetAttribute('physics:localPos0').Set(Gf.Vec3f(*start))
    cache=UsdUtils.StageCache.Get();cache.Insert(stage)
    omni.physx.get_physx_simulation_interface().attach_stage(cache.GetId(stage).ToLongInt())
    iface.start_simulation();iface.update_simulation(.001,0)
    view=tensors.create_simulation_view('numpy');body=view.create_rigid_body_view('/World/Tube')
    if body.count!=1:raise RuntimeError('Expected one tube')
    contacts=view.create_rigid_contact_view('/World/Tube',max_contact_data_count=4096)
    indices=np.array([0],dtype=np.uint32);step=0;phases=[]
    print('RUNTIME_PHASE contacts_ready',flush=True)
    for phase,steps in schedule(args.protocol):
        maximum=0.;min_axis=1.
        saturation=0
        for phase_step in range(steps):
            pre_pose=body.get_transforms().copy()[0];pre_vel=body.get_velocities().copy()[0]
            z=float((R.T@(pre_pose[:3]-slot))[2]);v=float(np.dot(pre_vel[:3],axis))
            force,zref,vref,saturated=command(args.protocol,phase,phase_step,steps,z,v,mass,-9.81*axis[2]);saturation+=int(saturated)
            body.apply_forces(np.array([axis*force],dtype=np.float32),indices,True)
            iface.update_simulation(.001,(step+1)*.001);step+=1
            pose=body.get_transforms().copy()[0];vel=body.get_velocities().copy()[0]
            contact=contacts.get_net_contact_forces(.001).copy()[0]
            if not np.isfinite(np.concatenate([pose,vel,contact])).all():raise RuntimeError('Nonfinite state')
            local=R.T@(pose[:3]-slot);norm=float(np.linalg.norm(contact));maximum=max(maximum,norm);min_axis=min(min_axis,float(local[2]))
            report['rows'].append(dict(step=step,phase=phase,pose=pose.tolist(),velocity=vel.tolist(),slot_local=local.tolist(),contact_force_n=contact.tolist(),axial_input_n=force,z_ref=zref,v_ref=vref))
            if step%300==0:print('RUNTIME_PHASE '+json.dumps(dict(step=step,phase=phase,slot_z=float(local[2]),contact_n=norm)),flush=True)
        phases.append(dict(phase=phase,steps=steps,end_slot_local=local.tolist(),max_contact_n=maximum,min_axis_m=min_axis,
            saturation_steps=saturation,target_distance_m=float(np.linalg.norm(local-np.array([0,0,-.03])))))
    report['phases']=phases
    end_insert=next(p for p in phases if p['phase']==('insert' if args.protocol=='force' else 'hold_insert'))
    report['observations']=dict(contact_detected=max(p['max_contact_n'] for p in phases)>.001,
        target_reached=end_insert['target_distance_m']<.005,
        withdrawn=phases[-1]['end_slot_local'][2]>.015)
    report['runtime_completed']=True
except Exception as error:report['errors'].append(repr(error));traceback.print_exc()
finally:
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,allow_nan=False),encoding='utf-8')
    print('REPORT_JSON '+json.dumps({k:v for k,v in report.items() if k!='rows'},allow_nan=False),flush=True)
    if view is not None:view.invalidate()
    iface.reset_simulation()
    omni.physx.get_physx_simulation_interface().detach_stage()
    app.close()
