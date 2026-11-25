"""Single sphere/plane normal-contact calibration, isolated from insertion.

One fitted load (0.05N), three held-out loads; same mass, shape, timestep,
initial pose, force and gravity. Never declares task/contact equivalence.
"""
import argparse,json,math,os,sys,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--backend',choices=('mujoco','isaac'),required=True)
parser.add_argument('--stiffness',type=float,default=200.)
parser.add_argument('--damping',type=float,default=.5)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
home=Path.home()/'AppData/LocalLow/CellCultureLabReference/contact_calibration';home.mkdir(parents=True,exist_ok=True)
os.environ['TEMP']=os.environ['TMP']=str(home)
ref=json.loads((ROOT/'outputs/runtime_validation/5430_single_slot/reference.json').read_text(encoding='utf-8'))
mass=ref['tube_mass_kg'];radius=.00535;dt=.001;steps=600;loads=[.05,.02,.1,.2]
report=dict(runtime_completed=False,backend=args.backend,mass_kg=mass,radius_m=radius,dt=dt,gravity=[0,0,0],
    source_solref=[.02,1],source_solimp=[.9,.95,.001,.5,2],fit_load_n=.05,held_out_loads_n=[.02,.1,.2],
    native_stiffness_n_per_m=args.stiffness,native_damping_n_s_per_m=args.damping,
    thresholds_declared_before_run=dict(heldout_relative_depth_error=.2,transient_depth_rms_m=.00005,velocity_max_difference_m_s=.02),
    scope='single normal contact only; no friction or multipoint or task equivalence',full_task_validated=False,
    runs=[],errors=[])
app=None;view=None
try:
    if args.backend=='mujoco':
        sys.path.insert(0,str(Path.home()/'AppData/LocalLow/CellCultureLabReference/mujoco330'))
        import mujoco as mj
        if mj.__version__!='3.3.0':raise RuntimeError('Requires3.3.0')
        m=mj.MjModel.from_xml_string(f'''<mujoco><option timestep=".001" gravity="0 0 0" integrator="implicitfast"/>
        <default><geom friction="0 0 0" condim="1" solref=".02 1" solimp=".9 .95 .001 .5 2"/></default>
        <worldbody><geom type="plane" size="1 1 .1"/><body name="ball" pos="0 0 {radius}"><freejoint/>
        <inertial mass="{mass}" pos="0 0 0" diaginertia="{mass*radius*radius*.4} {mass*radius*radius*.4} {mass*radius*radius*.4}"/>
        <geom type="sphere" size="{radius}"/></body></worldbody></mujoco>''')
        for load in loads:
            d=mj.MjData(m);mj.mj_forward(m,d);rows=[]
            for i in range(steps):
                d.qfrc_applied[2]=-load;mj.mj_step(m,d);mj.mj_forward(m,d)
                normal=0.
                import numpy as np
                for k in range(d.ncon):
                    f=np.zeros(6);mj.mj_contactForce(m,d,k,f);normal+=float(f[0])
                rows.append(dict(step=i+1,z=float(d.qpos[2]),v=float(d.qvel[2]),depth_m=radius-float(d.qpos[2]),contact_n=normal))
            report['runs'].append(dict(load_n=load,role='fit' if load==.05 else 'held_out',rows=rows))
    else:
        from isaacsim import AppFramework
        import isaacsim
        package=Path(isaacsim.__file__).parent
        argv=[str(ROOT/'scripts/c1000_physics_only.kit'),'--no-window','--portable-root',str(home)]
        for folder in (package/'exts',package/'extscache',package/'kit/exts',package/'kit/extscore'):
            if folder.is_dir():argv.extend(['--ext-folder',str(folder)])
        app=AppFramework(argv=argv)
        for _ in range(3):app.update()
        import numpy as np
        import omni.physx,omni.physics.tensors as tensors
        from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema,UsdUtils,Gf
        iface=omni.physx.get_physx_interface();sim=omni.physx.get_physx_simulation_interface();cache=UsdUtils.StageCache.Get()
        for load in loads:
            stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
            scene=UsdPhysics.Scene.Define(stage,'/World/Scene');scene.CreateGravityMagnitudeAttr(0)
            ps=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim());ps.CreateEnableGPUDynamicsAttr(False);ps.CreateBroadphaseTypeAttr('MBP');ps.CreateTimeStepsPerSecondAttr(1000)
            mat=UsdShade.Material.Define(stage,'/World/Material');ma=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            ma.CreateStaticFrictionAttr(0);ma.CreateDynamicFrictionAttr(0);ma.CreateRestitutionAttr(0)
            pm=PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim());pm.CreateCompliantContactAccelerationSpringAttr(False)
            pm.CreateCompliantContactStiffnessAttr(args.stiffness);pm.CreateCompliantContactDampingAttr(args.damping)
            plane=UsdGeom.Cube.Define(stage,'/World/Plane');plane.CreateSizeAttr(2);plane.AddTranslateOp().Set(Gf.Vec3d(0,0,-.01));plane.AddScaleOp().Set(Gf.Vec3f(.5,.5,.01))
            ball=UsdGeom.Sphere.Define(stage,'/World/Ball');ball.CreateRadiusAttr(radius);ball.AddTranslateOp().Set(Gf.Vec3d(0,0,radius))
            UsdPhysics.RigidBodyAPI.Apply(ball.GetPrim());mb=UsdPhysics.MassAPI.Apply(ball.GetPrim());mb.CreateMassAttr(mass)
            mb.CreateCenterOfMassAttr(Gf.Vec3f(0));mb.CreateDiagonalInertiaAttr(Gf.Vec3f(mass*radius*radius*.4))
            rb=PhysxSchema.PhysxRigidBodyAPI.Apply(ball.GetPrim());rb.CreateLinearDampingAttr(0);rb.CreateAngularDampingAttr(0)
            PhysxSchema.PhysxContactReportAPI.Apply(ball.GetPrim()).CreateThresholdAttr(0)
            for shape in (plane,ball):
                UsdPhysics.CollisionAPI.Apply(shape.GetPrim());co=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim());co.CreateContactOffsetAttr(.0002);co.CreateRestOffsetAttr(0)
                UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(mat,materialPurpose='physics')
            cache.Insert(stage);sim.attach_stage(cache.GetId(stage).ToLongInt());iface.start_simulation();iface.update_simulation(dt,0)
            view=tensors.create_simulation_view('numpy');body=view.create_rigid_body_view('/World/Ball');contacts=view.create_rigid_contact_view('/World/Ball')
            indices=np.array([0],dtype=np.uint32);rows=[]
            for i in range(steps):
                body.apply_forces(np.array([[0,0,-load]],dtype=np.float32),indices,True);iface.update_simulation(dt,(i+1)*dt)
                z=float(body.get_transforms()[0,2]);v=float(body.get_velocities()[0,2]);normal=float(contacts.get_net_contact_forces(dt)[0,2])
                if not all(math.isfinite(x) for x in (z,v,normal)):raise RuntimeError('Nonfinite state')
                rows.append(dict(step=i+1,z=z,v=v,depth_m=radius-z,contact_n=normal))
            report['runs'].append(dict(load_n=load,role='fit' if load==.05 else 'held_out',rows=rows))
            view.invalidate();view=None;iface.reset_simulation();sim.detach_stage();cache.Erase(stage);tensors.reset()
    for run in report['runs']:
        tail=run['rows'][-100:]
        run.update(steady_depth_m=sum(r['depth_m'] for r in tail)/len(tail),max_abs_v_m_s=max(abs(r['v']) for r in run['rows']),
            steady_contact_n=sum(r['contact_n'] for r in tail)/len(tail))
    report['runtime_completed']=True
except Exception as e:report['errors'].append(repr(e));traceback.print_exc()
finally:
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,allow_nan=False),encoding='utf-8')
    summary={k:v for k,v in report.items() if k!='runs'};summary['runs']=[{k:v for k,v in r.items() if k!='rows'} for r in report['runs']]
    print('REPORT_JSON '+json.dumps(summary,allow_nan=False),flush=True)
    if view is not None:view.invalidate()
    if app is not None:app.close()
