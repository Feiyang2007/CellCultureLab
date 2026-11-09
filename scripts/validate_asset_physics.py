"""Isolated physics acceptance for reusable assets; no task sequence or robot demo."""
import os,sys,json,time
from pathlib import Path
P=Path(__file__).resolve().parent.parent
runtime=Path(os.environ['LOCALAPPDATA'])/'IsaacSim/5.1/CellCultureLab-validation'
sys.argv=[sys.argv[0],'--portable-root',runtime.as_posix()]
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'disable_viewport_updates':True,'multi_gpu':False,'limit_cpu_threads':4,'extra_args':[f'--/log/file={(runtime/"physics-validation.log").as_posix()}']})
try:
    import omni.usd
    from pxr import Usd,UsdGeom,UsdPhysics,Gf,Sdf
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleRigidPrim
    from lab_assets import LabAssets
    from build_physics_fixture import build
    if not (P/'assets/interactive/validation/rack_fixture.usda').exists():build()
    world=World(stage_units_in_meters=1,physics_dt=1/240,rendering_dt=1/60,device='cpu')
    stage=omni.usd.get_context().get_stage()
    tool=UsdGeom.Xform.Define(stage,LabAssets.PIPETTE).GetPrim()
    tool.GetReferences().AddReference(str(P/'assets/interactive/pipette_200ul.usda'))
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(LabAssets.PIPETTE+'/Body')).CreateKinematicEnabledAttr(True)
    stage.GetPrimAtPath(LabAssets.PIPETTE+'/Body').GetAttribute('physxRigidBody:enableCCD').Set(False)
    for prim in Usd.PrimRange(tool):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.CreateAttribute('physxRigidBody:disableGravity',Sdf.ValueTypeNames.Bool).Set(True)
    tip=UsdGeom.Xform.Define(stage,'/World/Tip')
    tip.GetPrim().GetReferences().AddReference(str(P/'assets/interactive/tip_200ul.usda'))
    tip.AddTranslateOp().Set(Gf.Vec3d(0.3,0,0.12))
    floor=UsdGeom.Cube.Define(stage,'/World/Support')
    floor.CreateSizeAttr(1);floor.AddTranslateOp().Set(Gf.Vec3d(0,0,-0.005));floor.AddScaleOp().Set(Gf.Vec3d(1,1,0.01))
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    views={}
    for name,path in [('plunger',LabAssets.PIPETTE+'/pipette_button'),('ejector',LabAssets.PIPETTE+'/pipette_ejector'),('knob',LabAssets.PIPETTE+'/pipette_knob'),('tip','/World/Tip')]:
        views[name]=world.scene.add(SingleRigidPrim(prim_path=path,name=name))
    rackroot=UsdGeom.Xform.Define(stage,'/World/RackTest')
    rackroot.GetPrim().GetReferences().AddReference(str(P/'assets/interactive/validation/rack_fixture.usda'))
    rack_views={}
    for name in json.loads((P/'assets/interactive/validation/rack_fixture.json').read_text()):
        rack_views[name]=world.scene.add(SingleRigidPrim(prim_path='/World/RackTest/Tips/'+name,name=name))
    print('PHYSICS_INITIALIZING',flush=True)
    world.reset()
    print('PHYSICS_READY',flush=True)
    api=LabAssets(stage)
    targets={'plunger':-0.006,'ejector':-0.007,'volume_knob':30.0}
    for name,value in targets.items():api.set_control(name,value)
    start=time.monotonic()
    for i in range(120):
        world.step(render=False)
        if i%20==19:print(f'PHYSICS_STEP {i+1}/120 elapsed={time.monotonic()-start:.1f}s',flush=True)
    bodypos=[0,0,0]
    measured={name:float(views[name].get_world_pose()[0][2]-bodypos[2]) for name in ['plunger','ejector']}
    q=views['knob'].get_world_pose()[1]
    measured['volume_knob']=__import__('math').degrees(2*__import__('math').atan2(float(q[3]),float(q[0])))
    tip_position=views['tip'].get_world_pose()[0].tolist()
    from math import sqrt
    tip_q=views['tip'].get_world_pose()[1]
    pose=Gf.Matrix4d().SetRotate(Gf.Quatd(float(tip_q[0]),Gf.Vec3d(*map(float,tip_q[1:]))))
    pose.SetTranslateOnly(Gf.Vec3d(*tip_position))
    rootprim=stage.GetPrimAtPath('/World/Tip')
    xc=UsdGeom.XformCache();rootinv=xc.GetLocalToWorldTransform(rootprim).GetInverse()
    min_z=1e9
    for prim in Usd.PrimRange(rootprim):
        if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI):
            local=xc.GetLocalToWorldTransform(prim)*rootinv
            for point in UsdGeom.Mesh(prim).GetPointsAttr().Get():
                min_z=min(min_z,pose.Transform(local.Transform(Gf.Vec3d(point)))[2])
    initial=json.loads((P/'assets/interactive/validation/rack_fixture.json').read_text())
    drift={name:sqrt(sum((float(view.get_world_pose()[0][i])-initial[name][i])**2 for i in range(3))) for name,view in rack_views.items()}
    checks={
        'plunger_follows_drive':abs(measured['plunger']-targets['plunger'])<0.001,
        'ejector_follows_drive':abs(measured['ejector']-targets['ejector'])<0.001,
        'knob_follows_drive':abs(measured['volume_knob']-targets['volume_knob'])<3,
        'tip_supported_by_collision':-0.001<min_z<0.002 and float(tip_position[2])<0.07,
        'all_96_tips_remain_seated':max(drift.values())<0.015,
    }
    result={'passed':all(checks.values()),'checks':checks,'targets':targets,'measured':measured,'tip_final_world_position_m':tip_position,'tip_min_collider_world_z_m':min_z,'rack_tip_max_drift_m':max(drift.values()),'rack_tips_over_15mm':[k for k,v in drift.items() if v>=0.015],'physics_steps':120,'physics_wall_time_s':time.monotonic()-start,'stability_window_s':0.5,'physics_dt_s':1/240}
    (P/'docs/physics_validation_v04.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('PHYSICS_RESULT '+json.dumps(result),flush=True)
finally:app.close()
