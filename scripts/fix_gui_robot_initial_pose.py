"""Materialize the scene's authored robot joint state into pre-play USD transforms.

No simulator is launched. Joint targets, mount positions and equipment stay fixed.
"""
import json,math
from pathlib import Path
from validate_gpu_capacity import bare_usd
bare_usd()
from pxr import Usd,UsdGeom,UsdPhysics,Sdf,Gf
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'outputs/gui_review'
SOURCE=OUT/'biolab_v06_original_local.usda';DEST=OUT/'biolab_v06_original_parked.usda'
src=Usd.Stage.Open(str(SOURCE))
layer=Sdf.Layer.CreateNew(str(DEST)) if not DEST.exists() else Sdf.Layer.FindOrOpen(str(DEST))
layer.Clear();layer.subLayerPaths=[str(SOURCE)]
stage=Usd.Stage.Open(layer)
UsdGeom.SetStageUpAxis(stage,UsdGeom.GetStageUpAxis(src));UsdGeom.SetStageMetersPerUnit(stage,UsdGeom.GetStageMetersPerUnit(src))
stage.SetTimeCodesPerSecond(src.GetTimeCodesPerSecond());stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
layer.customLayerData=src.GetRootLayer().customLayerData
xf=UsdGeom.XformCache();world={};rows=[]
def frame(j,idx):
    pos=j.GetAttribute('physics:localPos'+str(idx)).Get() or Gf.Vec3f(0)
    rot=j.GetAttribute('physics:localRot'+str(idx)).Get() or Gf.Quatf(1)
    return Gf.Matrix4d(Gf.Rotation(Gf.Quatd(rot)),Gf.Vec3d(pos))
def getworld(path):
    return world.get(path,xf.GetLocalToWorldTransform(src.GetPrimAtPath(path)))
def writeworld(path,matrix):
    p=stage.GetPrimAtPath(path);parent=p.GetParent().GetPath()
    local=matrix*getworld(parent).GetInverse()
    x=UsdGeom.Xformable(p);x.ClearXformOpOrder();x.AddTransformOp(opSuffix='initialPose').Set(local)
    world[path]=matrix
for arm in (1,2):
    root=f'/World/Robot/Robot{arm}Mount/ur5e'
    joints=[p for p in Usd.PrimRange(src.GetPrimAtPath(root)) if p.IsA(UsdPhysics.RevoluteJoint)]
    pending=joints[:];done=set()
    while pending:
        progress=False
        for j in pending[:]:
            a=j.GetRelationship('physics:body0').GetTargets();b=j.GetRelationship('physics:body1').GetTargets()
            if len(a)!=1 or len(b)!=1:raise RuntimeError(str(j.GetPath()))
            # Resolve from base outward, not alphabetical joint order.
            parent_joint=[other for other in joints if other.GetRelationship('physics:body1').GetTargets()==a]
            if parent_joint and a[0] not in done:continue
            q=j.GetAttribute('state:angular:physics:position').Get()
            if q is None:raise RuntimeError('Missing declared initial state: '+str(j.GetPath()))
            axis=j.GetAttribute('physics:axis').Get();vec={'X':Gf.Vec3d(1,0,0),'Y':Gf.Vec3d(0,1,0),'Z':Gf.Vec3d(0,0,1)}[axis]
            rot=Gf.Matrix4d(Gf.Rotation(vec,float(q)),Gf.Vec3d(0))
            desired=frame(j,1).GetInverse()*rot*frame(j,0)*getworld(a[0])
            writeworld(b[0],desired);done.add(b[0]);pending.remove(j);progress=True
            rows.append({'joint':str(j.GetPath()),'declared_q_degrees':q,'body':str(b[0])})
        if not progress:raise RuntimeError('Joint graph unresolved')
    groot=f'/World/Robot/Gripper{arm}Temp/Robotiq_2F_85_edit/Robotiq_2F_85'
    fixed=src.GetPrimAtPath(groot+'/base_link/AssemblerFixedJoint')
    a=fixed.GetRelationship('physics:body0').GetTargets()[0];b=fixed.GetRelationship('physics:body1').GetTargets()[0]
    desired=frame(fixed,1).GetInverse()*frame(fixed,0)*getworld(a)
    delta=getworld(b).GetInverse()*desired
    writeworld(Sdf.Path(groot),getworld(Sdf.Path(groot))*delta)
layer.Export(str(DEST));stage=Usd.Stage.Open(str(DEST));cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render])
therm=cache.ComputeWorldBound(stage.GetPrimAtPath('/World/TaskArea/Equipment/ThermalCycler')).ComputeAlignedRange()
overlaps=[];bounds=[]
for p in Usd.PrimRange(stage.GetPrimAtPath('/World/Robot'),Usd.TraverseInstanceProxies()):
    if not p.IsA(UsdGeom.Mesh):continue
    if UsdGeom.Imageable(p).ComputeVisibility()=='invisible':continue
    box=cache.ComputeWorldBound(p).ComputeAlignedRange()
    a,b=box.GetMin(),box.GetMax();c,d=therm.GetMin(),therm.GetMax()
    penetration=[min(b[i],d[i])-max(a[i],c[i]) for i in range(3)]
    if all(v>0 for v in penetration):overlaps.append({'mesh':str(p.GetPath()),'aabb_overlap_m':penetration})
    bounds.append({'mesh':str(p.GetPath()),'min':list(a),'max':list(b)})
report={'scene':str(DEST),'joints':rows,'thermal_cycler_bounds':str(therm),'robot_meshes_with_aabb_overlap':overlaps,'robot_mesh_bounds':bounds,'gui_verified':False,'simulation_tested':False}
(OUT/'parked_pose_audit.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in report.items() if k!='robot_mesh_bounds'},indent=2))
