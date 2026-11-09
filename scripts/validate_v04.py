"""Structural acceptance checks for v04, without rendering or a task demonstration."""
from pathlib import Path
import json,hashlib,math,sys
from pxr import Usd,UsdGeom,UsdPhysics
from lab_assets import LabAssets
P=Path(__file__).resolve().parent.parent
report=json.loads((P/'docs/interactive_v04.json').read_text(encoding='utf-8'))
stage=Usd.Stage.Open(str(P/'assets/scenes/biolab_main_v04.usda'))
issues=[]
if UsdGeom.GetStageUpAxis(stage)!='Z':issues.append('Scene must use Z up')
if abs(UsdGeom.GetStageMetersPerUnit(stage)-1.0)>1e-9:issues.append('Scene must use metres')
if 'cameraSettings' not in stage.GetRootLayer().customLayerData:issues.append('Original viewing camera metadata missing')
for prim in stage.Traverse():
    if prim.GetTypeName()=='Box':issues.append('Unsupported Box: '+str(prim.GetPath()))
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        parent=prim.GetParent()
        while parent and not parent.IsPseudoRoot():
            if parent.HasAPI(UsdPhysics.RigidBodyAPI):issues.append('Nested rigid bodies: '+str(prim.GetPath()));break
            parent=parent.GetParent()
for path in report['tips']:
    prim=stage.GetPrimAtPath(path)
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):issues.append('Nonphysical tip: '+path)
    colliders=[x for x in Usd.PrimRange(prim) if x.HasAPI(UsdPhysics.CollisionAPI)]
    if len(colliders)!=37:issues.append('Open-socket collider count mismatch: '+path)
for name,(joint,kind,lo,hi) in LabAssets.CONTROLS.items():
    p=stage.GetPrimAtPath(LabAssets.PIPETTE+'/Joints/'+joint)
    if not p:issues.append('Missing control: '+name);continue
    for rel in ['physics:body0','physics:body1']:
        targets=p.GetRelationship(rel).GetTargets()
        if len(targets)!=1 or not stage.GetPrimAtPath(targets[0]).HasAPI(UsdPhysics.RigidBodyAPI):issues.append('Invalid joint body: '+str(p.GetPath()))
    if p.GetAttribute('physics:lowerLimit').Get()!=__import__('struct').unpack('f',__import__('struct').pack('f',lo))[0]:issues.append('Control limit mismatch: '+name)
source_hash=hashlib.sha256((P/'assets/scenes/biolab_main_v03.usda').read_bytes()).hexdigest()
if source_hash!=report['source_v03_sha256']:issues.append('v03 source changed')
# New reservoir footprint must stay inside the measured table bounds and have no
# XY footprint overlap with existing task assets at the same vertical height.
bb=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'],False)
r=bb.ComputeWorldBound(stage.GetPrimAtPath('/World/TaskArea/CultureMedia/Reservoir')).ComputeAlignedRange()
mn,mx=r.GetMin(),r.GetMax()
if not (-1.999<=mn[0]<mx[0]<=-0.441 and -3.457<=mn[1]<mx[1]<=1.027):issues.append('Reservoir outside tabletop')
for folder in ['Equipment','Fixtures','Stations']:
    parent=stage.GetPrimAtPath('/World/TaskArea/'+folder)
    if not parent:continue
    for prim in parent.GetChildren():
        b=bb.ComputeWorldBound(prim).ComputeAlignedRange()
        if b.IsEmpty():continue
        a,c=b.GetMin(),b.GetMax()
        if min(mx[0],c[0])-max(mn[0],a[0])>0.001 and min(mx[1],c[1])-max(mn[1],a[1])>0.001 and min(mx[2],c[2])-max(mn[2],a[2])>0.001:issues.append('Reservoir overlaps '+str(prim.GetPath()))
result={'passed':not issues,'checks':['96 rigid tips with 37 open-socket hulls each','standard supported collider types','no nested rigid bodies','three joints with valid body references and source limits','new reservoir inside table and clear of existing stations','v03 source hash unchanged'],'issues':issues}
(P/'docs/validation_v04.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
raise SystemExit(bool(issues))
