"""Restore original shading and four official robot payloads, with a Z-up camera."""
import hashlib,json
from pathlib import Path
from validate_gpu_capacity import bare_usd
bare_usd()
from pxr import Usd,UsdGeom,UsdShade,Sdf,Gf
ROOT=Path(__file__).resolve().parents[1];SOURCE=ROOT/'assets/scenes/biolab_main_v06.usda'
OUT=ROOT/'outputs/gui_review';DEST=OUT/'biolab_v06_original_local.usda'
CACHE=Path.home()/'IsaacAssets/5.1'
PREFIX='https://omniverse-content-production.s3-us-west-2.amazonaws.com/'
source_hash=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
source=Usd.Stage.Open(str(SOURCE))
layer=Sdf.Layer.CreateNew(str(DEST)) if not DEST.exists() else Sdf.Layer.FindOrOpen(str(DEST))
layer.Clear();layer.subLayerPaths=[str(SOURCE)]
stage=Usd.Stage.Open(layer)
UsdGeom.SetStageUpAxis(stage,UsdGeom.GetStageUpAxis(source));UsdGeom.SetStageMetersPerUnit(stage,UsdGeom.GetStageMetersPerUnit(source))
stage.SetTimeCodesPerSecond(source.GetTimeCodesPerSecond());stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
mapping=[]
for p in list(source.Traverse()):
    pl=p.GetMetadata('payload')
    if not pl:continue
    items=pl.GetAddedOrExplicitItems()
    if not any(i.assetPath.startswith(PREFIX) for i in items):continue
    replacements=[]
    for item in items:
        path=CACHE/item.assetPath[len(PREFIX):]
        assert path.is_file(),path
        replacements.append(Sdf.Payload(str(path),item.primPath,item.layerOffset))
        mapping.append({'prim':str(p.GetPath()),'source_url':item.assetPath,'local':str(path)})
    stage.GetPrimAtPath(p.GetPath()).GetPayloads().SetPayloads(replacements)
eye=Gf.Vec3d(-4.8,-5.2,3.3);target=Gf.Vec3d(-1.4,-1.4,1.1)
cam=UsdGeom.Camera.Define(stage,'/GuiReviewCamera')
cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse())
cam.CreateClippingRangeAttr(Gf.Vec2f(.01,100));cam.CreateFocalLengthAttr(24)
layer.customLayerData={'source_sha256':source_hash,'reviewScope':'original shaders and local official robots; no autoplay','cameraSettings':{'Perspective':{'position':eye,'target':target},'boundCamera':'/GuiReviewCamera'}}
layer.Export(str(DEST));reopened=Usd.Stage.Open(str(DEST))
errors=[str(e) for p in reopened.Traverse() for e in p.GetPrimIndex().localErrors]
assert not errors,errors
shader_attributes=lambda st:{(str(p.GetPath()),a.GetName()):(str(a.Get()),[str(x) for x in a.GetConnections()]) for p in st.Traverse() if p.GetTypeName() in ('Material','Shader','NodeGraph') for a in p.GetAttributes()}
original_shading=shader_attributes(source);restored=shader_attributes(reopened)
assert all(restored.get(k)==v for k,v in original_shading.items()),'Original shading changed'
cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render])
robots=[]
for row in mapping:
    p=reopened.GetPrimAtPath(row['prim'])
    count=sum(1 for c in Usd.PrimRange(p) if c.IsA(UsdGeom.Mesh))
    # Instance proxies need explicit traversal for source instanced visuals.
    proxy_count=sum(1 for c in Usd.PrimRange(p,Usd.TraverseInstanceProxies()) if c.IsA(UsdGeom.Mesh))
    assert proxy_count>0,row
    robots.append({**row,'meshes':count,'meshes_including_instances':proxy_count,'bounds':str(cache.ComputeWorldBound(p).ComputeAlignedRange())})
assert len(robots)==4
assert hashlib.sha256(SOURCE.read_bytes()).hexdigest()==source_hash
report={'scene':str(DEST),'source_sha256':source_hash,'original_shading_attributes_connections_unchanged':True,'source_shading_attribute_count':len(original_shading),'up_axis':str(UsdGeom.GetStageUpAxis(reopened)),'meters_per_unit':UsdGeom.GetStageMetersPerUnit(reopened),'composition_errors':errors,'robots':robots,'gui_original_materials_verified':False,'simulation_started':False}
(OUT/'original_local_build.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps(report,indent=2))
