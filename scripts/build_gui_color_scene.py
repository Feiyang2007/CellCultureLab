"""Source-colour preview override; never edits the source scene or assets.

This is an approximation of VRay MDL, not a render-equivalent material bake.
"""
import hashlib,json,re
from pathlib import Path
from validate_gpu_capacity import bare_usd
bare_usd()
from pxr import Usd,UsdGeom,UsdShade,Sdf,Gf
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'assets/scenes/biolab_main_v06.usda'
OUT=ROOT/'outputs/gui_review';DEST=OUT/'biolab_v06_color_zup.usda'
source_hash=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
source=Usd.Stage.Open(str(SOURCE))
layer=Sdf.Layer.CreateNew(str(DEST)) if not DEST.exists() else Sdf.Layer.FindOrOpen(str(DEST))
layer.Clear();layer.subLayerPaths=[str(SOURCE)]
stage=Usd.Stage.Open(layer)
UsdGeom.SetStageUpAxis(stage,UsdGeom.GetStageUpAxis(source))
UsdGeom.SetStageMetersPerUnit(stage,UsdGeom.GetStageMetersPerUnit(source))
stage.SetTimeCodesPerSecond(source.GetTimeCodesPerSecond())
stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
rows=[];excluded=[]
for p in list(stage.Traverse()):
    pl=p.GetMetadata('payload')
    if pl and any(i.assetPath.startswith(('http:','https:','omniverse:')) for i in pl.GetAddedOrExplicitItems()):
        excluded.append(str(p.GetPath()));p.GetPayloads().SetPayloads([])

def scalar(text,name,default):
    m=re.search(r'\b'+name+r'\s*:\s*([-+\d.eE]+)f?',text)
    return float(m[1]) if m else default

def texture(mat,shader,path,scale=(1,1),offset=(0,0)):
    uv=UsdShade.Shader.Define(stage,mat.GetPath().AppendChild('ReviewUV'))
    uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
    uv.CreateOutput('result',Sdf.ValueTypeNames.Float2)
    transform=UsdShade.Shader.Define(stage,mat.GetPath().AppendChild('ReviewUVTransform'))
    transform.CreateIdAttr('UsdTransform2d')
    transform.CreateInput('in',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
    transform.CreateInput('scale',Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*scale))
    transform.CreateInput('translation',Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*offset))
    transform.CreateOutput('result',Sdf.ValueTypeNames.Float2)
    tex=UsdShade.Shader.Define(stage,mat.GetPath().AppendChild('ReviewAlbedo'))
    tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(path)))
    tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB')
    tex.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('repeat');tex.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('repeat')
    tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(transform.ConnectableAPI(),'result')
    tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb')

for p in list(source.Traverse()):
    if not p.IsA(UsdShade.Material):continue
    original=UsdShade.Material(p);mat=UsdShade.Material.Get(stage,p.GetPath())
    children=[UsdShade.Shader(c) for c in p.GetChildren() if c.IsA(UsdShade.Shader)]
    mdl=[s for s in children if s.GetIdAttr().Get()!='UsdPreviewSurface' and s.GetSourceAsset('mdl')]
    preview=[s for s in children if s.GetIdAttr().Get()=='UsdPreviewSurface']
    # Remove all MDL surface routes, including duplicate routes on preview shaders.
    for o in mat.GetOutputs():
        if o.GetBaseName().startswith('mdl:'):o.GetAttr().SetConnections([])
    for s in mdl:stage.OverridePrim(s.GetPath()).SetActive(False)
    if preview:
        sh=UsdShade.Shader.Get(stage,preview[0].GetPath())
        sh.CreateImplementationSourceAttr().Set('id')
        for a in sh.GetPrim().GetAttributes():
            if a.GetName().startswith('info:mdl:'):a.Block()
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
        rows.append({'material':str(p.GetPath()),'mode':'existing_preview_preserved'})
        continue
    if not mdl:continue
    s=mdl[0];asset=s.GetSourceAsset('mdl');filepath=Path(asset.resolvedPath) if asset.resolvedPath else None
    color=None;rough=.5;metal=0.;opacity=1.;texpath=None;scale=(1,1);offset=(0,0)
    row={'material':str(p.GetPath()),'mode':'mdl_approximation','source':str(asset),'unmatched':[]}
    value=s.GetInput('diffuse_color_constant')
    if value and value.Get() is not None:color=tuple(value.Get())
    value=s.GetInput('diffuse_texture')
    if value and value.Get() and value.Get().path:
        texpath=Path(value.Get().resolvedPath) if value.Get().resolvedPath else None
        if not texpath:row['unmatched'].append('unresolved_albedo:'+value.Get().path)
    for n,target in [('reflection_roughness_constant','rough'),('metallic_constant','metal'),('opacity_constant','opacity')]:
        inp=s.GetInput(n)
        if inp and inp.Get() is not None:
            if target=='rough':rough=float(inp.Get())
            elif target=='metal':metal=float(inp.Get())
            else:opacity=float(inp.Get())
    if filepath and filepath.is_file():
        text=filepath.read_text(encoding='utf8',errors='replace')
        omni=re.search(r'\bdiffuse_color_constant\s*:\s*color\(([^)]+)\)',text)
        if omni and color is None:
            vals=[float(x.strip().rstrip('f')) for x in omni[1].split(',')]
            color=tuple(vals*3 if len(vals)==1 else vals)
        diff=re.search(r'\bDiffuse\s*:(.*?)(?:\n\s*diffuse_roughness\s*:)',text,re.S)
        part=diff[1] if diff else ''
        c=re.search(r'^\s*color\(([^)]+)\)',part)
        if c:
            vals=[float(x.strip().rstrip('f')) for x in c[1].split(',')]
            color=tuple(vals*3 if len(vals)==1 else vals)
        tm=re.search(r'texture_2d\("([^"]+)"',part)
        if tm:
            texpath=(filepath.parent/tm[1]).resolve()
            if not texpath.is_file():row['unmatched'].append('missing_albedo:'+str(texpath));texpath=None
            else:
                scale=(scalar(part,'clipw',1)*scalar(part,'U_Tiling',1),scalar(part,'cliph',1)*scalar(part,'V_Tiling',1))
                offset=(scalar(part,'clipu',0)+scalar(part,'U_Offset',0),scalar(part,'clipv',0)+scalar(part,'V_Offset',0))
                if any(scalar(part,n,0)!=0 for n in ['U_angle','V_angle','W_angle']):row['unmatched'].append('texture_rotation_not_translated')
        rough=scalar(text,'reflection_roughness_constant',.5) if omni else max(.04,1-scalar(text,'reflection_glossiness',.5))
        metal=scalar(text,'metallic_constant',0) if omni else scalar(text,'reflection_metalness',0)
        row['unmatched'].append('VRay_reflection_refraction_procedural_response_not_equivalent')
    elif color is None and texpath is None:row['unmatched'].append('no_extractable_source_colour')
    if color is None and not texpath:color=(.18,.18,.18);row['fallback_colour']=True
    sh=UsdShade.Shader.Define(stage,p.GetPath().AppendChild('ReviewPreview'))
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(color or (1,1,1))))
    sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metal)
    sh.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(opacity)
    sh.CreateOutput('surface',Sdf.ValueTypeNames.Token)
    if texpath:texture(mat,sh,texpath,scale,offset)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
    row.update(color=color,albedo=str(texpath) if texpath else None,roughness=rough,metallic=metal)
    rows.append(row)

# Explicit Z-up camera; geometry and physical transforms are untouched.
eye=Gf.Vec3d(-4.8,-5.2,3.3);target=Gf.Vec3d(-1.4,-1.4,1.1)
cam=UsdGeom.Camera.Define(stage,'/GuiReviewCamera')
cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse())
cam.CreateClippingRangeAttr(Gf.Vec2f(.01,100));cam.CreateFocalLengthAttr(24)
layer.customLayerData={'source_sha256':source_hash,'reviewScope':'source-colour approximation; remote robots absent; physics untested','cameraSettings':{'Perspective':{'position':eye,'target':target},'boundCamera':'/GuiReviewCamera'}}
layer.Export(str(DEST))
reopened=Usd.Stage.Open(str(DEST))
assert UsdGeom.GetStageUpAxis(reopened)=='Z' and UsdGeom.GetStageMetersPerUnit(reopened)==1
geom=lambda st:{str(p.GetPath()):p.GetTypeName() for p in st.Traverse() if p.IsA(UsdGeom.Gprim)}
assert geom(source)==geom(reopened)
physical=lambda st:{(str(p.GetPath()),a.GetName()):str(a.Get()) for p in st.Traverse() for a in p.GetAttributes() if a.GetName().startswith(('physics:','physx'))}
assert physical(source)==physical(reopened)
errors=[str(e) for p in reopened.Traverse() for e in p.GetPrimIndex().localErrors]
assert not errors
assert hashlib.sha256(SOURCE.read_bytes()).hexdigest()==source_hash
report={'scene':str(DEST),'up_axis':str(UsdGeom.GetStageUpAxis(reopened)),'meters_per_unit':UsdGeom.GetStageMetersPerUnit(reopened),'source_sha256':source_hash,'geometry_paths_types_unchanged':True,'physical_attributes_unchanged':True,'composition_errors':errors,'excluded_remote_payloads':excluded,'material_rows':rows,'original_appearance_equivalent':False,'gui_verified':False}
(OUT/'color_build.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in report.items() if k!='material_rows'},indent=2))
print('materials',len(rows),'existing previews',sum(r['mode']=='existing_preview_preserved' for r in rows),'textures',sum(bool(r.get('albedo')) for r in rows),'fallbacks',sum(bool(r.get('fallback_colour')) for r in rows))
