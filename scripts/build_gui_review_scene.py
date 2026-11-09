"""Create white-material GUI overrides, preserving the original scene layers."""
import hashlib,json
from pathlib import Path
from validate_gpu_capacity import bare_usd
bare_usd()
from pxr import Usd,UsdGeom,UsdShade,Sdf,Gf
ROOT=Path(__file__).resolve().parents[1];SOURCE=ROOT/'assets/scenes/biolab_main_v06.usda'
OUT=ROOT/'outputs/gui_review';OUT.mkdir(parents=True,exist_ok=True)
source_hash=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
source_stage=Usd.Stage.Open(str(SOURCE))
reports=[]
for preview in (False,True):
    dest=OUT/('biolab_v06_white_preview.usda' if preview else 'biolab_v06_white_physics.usda')
    layer=Sdf.Layer.CreateNew(str(dest)) if not dest.exists() else Sdf.Layer.FindOrOpen(str(dest))
    layer.Clear();layer.subLayerPaths=[str(SOURCE)]
    stage=Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(stage,UsdGeom.GetStageUpAxis(source_stage))
    UsdGeom.SetStageMetersPerUnit(stage,UsdGeom.GetStageMetersPerUnit(source_stage))
    stage.SetTimeCodesPerSecond(source_stage.GetTimeCodesPerSecond())
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    all_prims=list(stage.Traverse())
    source_geometry={str(p.GetPath()):p.GetTypeName() for p in all_prims if p.IsA(UsdGeom.Gprim)}
    excluded_remote=[]
    for p in all_prims:
        payload=p.GetMetadata('payload')
        if payload and any(str(item.assetPath).startswith(('http:','https:','omniverse:')) for item in payload.GetAddedOrExplicitItems()):
            excluded_remote.append(str(p.GetPath()));p.GetPayloads().SetPayloads([])
    shading=[p.GetPath() for p in all_prims if p.GetTypeName() in ('Material','Shader','NodeGraph')]
    switched=0
    for p in all_prims:
        if preview:
            for name in ('physics:rigidBodyEnabled','physics:collisionEnabled','physics:jointEnabled','physxArticulation:articulationEnabled'):
                attr=p.GetAttribute(name)
                if attr and attr.Get() is not False:attr.Set(False);switched+=1
    # Prevent Hydra from discovering original MDL materials; all opinions are local.
    for path in shading:
        if stage.GetPrimAtPath(path):stage.OverridePrim(path).SetActive(False)
    mat=UsdShade.Material.Define(stage,'/GuiReview/White')
    shader=UsdShade.Shader.Define(stage,'/GuiReview/White/PreviewSurface');shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.64,.68,.72))
    shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.65)
    shader.CreateOutput('surface',Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    roots=[p for p in stage.GetPseudoRoot().GetChildren() if p.GetPath()!=Sdf.Path('/GuiReview')]
    for p in roots:UsdShade.MaterialBindingAPI.Apply(p).Bind(mat,bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    try:
        from pxr import UsdLux
        UsdLux.DomeLight.Define(stage,'/GuiReview/Light').CreateIntensityAttr(900)
    except ImportError:pass
    layer.customLayerData={'reviewScope':'white material visual inspection only; physics disabled' if preview else 'white material override; source physics retained; not runtime validated',
        'source_sha256':source_hash,'source_scene':str(SOURCE)}
    layer.Export(str(dest))
    reopened=Usd.Stage.Open(str(dest));new_geometry={str(p.GetPath()):p.GetTypeName() for p in reopened.Traverse() if p.IsA(UsdGeom.Gprim)}
    assert UsdGeom.GetStageUpAxis(reopened)==UsdGeom.GetStageUpAxis(source_stage)
    assert UsdGeom.GetStageMetersPerUnit(reopened)==UsdGeom.GetStageMetersPerUnit(source_stage)
    if source_geometry!=new_geometry:raise RuntimeError('Geometry paths/types changed')
    errors=[str(e) for p in reopened.Traverse() for e in p.GetPrimIndex().localErrors]
    reports.append(dict(path=str(dest),preview_only=preview,source_geometry_count=len(source_geometry),geometry_paths_types_preserved=True,
        disabled_shading_prims=len(shading),disabled_physics_flags=switched,excluded_remote_payloads=excluded_remote,composition_errors=errors,up_axis=str(UsdGeom.GetStageUpAxis(reopened)),meters_per_unit=UsdGeom.GetStageMetersPerUnit(reopened),gui_runtime_verified=False))
if hashlib.sha256(SOURCE.read_bytes()).hexdigest()!=source_hash:raise RuntimeError('Source layer changed')
(OUT/'build.json').write_text(json.dumps(dict(source_sha256=source_hash,reports=reports),indent=2),encoding='utf-8')
print(json.dumps(reports),flush=True)
