import json
from pathlib import Path
from collections import Counter
from validate_gpu_capacity import bare_usd
bare_usd()
from pxr import Usd, UsdGeom, UsdShade
ROOT=Path(__file__).resolve().parents[1]
result={}
for name,path in [('original',ROOT/'assets/scenes/biolab_main_v06.usda'),('white',ROOT/'outputs/gui_review/biolab_v06_white_physics.usda')]:
    stage=Usd.Stage.Open(str(path))
    result[name]={'up_axis':str(UsdGeom.GetStageUpAxis(stage)),'meters_per_unit':UsdGeom.GetStageMetersPerUnit(stage),'authored_up_axis':stage.GetRootLayer().upAxis if hasattr(stage.GetRootLayer(),'upAxis') else str(stage.GetMetadata('upAxis'))}
    if name!='original':continue
    shaders=[]
    for p in stage.Traverse():
        if not p.IsA(UsdShade.Shader):continue
        s=UsdShade.Shader(p)
        shaders.append({'path':str(p.GetPath()),'id':str(s.GetIdAttr().Get()),'source':str(s.GetSourceAsset('mdl')),'inputs':{i.GetBaseName():str(i.Get()) for i in s.GetInputs()}})
    result['shaders']=shaders
    mats=[]
    for p in stage.Traverse():
        if p.IsA(UsdShade.Material):
            mat=UsdShade.Material(p)
            mats.append({'path':str(p.GetPath()),'outputs':{o.GetFullName():[str(c) for c in o.GetAttr().GetConnections()] for o in mat.GetOutputs()},'children':[(c.GetName(),str(UsdShade.Shader(c).GetIdAttr().Get())) for c in p.GetChildren() if c.IsA(UsdShade.Shader)]})
    result['materials']=mats
    counts=Counter()
    for p in stage.Traverse():
        if p.IsA(UsdGeom.Gprim):
            m,_=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()
            counts[str(m.GetPath()) if m else 'unbound']+=1
    result['bound_material_counts']=dict(counts)
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy])
    result['bounds']={str(p.GetPath()):str(cache.ComputeWorldBound(p).ComputeAlignedRange()) for p in stage.GetPrimAtPath('/World').GetChildren() if p.IsA(UsdGeom.Xformable)}
(ROOT/'outputs/gui_review/material_audit.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k in ('original','white','bounds')},indent=2))
print('shader_count',len(result['shaders']))
print('shader_ids',Counter(s['id']+' '+s['source'] for s in shaders).most_common(15))
print('input_names',Counter(i for s in shaders for i in s['inputs']).most_common(30))
print(json.dumps(shaders[:4],indent=2))
print('materials',json.dumps(result['materials'][:4],indent=2))
print('top_bindings',counts.most_common(15))
