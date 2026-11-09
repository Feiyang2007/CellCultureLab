"""Build an isolated, portable asset-validation fixture; no task demonstration."""
from pathlib import Path
import json
from pxr import Usd,UsdGeom,UsdPhysics,Gf

def build():
    P=Path(__file__).resolve().parent.parent
    source=Usd.Stage.Open(str(P/'assets/scenes/biolab_main_v04.usda'))
    path=P/'assets/interactive/validation/rack_fixture.usda'
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():path.unlink()
    st=Usd.Stage.CreateNew(str(path));root=UsdGeom.Xform.Define(st,'/RackTest');st.SetDefaultPrim(root.GetPrim());UsdGeom.SetStageMetersPerUnit(st,1);UsdGeom.SetStageUpAxis(st,'Z')
    cache=UsdGeom.XformCache();initial={}
    for i in range(8):
        for j in range(1,13):
            name=f'Tip_{chr(65+i)}{j:02}'
            original=source.GetPrimAtPath('/World/TaskArea/Objects/Tips/RackPool/'+name)
            target=UsdGeom.Xform.Define(st,'/RackTest/Tips/'+name)
            target.GetPrim().GetReferences().AddReference('../tip_200ul.usda')
            matrix=cache.GetLocalToWorldTransform(original);target.MakeMatrixXform().Set(matrix)
            initial[name]=list(matrix.ExtractTranslation())
    rack=UsdGeom.Xform.Define(st,'/RackTest/Rack')
    rack.GetPrim().GetReferences().AddReference('../../precision_tasks/pipette_tip_200ul/tiprack_200ul.usda')
    rack.MakeMatrixXform().Set(cache.GetLocalToWorldTransform(source.GetPrimAtPath('/World/TaskArea/Fixtures/TipRack200uL')))
    floor=UsdGeom.Cube.Define(st,'/RackTest/TableSupport');floor.CreateSizeAttr(1);floor.AddTranslateOp().Set(Gf.Vec3d(-1.5,-1.2,0.867));floor.AddScaleOp().Set(Gf.Vec3d(1,1,0.01));UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    st.GetRootLayer().Save();path.with_suffix('.json').write_text(json.dumps(initial),encoding='utf-8');print('Isolated 96-tip rack fixture created')
    return path

if __name__=='__main__':build()
