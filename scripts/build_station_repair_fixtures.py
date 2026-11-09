"""Separate legacy instruments from the other objects sharing Stations.

All outputs are diagnostic overlays; v06 is never saved. The native mini remains
candidate-version physics, explicitly marked, and is not production acceptance.
"""
import hashlib
import json
from pathlib import Path
from validate_gpu_capacity import bare_usd

bare_usd()
from pxr import Usd,UsdGeom,UsdPhysics,Sdf,Gf

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"outputs/runtime_validation"
SOURCE=OUT/"v06_tips_00.usda"
BRANCH="/World/TaskArea/Stations"
INSTRUMENTS=("VortexGenie2","ThermalCycler","CentriMini")


def main():
    base=Usd.Stage.Open(str(SOURCE))
    children=[p.GetName() for p in base.GetPrimAtPath(BRANCH).GetChildren()]
    objects=[name for name in children if name not in INSTRUMENTS]
    rows=[]
    for name,removed in [("instruments_off",INSTRUMENTS),("station_objects_off",objects)]:
        layer=Sdf.Layer.CreateAnonymous(name+".usda")
        stage=Usd.Stage.Open(Sdf.Layer.FindOrOpen(str(SOURCE)),layer)
        stage.SetEditTarget(layer)
        for node in removed:
            stage.GetPrimAtPath(BRANCH+"/"+node).SetActive(False)
        layer.subLayerPaths=[str(SOURCE)]
        layer.customLayerData={"validationScope":"diagnostic: Robot and tips inactive; only selected Stations children removed", "removed":','.join(removed)}
        dest=OUT/("v06_"+name+".usda")
        layer.Export(str(dest))
        rows.append(dict(name=name,path=str(dest),removed=list(removed),
            remaining_stations=[p.GetName() for p in stage.GetPrimAtPath(BRANCH).GetChildren()],
            enabled_colliders=sum(p.HasAPI(UsdPhysics.CollisionAPI) and p.GetAttribute("physics:collisionEnabled").Get() is not False for p in stage.Traverse())))
    # Preserve every remaining object; add back the native mini at its actual
    # station transform rather than treating origin-only validation as integration.
    parent=OUT/"v06_instruments_off.usda"
    layer=Sdf.Layer.CreateAnonymous("native_mini.usda")
    stage=Usd.Stage.Open(Sdf.Layer.FindOrOpen(str(parent)),layer)
    stage.SetEditTarget(layer)
    old=base.GetPrimAtPath(BRANCH+"/CentriMini")
    host=UsdGeom.Xform.Define(stage,"/World/NativeMiniPilot")
    matrix=UsdGeom.XformCache().GetLocalToWorldTransform(old)
    host.AddTransformOp().Set(matrix)
    asset=ROOT/"assets/instruments6_pilot/centrifuge_tiangen_tgear_mini.usda"
    host.GetPrim().GetReferences().AddReference(str(asset))
    layer.subLayerPaths=[str(parent)]
    layer.customLayerData={"validationScope":"candidate native mini integrated at original station; Robot/tips and two other legacy instruments inactive"}
    dest=OUT/"v06_native_mini_pilot.usda"
    layer.Export(str(dest))
    rows.append(dict(name="native_mini",path=str(dest),source_asset_sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),
                     original_version_matched=False))
    native_source=dest
    for keep in (1,8,32,96):
        layer=Sdf.Layer.CreateAnonymous("native_mini_tips.usda")
        stage=Usd.Stage.Open(Sdf.Layer.FindOrOpen(str(native_source)),layer)
        stage.SetEditTarget(layer)
        pool=stage.GetPrimAtPath("/World/TaskArea/Objects/Tips/RackPool")
        paths=sorted(str(p.GetPath()) for p in pool.GetAllChildren() if p.GetName().startswith("Tip_"))
        if len(paths)!=96:
            raise ValueError("Tip coverage mismatch")
        for index,path in enumerate(paths):
            stage.GetPrimAtPath(path).SetActive(index<keep)
        layer.subLayerPaths=[str(native_source)]
        layer.customLayerData={"validationScope":"candidate mini plus selected tips; two legacy instruments and Robot inactive", "tips_kept":keep}
        output=OUT/f"v06_native_mini_tips_{keep:02d}.usda"
        layer.Export(str(output))
        rows.append(dict(name="native_mini_tips",tips_kept=keep,path=str(output),
            enabled_colliders=sum(p.HasAPI(UsdPhysics.CollisionAPI) and p.GetAttribute("physics:collisionEnabled").Get() is not False for p in stage.Traverse())))
    report=dict(instruments=list(INSTRUMENTS),other_station_objects=objects,fixtures=rows,
        corrected_interpretation="Prior all-Stations removal also removed five non-instrument objects; cannot attribute that contrast solely to instruments")
    (OUT/"station_repair_fixtures.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report),flush=True)


if __name__=="__main__":
    main()
