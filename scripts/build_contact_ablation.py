"""Export small v06 contact-load fixtures, never saving the source layer.

Only local objects are loaded (Robot is inactive). All instrument geometry is
retained; the tip-count variable is explicit. These are diagnostic subsets.
"""
import json
import hashlib
from pathlib import Path
from collections import Counter
from validate_gpu_capacity import bare_usd

bare_usd()
from pxr import Usd,Sdf,UsdPhysics,UsdGeom

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/"outputs/runtime_validation/v06_local_capacity_fixture.usda"


def main():
    source_layer = Sdf.Layer.FindOrOpen(str(SOURCE))
    base = Usd.Stage.Open(source_layer)
    tips_root = "/World/TaskArea/Objects/Tips/RackPool"
    parent = base.GetPrimAtPath(tips_root)
    if not parent:
        raise ValueError("Tip pool not found")
    paths = sorted(str(p.GetPath()) for p in parent.GetChildren() if p.GetName().startswith("Tip_"))
    if len(paths) != 96:
        raise ValueError("Expected 96 tips, got " + str(len(paths)))
    rows = []
    for keep in (0,1,8,32,96):
        session = Sdf.Layer.CreateAnonymous("ablation.usda")
        stage = Usd.Stage.Open(source_layer,session)
        stage.SetEditTarget(session)
        for path in paths[keep:]:
            stage.GetPrimAtPath(path).SetActive(False)
        collisions = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI) and p.GetAttribute("physics:collisionEnabled").Get() is not False]
        counts = Counter()
        for p in collisions:
            label = "tips" if str(p.GetPath()).startswith(tips_root+"/") else "other"
            counts[label] += 1
        session.subLayerPaths = [str(SOURCE)]
        session.customLayerData = {"validationScope":"v06 local assets; Robot inactive; tip-count ablation only", "tips_kept":keep}
        dest = ROOT/f"outputs/runtime_validation/v06_tips_{keep:02d}.usda"
        session.Export(str(dest))
        rows.append(dict(tips_kept=keep,path=str(dest),colliders=len(collisions),by_branch=dict(counts),
            collision_api_prims=sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse()),
            rigid_bodies=sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse()),
            collision_groups=sum(p.IsA(UsdPhysics.CollisionGroup) for p in stage.Traverse())))
    # Isolate the legacy station instruments after the zero-tip run still failed.
    zero_tips = ROOT/"outputs/runtime_validation/v06_tips_00.usda"
    session = Sdf.Layer.CreateAnonymous("no_stations.usda")
    stage = Usd.Stage.Open(Sdf.Layer.FindOrOpen(str(zero_tips)),session)
    stage.SetEditTarget(session)
    stations = stage.GetPrimAtPath("/World/TaskArea/Stations")
    if not stations:
        raise ValueError("Legacy station branch not found")
    stations.SetActive(False)
    session.subLayerPaths = [str(zero_tips)]
    session.customLayerData = {"validationScope":"diagnostic only: Robot, all tips and legacy Stations inactive"}
    dest = ROOT/"outputs/runtime_validation/v06_no_tips_no_stations.usda"
    session.Export(str(dest))
    rows.append(dict(tips_kept=0,stations_active=False,path=str(dest),
        colliders=sum(p.HasAPI(UsdPhysics.CollisionAPI) and p.GetAttribute("physics:collisionEnabled").Get() is not False for p in stage.Traverse()),
        collision_api_prims=sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse()),
        rigid_bodies=sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse())))
    report = dict(scope="static counts only; no conclusion about actual contacts or penetration",fixtures=rows,
        source=str(SOURCE),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest())
    (ROOT/"outputs/runtime_validation/contact_ablation_build.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report),flush=True)


if __name__ == "__main__":
    main()
