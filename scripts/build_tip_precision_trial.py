"""Build an OFFLINE graded-collision experiment; no runtime switching is claimed.

The selected tip keeps all original precision geometry. Other tips retain hollow
socket sectors, with each group of three replaced by its convex hull. This adds
solid volume and changes inferred inertia; it is a resource experiment ONLY.
Never use this fixture for task data or switch it while simulation is running.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from validate_gpu_capacity import bare_usd

bare_usd()
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Vt, Gf

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"outputs/runtime_validation"
POOL="/World/TaskArea/Objects/Tips/RackPool"


def build(selected="Tip_A01"):
    source=OUT/"v06_native_mini_tips_96.usda"
    layer=Sdf.Layer.CreateAnonymous("graded_precision_trial.usda")
    stage=Usd.Stage.Open(Sdf.Layer.FindOrOpen(str(source)),layer)
    stage.SetEditTarget(layer)
    tips=list(stage.GetPrimAtPath(POOL).GetChildren())
    if len(tips)!=96 or selected not in [p.GetName() for p in tips]:
        raise ValueError("Expected 96 tips and an existing selected tip")
    rows=[]
    for tip in tips:
        original=[p for p in Usd.PrimRange(tip) if p.HasAPI(UsdPhysics.CollisionAPI) and p.GetAttribute("physics:collisionEnabled").Get() is not False]
        if len(original)!=37:
            raise ValueError("Precision collider count changed: "+str(tip.GetPath()))
        if tip.GetName()==selected:
            continue
        for group in ("SocketUpper","SocketNeck"):
            parent=str(tip.GetPath())+"/CollisionPrecision/"+group
            for first in range(0,18,3):
                members=[stage.GetPrimAtPath(parent+f"/Sector_{i:02d}") for i in range(first,first+3)]
                points=[]
                source_volume=0.
                for member in members:
                    # Current sectors have no transform; fail rather than silently
                    # fuse coordinates from different local frames.
                    if UsdGeom.Xformable(member).GetOrderedXformOps():
                        raise ValueError("Unexpected sector transform")
                    coords=np.asarray(UsdGeom.Mesh(member).GetPointsAttr().Get(),dtype=float)
                    source_volume+=ConvexHull(coords).volume
                    points.extend(coords.tolist())
                    UsdPhysics.CollisionAPI(member).CreateCollisionEnabledAttr(False)
                coords=np.asarray(points)
                hull=ConvexHull(coords)
                mesh=UsdGeom.Mesh.Define(stage,parent+f"/TrialCoarse_{first//3:02d}")
                mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*v) for v in coords]))
                mesh.CreateFaceVertexCountsAttr([3]*len(hull.simplices))
                mesh.CreateFaceVertexIndicesAttr(hull.simplices.ravel().tolist())
                mesh.CreateVisibilityAttr("invisible")
                UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
                UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
                member=members[0]
                for attrname in ("physxCollision:contactOffset","physxCollision:restOffset"):
                    attr=member.GetAttribute(attrname)
                    mesh.GetPrim().CreateAttribute(attrname,attr.GetTypeName(),custom=attr.IsCustom()).Set(attr.Get())
                mesh.GetPrim().SetMetadata("apiSchemas",Sdf.TokenListOp.CreateExplicit(["PhysicsCollisionAPI","PhysicsMeshCollisionAPI","PhysxCollisionAPI"]))
                rows.append(dict(tip=tip.GetName(),group=group,first_sector=first,
                    source_convex_volume_m3=source_volume,coarse_volume_m3=float(hull.volume),
                    added_solid_fraction=float(hull.volume/source_volume-1)))
    layer.subLayerPaths=[str(source)]
    layer.customLayerData={"validationScope":"RESOURCE EXPERIMENT ONLY: one precise tip, 95 coarse hollow tips; Robot and two legacy instruments inactive; no insertion/ejection or transition acceptance",
        "selected_precision_tip":selected,"task_data_approved":False,"runtime_switching_validated":False,
        "inferred_inertia_may_change":True}
    dest=OUT/"v06_native_mini_96tips_graded_trial.usda"
    layer.Export(str(dest))
    reopened=Usd.Stage.Open(str(dest))
    counts={p.GetName():sum(q.HasAPI(UsdPhysics.CollisionAPI) and q.GetAttribute("physics:collisionEnabled").Get() is not False for q in Usd.PrimRange(p)) for p in reopened.GetPrimAtPath(POOL).GetChildren()}
    assert counts[selected]==37 and all(n==13 for k,n in counts.items() if k!=selected)
    # Check authored geometry and physical settings of the precise tip survived.
    before=Usd.Stage.Open(str(source))
    for p in Usd.PrimRange(before.GetPrimAtPath(POOL+'/'+selected)):
        q=reopened.GetPrimAtPath(p.GetPath())
        for attr in p.GetAttributes():
            if attr.Get()!=q.GetAttribute(attr.GetName()).Get():
                raise ValueError("Selected precision tip was changed: "+str(attr.GetPath()))
    report=dict(scene=str(dest),selected_precision_tip=selected,counts=counts,total_tip_colliders=sum(counts.values()),
        original_tip_colliders=96*37,selected_tip_attributes_unchanged=True,
        resource_experiment_only=True,task_data_approved=False,runtime_switching_validated=False,
        inferred_inertia_may_change=True,sector_volume_changes=rows,
        source_asset_sha256=hashlib.sha256((ROOT/'assets/interactive/tip_200ul.usda').read_bytes()).hexdigest())
    (OUT/"tip_precision_trial_build.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps({k:v for k,v in report.items() if k not in ('counts','sector_volume_changes')}))


if __name__=="__main__":build()
