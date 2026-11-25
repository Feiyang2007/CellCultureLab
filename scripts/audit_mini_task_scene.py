"""Bare-pxr geometry audit of the mini pilot asset and the two candidate task scenes.

Answers the second-round supplement 四.4 with numbers instead of assumptions:
  * where the mini actually sits in world Z (so the ground plane can be placed with
    a measured clearance instead of an authored "top face = 0" comment),
  * which rigid bodies / joints / articulation roots exist after composition,
  * whether a scene's referenced asset resolves and what composition errors appear,
  * what physics scene configuration the exported layer actually carries.

No SimulationApp, no physics stepping: this is a static read of composed USD.
"""
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# 本机构的 pxr 只有在 kit 引导之后才可导入（裸解释器实测 ModuleNotFoundError），
# 所以这里仍要走一次 headless SimulationApp；本脚本不启动物理，只做静态组合读取。
os.environ["TEMP"] = os.environ["TMP"] = str(ROOT / "outputs" / "runtime_validation" / "_audit_tmp")
os.makedirs(os.environ["TEMP"], exist_ok=True)
from isaacsim import SimulationApp
_app = SimulationApp({"headless": True,
                      "extra_args": ["--portable-root", str(Path.home() / "CellCultureLabRuntime"),
                                     "--/app/settings/persistent=false",
                                     "--/renderer/multiGpu/enabled=false"]})
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Vt
PILOT = ROOT / "assets/instruments6_pilot/centrifuge_tiangen_tgear_mini.usda"
SCENES = ["assets/scenes/test_mini_articulation.usda", "assets/scenes/mini_lid_rotor_task.usda"]

out = {"pilot": {}, "scenes": []}


def composition_errors(stage):
    """组合错误取自真实 prim index 局部错误；取不到就写 missing，绝不填 0。"""
    # 本安装的 localErrors 挂在 **prim** 上（Usd.Stage 没有 GetPrimIndex，实测 AttributeError），
    # 沿用项目里 check_v5_independent.py / build_c1000_lock_pilot.py 已经用过的写法逐 prim 收。
    if not any(hasattr(p, "GetPrimIndex") for p in stage.Traverse()):
        return {"status": "missing_api_prim_GetPrimIndex"}
    try:
        errs = [str(e) for p in stage.Traverse() for e in p.GetPrimIndex().localErrors]
    except Exception as exc:
        return {"status": "error", "detail": "%s: %s" % (type(exc).__name__, exc)}
    hard = [e for e in errs if ("Could not open asset" in e or "unresolved" in e.lower()
                                or "error" in e.split("|")[0].lower())]
    return {"status": "ok", "count": len(errs), "hard_count": len(hard), "hard": hard[:8],
            "sample": errs[:4]}


def bbox_of(stage, prefix_filter=None):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy], False)
    mins = [1e9] * 3
    maxs = [-1e9] * 3
    n = 0
    for prim in stage.Traverse():
        if not prim.IsActive():
            continue
        typename = str(prim.GetTypeName())
        if typename not in ("Mesh", "Cube", "Cylinder", "Sphere", "Capsule", "Cone", "BasisCurves", "Points"):
            continue
        if prefix_filter and not str(prim.GetPath()).startswith(prefix_filter):
            continue
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if not rng.IsEmpty():
            n += 1
            lo, hi = rng.GetMin(), rng.GetMax()
            for i in range(3):
                mins[i] = min(mins[i], lo[i])
                maxs[i] = max(maxs[i], hi[i])
    return n, ([round(v, 6) for v in mins] if n else None), ([round(v, 6) for v in maxs] if n else None)


def audit(stage, label, root_path=""):
    bodies, joints, arts, cols, drives = [], [], [], 0, 0
    for prim in stage.Traverse():
        if UsdPhysics.RigidBodyAPI(prim):
            m = UsdPhysics.MassAPI(prim)
            bodies.append({"path": str(prim.GetPath()),
                           "mass": m.GetMassAttr().Get() if m.GetMassAttr().HasValue() else None})
        if prim.IsA(UsdPhysics.Joint):
            rec = {"path": str(prim.GetPath()), "type": str(prim.GetTypeName())}
            for a in ("physics:lowerLimit", "physics:upperLimit", "physics:axis",
                      "drive:angular:physics:stiffness", "drive:angular:physics:damping",
                      "drive:angular:physics:type", "drive:angular:physics:targetPosition"):
                attr = prim.GetAttribute(a)
                if attr and attr.HasValue():
                    rec[a.split(":")[-1]] = attr.Get()
            for r in ("physics:body0", "physics:body1"):
                rel = prim.GetRelationship(r)
                rec[r] = [str(t) for t in rel.GetTargets()] if rel and rel.GetTargets() else None
            for a in ("physics:localPos0", "physics:localPos1"):
                attr = prim.GetAttribute(a)
                if attr and attr.Get() is not None:
                    rec[a] = list(attr.Get())
            drives += sum(1 for a in prim.GetAttributes() if a.GetName().startswith("drive:"))
            joints.append(rec)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            arts.append(str(prim.GetPath()))
        if UsdPhysics.CollisionAPI(prim) and str(prim.GetTypeName()) in ("Mesh", "Cube", "Cylinder", "Sphere", "Capsule", "Cone"):
            cols += 1
    n, mn, mx = bbox_of(stage, root_path or None)
    return {"label": label,
            "default_prim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
            "rigid_bodies": len(bodies), "body_sample": bodies[:6],
            "joints": joints, "articulation_roots": arts,
            "colliders": cols, "drive_attrs_on_joints": drives,
            "geom_count": n, "world_min": mn, "world_max": mx,
            "has_physics_scene": [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)],
            "composition_errors": composition_errors(stage)}


pilot_stage = Usd.Stage.Open(str(PILOT))
try:
    out["pilot"] = audit(pilot_stage, "pilot asset")
    out["pilot"]["path"] = str(PILOT.relative_to(ROOT))
except Exception as exc:                                  # 记录局部失败，不让整个体检半途而废
    out["pilot"] = {"error": "%s: %s" % (type(exc).__name__, exc)}

for s in SCENES:
    p = ROOT / s
    if not p.exists():
        out["scenes"].append({"scene": s, "exists": False})
        continue
    st = Usd.Stage.Open(str(p))
    rec = {"scene": s, "exists": True}
    try:
        rec["unresolved"] = ("missing_api_TraverseUnresolvedReferencePaths"
                             if not hasattr(Usd, "TraverseUnresolvedReferencePaths")
                             else [str(x.GetPath()) for x in
                                  Usd.TraverseUnresolvedReferencePaths(st)])
        rec.update(audit(st, s))
        for prim in st.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                rec["physics_scene_prim"] = {"path": str(prim.GetPath()),
                                             "api": [str(x.GetTypeName()) for x in prim.GetAPIs()],
                                             "attrs": {a.GetName(): a.Get() for a in prim.GetAttributes() if a.HasValue()}}
    except Exception as exc:
        rec["error"] = "%s: %s" % (type(exc).__name__, exc)
    out["scenes"].append(rec)

# ground plane facts of the v1 task scene, computed rather than assumed
task = ROOT / "assets/scenes/mini_lid_rotor_task.usda"
if task.exists():
    st = Usd.Stage.Open(str(task))
    g = {"exists": True}
    try:
        ground = st.GetPrimAtPath("/World/Ground")
        g["ground_present"] = bool(ground)
        if ground:
            xf = UsdGeom.Xformable(ground)
            m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            rng = cache.ComputeWorldBound(ground).ComputeAlignedRange()
            g["transform_translation"] = [m[3][0], m[3][1], m[3][2]]
            g["ground_world_min"] = list(rng.GetMin())
            g["ground_world_max"] = list(rng.GetMax())
        mini = st.GetPrimAtPath("/World/Mini")
        g["mini_present"] = bool(mini)
        if mini:
            n, mn, mx = bbox_of(st, "/World/Mini")
            g["mini_geom_count"] = n
            g["mini_world_min"] = mn
            g["mini_world_max"] = mx
            if mn and g.get("ground_world_max"):
                g["ground_top_minus_mini_bottom_m"] = round(g["ground_world_max"][2] - mn[2], 6)
                g["initial_overlap"] = bool(g["ground_world_max"][2] - mn[2] > 0)
    except Exception as exc:
        g["error"] = "%s: %s" % (type(exc).__name__, exc)
    out["v1_ground_check"] = g

OUT = ROOT / "outputs" / "runtime_validation" / "mini_task_scene_audit.json"
OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(json.dumps(out, indent=1, ensure_ascii=False, default=str), encoding="utf-8")
print("AUDIT_WRITTEN", OUT, flush=True)
_app.close()
