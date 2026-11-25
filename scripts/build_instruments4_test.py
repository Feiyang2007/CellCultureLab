"""
搭一个迷你验证场景：地面 + 六台自转仪器，用来确认转换器出的东西
在 PhysX 里真能被仿真、且不会一开场就炸。产出 assets/scenes/test_instruments4.usda。
"""
import os, sys

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(PROJ, "assets", "instruments4")
DST = os.path.join(PROJ, "assets", "scenes", "test_instruments4.usda")

NAMES = ["centrifuge_eppendorf_5430", "centrifuge_eppendorf_5910_ri",
         "centrifuge_tiangen_tgear_mini", "thermal_cycler_biorad_c1000",
         "thermal_mixer_eppendorf_c", "vortex_mixer_genie_2"]

stage = Usd.Stage.CreateNew(DST)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr().Set(9.81)

# 地面
g = UsdGeom.Cylinder.Define(stage, "/Ground")
g.CreateRadiusAttr(6.0)
g.CreateHeightAttr(0.04)
g.CreateAxisAttr("Z")
UsdGeom.Xformable(g.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.02))
UsdPhysics.CollisionAPI.Apply(g.GetPrim())

top = UsdGeom.Xform.Define(stage, "/Instruments")
stage.SetDefaultPrim(top.GetPrim())

y = -2.0
bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                       [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)
placed = []
for nm in NAMES:
    fp = os.path.join(OUT_DIR, nm + ".usda")
    if not os.path.isfile(fp):
        print("  缺文件", nm)
        continue
    sub = Usd.Stage.Open(fp)
    dpr = sub.GetDefaultPrim() if sub else None
    if not dpr:
        print('  跳过（没有 defaultPrim，转换失败的文件）:', nm)
        continue
    rng = bb.ComputeWorldBound(dpr).ComputeAlignedRange()
    depth = 0.5
    if not rng.IsEmpty():
        a, b = rng.GetMin(), rng.GetMax()
        depth = b[1] - a[1]
    prim = stage.DefinePrim("/Instruments/" + nm, "Xform")
    prim.GetReferences().AddReference("../instruments4/" + nm + ".usda")
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0, y + depth / 2.0, 0.0))
    y += depth + 0.35
    placed.append((nm, depth))

stage.GetRootLayer().Save()
print("放进迷你场景的仪器：")
for nm, d in placed:
    print("   %-34s 占 y 深 %.3f m" % (nm, d))
print("总长 %.2f m，产物 %s" % (y + 2.0, os.path.basename(DST)))
