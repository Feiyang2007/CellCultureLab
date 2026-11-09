"""
排版前的量尺：台面可用范围、两条臂的底座位置与可达圆、现有工位占了哪些地方。
裸 pxr 就够，不用开 Isaac。

用法： D:/IsaacSim5/env/Scripts/python.exe scripts/layout_probe.py
"""
import os, sys

ROOT = os.environ.get("ISAAC_PY_ROOT", r"D:/IsaacSim5/env/Lib/site-packages/isaacsim")
U = os.path.join(ROOT, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(ROOT, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdGeom, Sdf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(PROJ, "assets", "scenes", "biolab_main_v02.usda")
UR5E_REACH = 0.85   # 米，官方臂展，用来判断"摆下去够不够得着"

st = Usd.Stage.Open(SCENE)
xf = UsdGeom.XformCache()


def world(path):
    p = st.GetPrimAtPath(path)
    if not p:
        return None, None
    m = xf.GetLocalToWorldTransform(p)
    t = m.ExtractTranslation()
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)
    rng = bbox.ComputeWorldBound(p).ComputeAlignedRange()
    return t, (None if rng.IsEmpty() else (rng.GetMin(), rng.GetMax()))


print("=" * 88)
print("场景：", SCENE)

t, r = world("/World/Environment/Lab_021/TableTopCollision_Physics")
if r:
    (x0, y0, z0), (x1, y1, z1) = r
    print("\n【台面】x %.3f ~ %.3f (%.3f m)   y %.3f ~ %.3f (%.3f m)   台面高 z=%.3f"
          % (x0, x1, x1 - x0, y0, y1, y1 - y0, z1))

print("\n【两条臂的底座】（UR5e 臂展按 %.2f m 算可达圆）" % UR5E_REACH)
mounts = []
for name in ("Robot1Mount", "Robot2Mount"):
    t, r = world("/World/Robot/%s/ur5e" % name)
    if t is None:
        t, r = world("/World/Robot/" + name)
    mounts.append((name, tuple(t) if t else None))
    print("   %-12s 底座 (%.3f, %.3f, %.3f)   可达 x∈[%.2f,%.2f] y∈[%.2f,%.2f]"
          % (name, t[0], t[1], t[2], t[0] - UR5E_REACH, t[0] + UR5E_REACH,
             t[1] - UR5E_REACH, t[1] + UR5E_REACH))

def reachable(x, y):
    out = []
    for name, m in mounts:
        if m:
            d = ((m[0] - x) ** 2 + (m[1] - y) ** 2) ** 0.5
            if d <= UR5E_REACH:
                out.append(name)
    return out

print("\n【现有工位占用与可达性】")
for path in ["/World/TaskArea/Equipment/Centrifuge",
             "/World/TaskArea/Equipment/Pipette",
             "/World/TaskArea/Fixtures/TipRack200uL",
             "/World/TaskArea/Fixtures/PipetteRack",
             "/World/TaskArea/Fixtures/TubeRack",
             "/World/TaskArea/Fixtures/TubeRack_B",
             "/World/TaskArea/Fixtures/CentrifugeTubeRack"]:
    t, r = world(path)
    if t is None:
        continue
    who = reachable(t[0], t[1])
    size = ""
    if r:
        (x0, y0, z0), (x1, y1, z1) = r
        size = "占地 %.3f×%.3f m  顶面 z=%.3f" % (x1 - x0, y1 - y0, z1)
    print("   %-40s (%7.3f,%7.3f,%6.3f)  %-34s 可达：%s"
          % (path.split("/")[-1], t[0], t[1], t[2], size, ",".join(who) or "★都不够"))

print("\n【现有占地的 x/y 边界】（新工位要避开，且必须落在台面内）")
xs = []
ys = []
for prim in st.Traverse():
    p = str(prim.GetPath())
    if not p.startswith("/World/TaskArea"):
        continue
    if p.count("/") != 4:
        continue
    t, r = world(p)
    if r:
        (x0, y0, z0), (x1, y1, z1) = r
        xs += [x0, x1]; ys += [y0, y1]
if xs:
    print("   已用 x %.3f ~ %.3f    y %.3f ~ %.3f" % (min(xs), max(xs), min(ys), max(ys)))
