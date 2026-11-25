"""
静态查"出生即干涉"：所有工位/器械在世界坐标下的 AABB 两两重叠情况。

为什么要它：单轮 fresh 启动实测 v04/v05 跑 10 秒有 145 个刚体飞散（最大漂移 3.02 m）。
最可能的原因是物体一开始就互相插着或插进台面，PhysX 会把它们猛推开——
这与"求解器配置"无关，所以先前那些 CCD / 迭代的解释都不成立。
这个脚本只读几何、不开 Isaac。

用法：
    D:/IsaacSim5/env/Scripts/python.exe scripts/check_spawn_overlap.py [场景]
"""
import os, sys, itertools, collections

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdGeom, UsdPhysics

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
scene = os.path.abspath(sys.argv[1] if len(sys.argv) > 1
                        else os.path.join(PROJ, "assets", "scenes", "biolab_main_v05.usda"))
st = Usd.Stage.Open(scene)
bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                       [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)

def boxes_under(root_path, depth_from_root=1):
    """取某个容器下若干物体的世界 AABB。"""
    out = []
    root = st.GetPrimAtPath(root_path)
    if not root:
        return out
    for prim in root.GetChildren():
        rng = bb.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            continue
        a, b = rng.GetMin(), rng.GetMax()
        out.append((str(prim.GetPath()), (a[0], a[1], a[2], b[0], b[1], b[2])))
    return out

items = []
for cont in ("/World/TaskArea/Stations", "/World/TaskArea/Equipment",
             "/World/TaskArea/Fixtures", "/World/TaskArea/Objects"):
    got = boxes_under(cont)
    print("%-34s %d 个子物体" % (cont, len(got)))
    items += got

# 台面（用来查有没有插进台子）
top = None
for p in ("/World/Environment/Lab_021/TableTopCollision_Physics",):
    prim = st.GetPrimAtPath(p)
    if prim:
        rng = bb.ComputeWorldBound(prim).ComputeAlignedRange()
        if not rng.IsEmpty():
            b = rng.GetMax()
            top = b[2]
print("台面顶面 z =", top)

def overlap2d(a, b):
    ox = min(a[3], b[3]) - max(a[0], b[0])
    oy = min(a[4], b[4]) - max(a[1], b[1])
    oz = min(a[5], b[5]) - max(a[2], b[2])
    if ox > 0 and oy > 0 and oz > 0:
        return ox * oy * oz
    return 0.0

pairs = []
for (n1, b1), (n2, b2) in itertools.combinations(items, 2):
    v = overlap2d(b1, b2)
    if v > 1e-6:
        pairs.append((v, n1, n2))
pairs.sort(reverse=True)

print()
print("=" * 92)
print("物体两两体积重叠：共 %d 对" % len(pairs))
for v, n1, n2 in pairs[:20]:
    print("   重叠 %.2f cm^3  %s  <->  %s" % (v * 1e6, n1.split("/")[-1], n2.split("/")[-1]))

# 插进台面的（最低点低于台面顶）
if top:
    sunks = [(n, b[2] - top) for n, b in items if b[2] < top - 1e-4]
    sunks.sort(key=lambda x: x[1])
    print()
    print("最低点低于台面顶面的物体：%d 个" % len(sunks))
    for n, dz in sunks[:15]:
        print("   %+.4f m  %s" % (dz, n))
