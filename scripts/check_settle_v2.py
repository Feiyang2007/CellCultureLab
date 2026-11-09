"""
可信版稳定性检查（第二版）。

第一版错在哪：用 stage.TraverseAll() 收集刚体 —— 它会把【实例原型内部】的 prim 也算进来，
这些 prim 的世界变换没有物理意义，于是读出 −5964 m、8153 km 这种不可能的"位移"，
把我前几轮的结论全带偏了。

这一版：
  · 只收 Usd.Traverse()（不展开实例内部）且确实挂了 RigidBodyAPI 的 prim；
  · 排除祖先链上已有刚体的（PhysX 里子级不会各自成体，避免把层级里套出来的假独立体算进来）；
  · 所有数值做有限性与量级校验（|Δ| > 5 m 直接判为读数无效，单列而不是当成真实飞散）；
  · 同时输出前 10 大位移体的起止坐标，便于人工核对。

用法：
    D:/IsaacSim5/env/Scripts/python.exe scripts/check_settle_v2.py [场景] [--frames 600]
"""
import os, sys, math, time

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd, omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
args = [a for a in sys.argv[1:] if not a.startswith("--")]
scene = os.path.abspath(args[0] if args else os.path.join(
    PROJ, "assets", "scenes", "biolab_main_v04.usda"))
frames = 600
if "--frames" in sys.argv:
    frames = int(sys.argv[sys.argv.index("--frames") + 1])

ctx = omni.usd.get_context()
if not ctx.open_stage(scene):
    print("打不开：", scene); app.close(); sys.exit(1)
st = ctx.get_stage()

# 只走实例外的 prim；再剔除"祖先已有刚体"的，得到真正的独立刚体根
bodies = []
for prim in st.Traverse():
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        continue
    a = prim.GetParent()
    nested = False
    while a:
        if a.HasAPI(UsdPhysics.RigidBodyAPI):
            nested = True
            break
        a = a.GetParent()
    if not nested:
        bodies.append(prim)
inst = sum(1 for p in st.Traverse() if p.IsInstance())
print("=" * 90)
print("场景：", os.path.basename(scene))
print("独立刚体根 %d 个（实例数 %d；Traverse 不展开实例内部）" % (len(bodies), inst))


def snap(bb):
    out = {}
    for p in bodies:
        rng = None
        m = bb.GetLocalToWorldTransform(p)
        if not m:
            continue
        t = m.ExtractTranslation()
        vals = [t[0], t[1], t[2]]
        if any(not math.isfinite(v) for v in vals):
            continue                      # 读到 NaN/inf 的不算数
        out[str(p.GetPath())] = tuple(vals)
    return out


tl = omni.timeline.get_timeline_interface()
tl.stop(); tl.play()
bb = UsdGeom.XformCache()
a = snap(bb)
t_start = time.perf_counter()
for _ in range(frames):
    app.update()
    bb = UsdGeom.XformCache()          # 每帧重建，避免缓存过期读到旧值
b = snap(bb)
el = time.perf_counter() - t_start

moved, absurd = [], []
for k, v in b.items():
    if k not in a:
        continue
    dz = v[2] - a[k][2]
    dh = math.dist(v[:2], a[k][:2])
    d3 = math.dist(v, a[k])
    if d3 > 5.0:
        absurd.append((d3, k, a[k], v))
        continue
    if d3 > 0.001:
        moved.append((d3, dz, dh, k, a[k], v))

moved.sort(reverse=True)
print("仿真 %d 帧耗时 %.1f s（%.1f ms/帧，含取变换开销）" % (frames, el, el / frames * 1000))
print("有效位移刚体：%d 个   读数不合理(>5m)被剔除：%d 个" % (len(moved), len(absurd)))
print()
print("位移最大的 10 个：")
for d3, dz, dh, k, pa, pb in moved[:10]:
    print("   %-9.4f m (Δz %+8.4f 水平 %7.4f)  %s" % (d3, dz, dh, k.replace("/World/TaskArea/", "")))
    print("              起 (%+.4f,%+.4f,%+.4f)  止 (%+.4f,%+.4f,%+.4f)" % (pa + pb))
if absurd:
    print("\n被剔除的可疑读数（多半仍是实例/失效 prim，需要单独看）：")
    for d3, k, pa, pb in absurd[:6]:
        print("   %12.1f m  %s" % (d3, k))

sinks = [m for m in moved if m[1] < -0.005]
drift = [m for m in moved if m[2] > 0.010]
print()
print("判读：沉降>5mm 的 %d 个；水平漂移>10mm 的 %d 个" % (len(sinks), len(drift)))
print("  → 沉降与漂移都为 0 才可判定场景在物理上是稳的；结论只覆盖这 %d 个独立刚体根" % len(bodies))
app.close()
