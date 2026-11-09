"""
修工位摆放：接地到台面 + 硬体之间零重叠，产出 v06（Export，不覆盖 v05）。

两个必须解决的实测缺陷：
  1) 4 个工位最低点在台面之下（最深 -18.7 mm）——PhysX 会把埋进碰撞体的物体猛顶出来，
     这就是"跑 10 秒 145 个刚体飞散、最大漂移 3.02 m"的来源；
  2) CentriMini 与 TubeRack 体积重叠 15 cm³——被压在了现有管架上。

规则：
  - 接地：让每个工位的**世界包围盒最低点**落在台面顶 + 0.5 mm，而不是假设资产原点在底面；
  - 硬体之间（Stations / Equipment / Fixtures）AABB 不得有体积重叠；
    Objects（架子里的耗材）与 Fixtures 的重叠是设计内的，不管；
  - 挪不动时沿 y 逐步外推，仍须留在台面与双臂可达带内。
"""
import os, sys, itertools, collections

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdGeom, Gf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(PROJ, "assets", "scenes", "biolab_main_v05.usda")
DST = os.path.join(PROJ, "assets", "scenes", "biolab_main_v06.usda")

TABLE_TOP = 0.8716955
TABLE = (-1.999, -0.441, -3.457, 1.027)
REACH = (-1.999, -1.090, -2.830, -0.350)
GAP = 0.0005
HARD_CONTAINERS = ("/World/TaskArea/Stations", "/World/TaskArea/Equipment",
                   "/World/TaskArea/Fixtures")

st = Usd.Stage.Open(SRC)
bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                       [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)


def refresh_cache():
    """改过 xform 之后包围盒缓存会过期，重建一次，别拿旧值判断重叠。"""
    global bb
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)
    UsdGeom.XformCache().GetLocalToWorldTransform(  # noqa  仅为触发一次求值
        st.GetPseudoRoot()) if False else None


def world_box(prim):
    rng = bb.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        return None
    a, b = rng.GetMin(), rng.GetMax()
    return (a[0], a[1], a[2], b[0], b[1], b[2])


def translate_op(prim):
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op
    return xf.AddTranslateOp()


def hard_items():
    out = []
    for cont in HARD_CONTAINERS:
        root = st.GetPrimAtPath(cont)
        if not root:
            continue
        for child in root.GetChildren():
            b = world_box(child)
            if b:
                out.append((child, b))
    return out


def vol_overlap(a, b):
    ox = min(a[3], b[3]) - max(a[0], b[0])
    oy = min(a[4], b[4]) - max(a[1], b[1])
    oz = min(a[5], b[5]) - max(a[2], b[2])
    return (ox * oy * oz) if (ox > 0 and oy > 0 and oz > 0) else 0.0


# ---------- 第一轮：接地 ----------
fixed_sink = 0
for cont in HARD_CONTAINERS:
    root = st.GetPrimAtPath(cont)
    if not root:
        continue
    for child in root.GetChildren():
        b = world_box(child)
        if not b:
            continue
        dz = (TABLE_TOP + GAP) - b[2]
        if dz < -1e-4:
            op = translate_op(child)
            cur = op.Get() or Gf.Vec3d(0, 0, 0)
            op.Set(Gf.Vec3d(cur[0], cur[1], cur[2] + dz))
            fixed_sink += 1
            print("接地 %-38s 上提 %.1f mm" % (child.GetName(), -dz * 1000))
print("接地修正：%d 个\n" % fixed_sink)

# ---------- 第二轮：把硬体之间的重叠推开 ----------
for it in range(1, 41):
    items = hard_items()
    worst = None
    for (p1, b1), (p2, b2) in itertools.combinations(items, 2):
        v = vol_overlap(b1, b2)
        if v > 1e-7 and (worst is None or v > worst[0]):
            worst = (v, p1, p2, b1, b2)
    if worst is None:
        print("第 %d 轮：硬体之间已无重叠" % it)
        break
    v, p1, p2, b1, b2 = worst
    # 只挪 Stations 里的新工位；原有 Equipment/Fixtures 不动（那是师兄摆好的）
    tgt, other = (p1, p2) if "Stations" in str(p1.GetPath()) else (p2, p1)
    if "Stations" not in str(tgt.GetPath()):
        print("  ! 重叠双方都不是新工位，需要人工判断：", p1.GetPath(), p2.GetPath())
        break
    ob = world_box(other)
    tb = world_box(tgt)
    op = translate_op(tgt)
    cur = op.Get()
    # 沿 y 远离对方，步长 2 cm；不行就沿 x
    moved = False
    for axis, sign in ((1, 1), (1, -1), (0, 1), (0, -1)):
        for step in range(1, 40):
            trial = list(cur)
            trial[axis] += sign * 0.02 * step
            # 检查仍在台面与可达带内
            dx = trial[0] - cur[0]
            dy = trial[1] - cur[1]
            nx0, nx1 = tb[0] + dx, tb[3] + dx
            ny0, ny1 = tb[1] + dy, tb[4] + dy
            cx, cy = (nx0 + nx1) / 2, (ny0 + ny1) / 2
            if not (TABLE[0] <= nx0 and nx1 <= TABLE[1] and TABLE[2] <= ny0 and ny1 <= TABLE[3]):
                break
            if not (REACH[0] <= cx and cx <= REACH[1] and REACH[2] <= cy and cy <= REACH[3]):
                continue
            op.Set(Gf.Vec3d(trial[0], trial[1], cur[2]))
            refresh_cache()
            refresh_cache()
            nb = world_box(tgt)
            if vol_overlap(nb, ob) <= 1e-7:
                moved = True
                print("推开 %-24s 与 %-22s 重叠 %.1f cm^3 -> 位移 (%+.0f, %+.0f) mm"
                      % (tgt.GetName(), other.GetName(), v * 1e6,
                         (trial[0] - cur[0]) * 1000, (trial[1] - cur[1]) * 1000))
                break
        if moved:
            break
    if not moved:
        print("  ! 推不开：", tgt.GetPath(), other.GetPath(), "重叠 %.1f cm^3" % (v * 1e6))
        break

# ---------- 结果复核 ----------
print()
items = hard_items()
bad_pairs = [(p1.GetPath(), p2.GetPath(), vol_overlap(b1, b2))
             for (p1, b1), (p2, b2) in itertools.combinations(items, 2)
             if vol_overlap(b1, b2) > 1e-7]
buried = [(str(p.GetPath()), b[2] - TABLE_TOP) for p, b in items if b[2] < TABLE_TOP - 1e-4]
print("复核：硬体重叠 %d 对；陷进台面 %d 个" % (len(bad_pairs), len(buried)))
for a, b, v in bad_pairs[:6]:
    print("   重叠 %.1f cm^3  %s <-> %s" % (v * 1e6, a, b))
for n, dz in buried[:6]:
    print("   低于台面 %+.4f m  %s" % (dz, n))

ok = st.GetRootLayer().Export(DST)
print("\n导出 v06：", "成功" if ok else "失败", os.path.basename(DST),
      os.path.getsize(DST), "字节")
