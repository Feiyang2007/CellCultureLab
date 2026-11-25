"""
修工位摆放（第二版），产出 v06。第一版把接地符号写反、且把"器械挂在架子里"当成缺陷，作废。

规则：
  1. 只处理【最低点低于台面】的物体：抬到 台面顶 + 0.5 mm。**绝不向下压任何东西**——
     悬在枪架里的移液器、放在架子上的孔板本来就高于台面，压下去反而制造干涉。
  2. 避让只对"我新增的工位"（Stations/*）生效；Equipment / Fixtures / Objects 之间原有的
     包围盒重叠是"耗材插在架子里"的设计内关系，不动也不报错。
  3. 避让后仍须落在台面与双臂可达带内；塞不下就明确报失败，不硬塞。
"""
import os, sys, itertools

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
CONTAINERS = ("/World/TaskArea/Stations", "/World/TaskArea/Equipment",
              "/World/TaskArea/Fixtures", "/World/TaskArea/Objects")

st = Usd.Stage.Open(SRC)


def cache():
    return UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)


def world_box(prim, bb):
    rng = bb.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        return None
    a, b = rng.GetMin(), rng.GetMax()
    return (a[0], a[1], a[2], b[0], b[1], b[2])


def all_items(bb):
    out = []
    for cont in CONTAINERS:
        root = st.GetPrimAtPath(cont)
        if not root:
            continue
        for child in root.GetChildren():
            b = world_box(child, bb)
            if b:
                out.append((child, b))
    return out


def translate_op(prim):
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op
    return xf.AddTranslateOp()


def vol_overlap(a, b):
    ox = min(a[3], b[3]) - max(a[0], b[0])
    oy = min(a[4], b[4]) - max(a[1], b[1])
    oz = min(a[5], b[5]) - max(a[2], b[2])
    return (ox * oy * oz) if (ox > 0 and oy > 0 and oz > 0) else 0.0


# ---------------- 1. 只把低于台面的抬上来 ----------------
raised = 0
bb = cache()
for cont in CONTAINERS:
    root = st.GetPrimAtPath(cont)
    if not root:
        continue
    for child in root.GetChildren():
        b = world_box(child, bb)
        if not b:
            continue
        pen = TABLE_TOP - b[2]
        if pen > 1e-4:
            op = translate_op(child)
            cur = op.Get() or Gf.Vec3d(0, 0, 0)
            op.Set(Gf.Vec3d(cur[0], cur[1], cur[2] + pen + GAP))
            raised += 1
            print("抬升 %-42s 穿透 %.1f mm -> 上移 %.1f mm"
                  % (str(child.GetPath()).replace("/World/TaskArea/", ""),
                     pen * 1000, (pen + GAP) * 1000))
print("抬升 %d 个\n" % raised)

# ---------------- 2. 只挪新工位，消除体积重叠 ----------------
for it in range(1, 61):
    bb = cache()
    items = all_items(bb)
    worst = None
    for (p1, b1), (p2, b2) in itertools.combinations(items, 2):
        s1 = "Stations" in str(p1.GetPath())
        s2 = "Stations" in str(p2.GetPath())
        if not (s1 or s2):
            continue
        v = vol_overlap(b1, b2)
        if v > 1e-7 and (worst is None or v > worst[0]):
            worst = (v, (p1, b1) if s1 else (p2, b2), (p2, b2) if s1 else (p1, b1))
    if worst is None:
        print("第 %d 轮：新工位与其它物体之间已无体积重叠" % it)
        break
    v, (tgt, tb), (oth, ob) = worst
    op = translate_op(tgt)
    cur = op.Get()
    moved = False
    for axis, sign in ((1, 1), (1, -1), (0, 1), (0, -1)):
        for step in range(1, 60):
            trial = list(cur)
            trial[axis] += sign * 0.02 * step
            dx, dy = trial[0] - cur[0], trial[1] - cur[1]
            nx0, nx1 = tb[0] + dx, tb[3] + dx
            ny0, ny1 = tb[1] + dy, tb[4] + dy
            cx, cy = (nx0 + nx1) / 2, (ny0 + ny1) / 2
            if not (TABLE[0] <= nx0 and nx1 <= TABLE[1]
                    and TABLE[2] <= ny0 and ny1 <= TABLE[3]):
                break
            if not (REACH[0] <= cx and cx <= REACH[1]
                    and REACH[2] <= cy and cy <= REACH[3]):
                continue
            op.Set(Gf.Vec3d(trial[0], trial[1], cur[2]))
            nb = world_box(tgt, cache())
            if nb and vol_overlap(nb, ob) <= 1e-7:
                moved = True
                print("避让 %-22s 让开 %-22s 原重叠 %.1f cm^3 -> 位移 (%+.0f, %+.0f) mm"
                      % (tgt.GetName(), oth.GetName(), v * 1e6, dx * 1000, dy * 1000))
                break
        if moved:
            break
    if not moved:
        print("  ! 避让失败（台面/可达带内已无空位）：", tgt.GetPath(), "<->", oth.GetPath())
        break

# ---------------- 3. 复核 ----------------
bb = cache()
items = all_items(bb)
buried = [(str(p.GetPath()), TABLE_TOP - b[2]) for p, b in items if b[2] < TABLE_TOP - 1e-4]
hard_bad = [(str(p1.GetPath()), str(p2.GetPath()), vol_overlap(b1, b2))
            for (p1, b1), (p2, b2) in itertools.combinations(items, 2)
            if ("Stations" in str(p1.GetPath()) or "Stations" in str(p2.GetPath()))
            and vol_overlap(b1, b2) > 1e-7]
print()
print("复核：陷进台面 %d 个；新工位相关体积重叠 %d 对" % (len(buried), len(hard_bad)))
for n, dz in buried[:8]:
    print("   低于台面 %+.4f m  %s" % (dz, n))
for a, b, v in hard_bad[:8]:
    print("   重叠 %.1f cm^3  %s <-> %s" % (v * 1e6, a, b))

ok = st.GetRootLayer().Export(DST)
print("\n导出 v06：", "成功" if ok else "失败", os.path.basename(DST),
      os.path.getsize(DST), "字节")
