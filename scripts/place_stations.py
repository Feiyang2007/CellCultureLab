"""
把新转出来的仪器与耗材摆进大场景，生成 biolab_main_v03.usda。

两条硬约束（都由 layout_probe.py 量出来的真实数字决定，不是拍脑袋）：
  1. 必须在台面内：x∈[-1.999,-0.441]，y∈[-3.457,1.027]，台面高 z=0.872
  2. 必须在机械臂可达带内：两条 UR5e 底座都在 x=-1.905，臂展按 0.85 m 算
     → 中心点 x 不能大于 -1.05（再往外夹不到），y 只在 [-2.88,-0.29] 这段够得着
另外：与现有工位（离心机、移液枪、各种架子）的占地不能重叠，留 3 cm 间隙。

摆放顺序按细胞培养动线沿 y 轴排开：耗材存放 → 移液 → 离心 → 混匀 → 扩增。

用法：
    PY="D:/IsaacSim5/env/Scripts/python.exe"
    $PY scripts/place_stations.py --dry     # 只看排布方案，不写文件
    $PY scripts/place_stations.py           # 生成 ../assets/scenes/biolab_main_v03.usda
"""
import os, sys, json, traceback

ROOT = os.environ.get("ISAAC_PY_ROOT", r"D:/IsaacSim5/env/Lib/site-packages/isaacsim")
U = os.path.join(ROOT, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(ROOT, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENES = os.path.join(PROJ, "assets", "scenes")
SRC = os.path.join(SCENES, "biolab_main_v02.usda")
DST = os.path.join(SCENES, "biolab_main_v03.usda")

# 台面与可达（来自 layout_probe.py 的实测输出）
TABLE = (-1.999, -0.441, -3.457, 1.027)     # xmin xmax ymin ymax
REACH = (-1.999, -1.090, -2.830, -0.350)    # 中心点允许区（给臂展留 4 cm 余量）
BENCH_TOP = 0.872
CLEAR = 0.03                                 # 工位间最小间隙（米）

# (显示名, usd 相对路径, 期望 y（越靠前越靠"上游工序"）, 朝向角度)
WANT = [
    ("TipBox24",     "../objects/tip_box.usd",                -0.45, 0),
    ("Cryovial",     "../objects/cryovial_1-8ml.usd",         -0.62, 0),   # 单支冻存管，不是盒
    ("WellPlate96",  "../objects/pcr_plate_96well.usd",       -0.78, 0),
    ("VortexGenie2", "../instruments/vortex_mixer_genie_2.usd", -2.85, 90),
    # 5910 暂不摆：导入器产物包围盒 0.92×0.73 m，比实物大三倍，几何有问题待重转
    ("ThermalMixer", "../instruments/thermal_mixer_eppendorf_c.usd",    -1.55, 90),
    ("ThermalCycler", "../instruments/thermal_cycler_biorad_c1000.usd", -0.95, 90),
    ("CentriMini",   "../instruments/centrifuge_tiangen_tgear_mini.usd", -1.35, 90),
    ("Dish100",      "../objects/cell_dish_100.usd",          -0.55, 0),
    ("PlateRack60",  "../objects/centrifuge_plate_60well.usd", -1.15, 0),  # 60 孔板架
]


def bbox_of(stage, prim, xfc):
    rng = xfc.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        return None
    return rng.GetMin(), rng.GetMax()


def existing_footprints(stage):
    """现有工位的占地 (xmin,xmax,ymin,ymax,名字)。"""
    xfc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)
    out = []
    ta = stage.GetPrimAtPath("/World/TaskArea")
    for prim in Usd.PrimRange(ta):
        p = str(prim.GetPath())
        if p.count("/") != 4:           # 只看 TaskArea 下的一层子级
            continue
        r = bbox_of(stage, prim, xfc)
        if r:
            (x0, y0, z0), (x1, y1, z1) = r
            out.append((x0, x1, y0, y1, p.split("/")[-1]))
    return out


def measure_asset(path):
    """量一个待接入资产的世界包围盒，用来决定占地和落点。
    官方 MJCF/URDF 导入器的外层文件组合不全（网格在 _base.usd 里），
    所以那种情况改量 configuration/<名>_base.usd，否则会得到假的巨大包围盒。"""
    base = os.path.join(os.path.dirname(path), "configuration",
                        os.path.basename(path).rsplit(".", 1)[0] + "_base.usd")
    if os.path.isfile(base):
        path = base
    st = Usd.Stage.Open(path)
    if st is None:
        return None
    root = st.GetDefaultPrim()
    if not root:
        kids = st.GetPseudoRoot().GetChildren()
        if not kids:
            return None
        root = kids[0]
    xfc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)
    r = xfc.ComputeWorldBound(root).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    a, b = r.GetMin(), r.GetMax()
    return (b[0] - a[0], b[1] - a[1], b[2] - a[2], a[2], b[2])


def overlap(a, b):
    return not (a[1] + CLEAR <= b[0] or b[1] + CLEAR <= a[0]
                or a[3] + CLEAR <= b[2] or b[3] + CLEAR <= a[2])


def find_slot(w, d, blocked, want_y, x_pref=-1.50, edge=0.02):
    """在可达带内找 w×d 空位。代价 = 离期望 y 的距离 + 离"顺手带"中心 x 的距离，
    避免所有东西都堆在台子最边上（那里其实是臂座后方，不顺手）。"""
    best = None
    cx_lo, cx_hi, cy_lo, cy_hi = REACH
    step = 0.03
    for yi in range(0, int((cy_hi - cy_lo) / step) + 1):
        y = cy_lo + yi * step
        if not (cy_lo <= y <= cy_hi):
            continue
        for xi in range(0, int((cx_hi - cx_lo) / step) + 1):
            x = cx_lo + xi * step
            box = (x - w / 2, x + w / 2, y - d / 2, y + d / 2)
            if x - w / 2 < cx_lo or x + w / 2 > cx_hi:
                continue
            if y - d / 2 < cy_lo or y + d / 2 > cy_hi:
                continue
            # 必须整块落在台面内（台面 x∈[-1.999,-0.441] y∈[-3.457,1.027]）
            if not (TABLE[0] <= box[0] and box[1] <= TABLE[1]
                    and TABLE[2] <= box[2] and box[3] <= TABLE[3]):
                continue
            if any(overlap(box, bl) for bl in blocked):
                continue
            cost = abs(y - want_y) + 0.8 * abs(x - x_pref) + edge * 0
            if best is None or cost < best[0]:
                best = (cost, x, y, box)
    return best


def main():
    dry = "--dry" in sys.argv
    print("=" * 92)
    print("基准场景：", SRC)
    st = Usd.Stage.Open(SRC)
    blocked = [(b[0], b[1], b[2], b[3], b[4]) for b in existing_footprints(st)]
    print("现有工位 %d 个已占用：" % len(blocked))
    for b in blocked:
        print("   %-24s x %.3f~%.3f  y %.3f~%.3f" % (b[4], b[0], b[1], b[2], b[3]))

    added, failed = [], []
    for name, rel, want_y, yaw in WANT:
        # 官方导入器出 .usd，自写转换器出 .usda，两种都试
        path = os.path.normpath(os.path.join(SCENES, rel))
        if not os.path.isfile(path):
            for alt in (rel.replace(".usd", ".usda"), rel.replace(".usda", ".usd")):
                cand = os.path.normpath(os.path.join(SCENES, alt))
                if os.path.isfile(cand):
                    rel, path = alt, cand
                    break
        if not os.path.isfile(path):
            failed.append((name, "资产还没转出来"))
            continue
        m = measure_asset(path)
        if not m or max(m[0], m[1]) < 1e-4:
            failed.append((name, "量不到尺寸（网格可能是空的）"))
            continue
        w, d, h = m[0], m[1], m[2]
        if yaw in (90, 270):
            w, d = d, w          # 转 90° 后占地互换
        slot = find_slot(w, d, blocked, want_y)
        if not slot:
            failed.append((name, "可达带里放不下（占地 %.2f×%.2f m）" % (w, d)))
            continue
        _, x, y, box = slot
        blocked.append(box + (name,))
        added.append((name, rel, x, y, yaw, w, d, h))
        print("  [+ %-13s] 占地 %.3f×%.3f m  放到 (%.3f, %.3f) 朝向 %s°  高度 %.3f m"
              % (name, w, d, x, y, yaw, h))

    if failed:
        print("\n没放进去的：")
        for name, why in failed:
            print("   %-14s %s" % (name, why))

    if dry:
        print("\n(--dry) 未写文件。共可放置 %d 个工位" % len(added))
        return

    # ---------- 真正写进 v03 ----------
    for p in ("/World/TaskArea/Stations",):
        UsdGeom.Xform.Define(st, p)
    for name, rel, x, y, yaw, w, d, h in added:
        path = "/World/TaskArea/Stations/" + name
        prim = st.DefinePrim(path, "Xform")
        prim.GetReferences().AddReference(rel)
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        # 资产自带原点在别处时，用 -bbox 最低 z 把它"坐到"台面上；再补一个抬高件
        tz = BENCH_TOP + 0.001
        xf.AddTranslateOp().Set(Gf.Vec3d(x, y, tz))
        if yaw:
            xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, float(yaw)))
        # 打语义标签：让后面的数据生成脚本认得出这是可交互设备
        try:
            prim.SetMetadata("kind", "component")
        except Exception:
            pass
    # 注意：Save() 会写回 v02 本身！要另存必须用 Export(新路径)
    if not st.GetRootLayer().Export(DST):
        print("!! 另存失败")
    print("\n已写出：", DST)
    print("新增工位 %d 个（另 %d 个没放进去，见上）" % (len(added), len(failed)))


if __name__ == "__main__":
    main()
