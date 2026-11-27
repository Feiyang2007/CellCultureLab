"""诊断"Invalid PhysX transform"：把可疑部件的变换矩阵与包围盒一起摊开。

起因：eval 跑 v01.3 时刷了 646×3 条
    [Warning] omni.physx.plugin  Invalid PhysX transform detected for
        /World/TaskArea/Stations/CentriMini/body/{body,lid,rotor}
        /World/TaskArea/Stations/VortexGenie2/body/{body,switch,platform,knob}
（ThermalCycler 一条没有。）这两个工位正好在"12 个部件 0 个碰撞体"那批里，
而清单里 `VortexGenie2/body/platform` 还有一条"量不出包围盒"。
所以这三件事可能是同一件事：PhysX 拿到的矩阵本身不合法（奇异 / NaN / 零缩放），
那么包围盒、碰撞体、转角回读全都是垃圾 —— 和"drive 三项全 0"是两条独立的病。

只盘带物理接口的 prim（刚体/碰撞体/关节），不然纯视觉 mesh 的空包围盒会刷屏。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/_diag_xform.py [scene]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATIONS = [
    ("CentriMini", "/World/TaskArea/Stations/CentriMini"),
    ("VortexGenie2", "/World/TaskArea/Stations/VortexGenie2"),
    ("ThermalCycler", "/World/TaskArea/Stations/ThermalCycler"),
    ("Centrifuge", "/World/TaskArea/Equipment/Centrifuge"),
    ("BiosafetyCabinet", "/World/TaskArea/Stations/AClass/BiosafetyCabinet"),
]


def ops_of(prim):
    out = []
    for op in prim.GetMetadata("xformOpOrder") or []:
        a = prim.GetAttribute(str(op))
        out.append("%s=%s" % (str(op).split(":")[-1], a.Get()))
    return "  ".join(out) if out else "(无 xformOp)"


def det3(m):
    # GetDeterminant3 / GetRow3 —— 这套绑定没有 Get3x3()，也没有 Determinant()。
    # 三行三列的行列式才是"有没有被压成一片"的判据，第四行是平移，不参与。
    return float(m.GetDeterminant3())


def rows3(m):
    return [[float(c) for c in m.GetRow3(j)] for j in range(3)]


def main():
    scene = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        PROJ, "assets", "scenes", "biolab_main_v09.usda")
    st = Usd.Stage.Open(scene)
    if not st:
        print("打不开：", scene)
        return 1
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    print("场景:", os.path.relpath(scene, PROJ).replace("\\", "/"))
    for name, path in STATIONS:
        root = st.GetPrimAtPath(path)
        if not root:
            print("=== %s ===  场景里没有这台" % name)
            continue
        print("=== %s ===" % name)
        n_bad = n_seen = 0
        lines = []
        short = lambda q: q.replace(path + "/", "")
        for prim in Usd.PrimRange(root):
            t = str(prim.GetTypeName())
            is_phys = (prim.HasAPI(UsdPhysics.RigidBodyAPI)
                       or prim.HasAPI(UsdPhysics.CollisionAPI)
                       or t.startswith("Physics") and t.endswith("Joint"))
            if not is_phys:
                continue
            n_seen += 1
            p = short(str(prim.GetPath()))
            m = xf.GetLocalToWorldTransform(prim)
            rows = rows3(m)
            det = det3(m)
            finite = all(abs(v) < 1e30 for r in rows for v in r)
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            bb_ok = (not box.IsEmpty()) and all(
                abs(v) < 1e30 for v in list(box.GetMin()) + list(box.GetMax()))
            why = []
            if not finite:
                why.append("世界矩阵 NaN/Inf")
            elif abs(det) < 1e-12:
                why.append("世界矩阵奇异(等效零缩放)")
            if not bb_ok:
                why.append("包围盒量不出")
            if not why:
                continue
            n_bad += 1
            lines.append("   %-52s %-9s %s" % (p, "det=%+.2e" % det, "，".join(why)))
            lines.append("        自己：%s" % ops_of(prim))
            par = prim.GetParent()
            while par and str(par.GetPath()).count("/") >= 5:
                pm = xf.GetLocalToWorldTransform(par)
                pdet = det3(pm)
                lines.append("        祖先 %-18s det=%+.2e  %s"
                             % (par.GetName(), pdet, ops_of(par)))
                par = par.GetParent()
        # 运行期报 Invalid PhysX transform 的那两台，作者态矩阵全是对的，
        # 所以矩阵本身不是病因。真正能一眼看出的差异只剩"这具刚体有没有质量和几何"：
        # 零碰撞体 + 没写 physics:mass 的动态刚体，惯量是求解器现算的，
        # 算不出来（0 或退化）位姿就是 NaN —— 这正是 B 批要补碰撞体那一步。
        print("   刚体属性（碰撞体/几何/mass/kinematic）：")
        for prim in Usd.PrimRange(root):
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            ncol = sum(1 for d in Usd.PrimRange(prim)
                       if d.HasAPI(UsdPhysics.CollisionAPI))
            ngeo = sum(1 for d in Usd.PrimRange(prim) if d.IsA(UsdGeom.Gprim))
            ma = prim.GetAttribute("physics:mass").Get()
            kin = prim.GetAttribute("kinematicEnabled").Get()
            print("      %-46s 碰撞 %d 几何 %d mass=%s kinematic=%s"
                  % (short(str(prim.GetPath())),
                     ncol, ngeo, ma, kin))
        for ln in lines[:40]:
            print(ln)
        if len(lines) > 40:
            print("   （另 %d 行省略）" % (len(lines) - 40))
    return 0


if __name__ == "__main__":
    sys.exit(main())
