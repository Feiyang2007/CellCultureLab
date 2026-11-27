"""查 ART 工位：托盘有没有碰撞体、关节 drive 什么参数、外壳是不是刚体。

为什么要单独查这个：任务判据里有 7 个关节谓据（开盖/压杆），前提是
"这些仪器确实能交互"。现在的实测是——reachable 跑到这 7 条时托盘被顶到
1e15 m 外面，报 "Disjointed body transform" + 负质量/无效惯量，
盖子一步不
动。也就是说这 7 条的判据现在还落不了地。

修法取决于两件事，都得先看数据：
1. 托盘带不带 CollisionAPI。带 → 给 drive 加 spring 会让托盘自己弹开
   （托盘是刚体、弹簧把它推到目标位，但它下面没有东西接着就会被顶飞）；
   不带 → 托盘可以自由穿过盖子网格，那 spring 才是安全的。
2. 外壳是不是独立刚体。是 → 开盖时外壳会跟着动（USD 里外壳 1e15 那条
   就是这个），得先把层级或 body 关系理清。

用法：D:/isaacsim-5.1.0/python.bat scripts/_probe_art.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
from pxr import Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_root = "/World/TaskArea/Stations"
_pre = Usd.Stage.Open(os.path.join(PROJ, "assets", "scenes", "biolab_main_v07.usda"))
STATIONS = [str(c.GetName()) for c in _pre.GetPrimAtPath(_root).GetChildren()
            if c.GetTypeName() == "Xform"]
print("Stations 下共有：%s" % ", ".join(STATIONS))

st = _pre
xf = UsdGeom.XformCache(Usd.TimeCode.Default())

for name in STATIONS:
    root = "/World/TaskArea/Stations/" + name
    prim = st.GetPrimAtPath(root)
    if not prim:
        print("%s: 不存在" % name)
        continue
    print("==== %s ====" % name)
    n_rb = n_col = 0
    plates = []
    for p in Usd.PrimRange(prim):
        path = str(p.GetPath())
        rb = p.HasAPI(UsdPhysics.RigidBodyAPI)
        col = p.HasAPI(UsdPhysics.CollisionAPI)
        if rb:
            n_rb += 1
        if col:
            n_col += 1
        if rb and ("plate" in path.lower() or "tray" in path.lower()
                   or "block" in path.lower() or "platform" in path.lower()):
            a, b = None, None
            try:
                r = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                      [UsdGeom.Tokens.default_,
                                       UsdGeom.Tokens.render]).ComputeWorldBound(p).ComputeAlignedRange()
                if not r.IsEmpty():
                    a, b = list(r.GetMin()), list(r.GetMax())
            except Exception:
                pass
            plates.append((path, rb, col, a, b))
    print("  刚体 %d 个，带碰撞 %d 个" % (n_rb, n_col))
    for path, rb, col, a, b in plates:
        print("  盘: %-52s rigid=%-5s collision=%-5s" % (path.replace(root, "~"), rb, col))
        if a:
            print("      bbox %s ~ %s" % ([round(v, 4) for v in a], [round(v, 4) for v in b]))
    for p in Usd.PrimRange(prim):
        if str(p.GetTypeName()) not in ("PrismaticJoint", "RevoluteJoint",
                                       "SphericalJoint", "FixedJoint"):
            continue
        print("  关节 %-56s %s" % (str(p.GetPath()).replace(root, "~"), p.GetTypeName()))
        rels = []
        for nm in ("physics:body0", "physics:body1"):
            r = p.GetRelationship(nm)
            if r:
                rels.append("%s=%s" % (nm, [str(t) for t in r.GetTargets()]))
        print("      ", " ".join(rels))
        # DriveAPI 在 USD 里是**一个** schema、按 schemaType 分 linear/angular，
        # 属性写在 joint prim 自己身上（不是子 prim），名字形如
        # physics:drive:linear:physics:damping。
        for kind in ("linear", "angular"):
            if not UsdPhysics.DriveAPI.HasAPI(p, kind):
                continue
            print("      drive[%s]:" % kind)
            for a in p.GetAttributes():
                nm = a.GetFullName()
                if "drive:" + kind not in nm and not nm.startswith("physics:drive:"):
                    continue
                if a.HasAuthoredValue():
                    print("        %-58s = %s" % (nm, a.Get()))
