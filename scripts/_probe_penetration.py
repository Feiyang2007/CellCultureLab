"""落点穿模探针：同一件物体的**视觉包围盒**和**碰撞包围盒**不是一个东西。

起因：reachable 模式里 96 孔板往"CO2 上层搁板"和"实验台面"放，oracle 摆完之后
一路掉到 z = −1.78 / −2.19（dz 都是 −3.0738），而同一趟里 100 mm 皿放安全柜台面
dz = −0.0 完美。上一轮枪头掉到 −19.5 m 也是同一类现象。
判据用的是"承托面 + sink"，sink 从作者态 `pos_z − bbox.min_z` 现取 ——
如果 bbox 取的是**视觉**网格，而 PhysX 用的是**碰撞**网格，
两者底面差几毫米就够把物体起手埋进 10 mm 厚的台面板里，
求解器朝下解算 → 穿出去 → 自由落体（§2 那条铁律的另一半）。

这里把两类包围盒分开量，谁的底面更低一目了然。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/_probe_penetration.py [scene]
"""
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SUITE = os.path.join(PROJ, "tasks", "celllab_suite_v01.yaml")


def main():
    scene = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        PROJ, "assets", "scenes", "biolab_main_v09.usda")
    import yaml
    with open(SUITE, encoding="utf-8") as f:
        suite = yaml.safe_load(f)
    st = Usd.Stage.Open(scene)
    if not st:
        print("打不开：", scene)
        return 1
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    print("场景:", os.path.relpath(scene, PROJ).replace("\\", "/"))
    print("sink = 原点离底面高度（米）。vis=视觉网格，col=碰撞网格。"
          "Δ=col_min−vis_min：负数=碰撞体比视觉低，按视觉摆就会穿模")
    print("%-22s %-28s %8s %8s %9s %9s" %
          ("类", "代表 prim", "sink_vis", "sink_col", "Δmin(m)", "Δ(mm)"))
    n_bad = 0
    for name, spec in suite["objects"].items():
        rx = re.compile(spec["prim_pattern"])
        hit = next((str(p.GetPath()) for p in st.Traverse()
                    if p.IsA(UsdGeom.Xformable) and rx.match(str(p.GetPath()))), None)
        if not hit:
            print("%-22s %-28s  场景里没有" % (name, spec["prim_pattern"][:26]))
            continue
        prim = st.GetPrimAtPath(hit)
        t = xf.GetLocalToWorldTransform(prim).ExtractTranslation()
        vis, col = [], []
        for d in Usd.PrimRange(prim):
            if d.HasAPI(UsdPhysics.CollisionAPI):
                b = cache.ComputeWorldBound(d).ComputeAlignedRange()
                if not b.IsEmpty():
                    col.append(b.GetMin()[2])
            if d.IsA(UsdGeom.Gprim) and not d.HasAPI(UsdPhysics.CollisionAPI):
                b = cache.ComputeWorldBound(d).ComputeAlignedRange()
                if not b.IsEmpty():
                    vis.append(b.GetMin()[2])
        # 有些资产的碰撞 prim 同时也是唯一的网格，上面那条"非碰撞"筛法会漏掉视觉，
        # 所以视觉另算一次：整棵子树的 min。
        ball = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if not ball.IsEmpty():
            vis = vis or [ball.GetMin()[2]]
        if not vis or not col:
            print("%-22s %-28s %8s %8s   （缺一类网格：视觉 %d 碰撞 %d）"
                  % (name, hit.replace("/World/TaskArea/", ""),
                     "%.4f" % (t[2] - min(vis)) if vis else "-",
                     "%.4f" % (t[2] - min(col)) if col else "-",
                     len(vis), len(col)))
            continue
        sv, sc = t[2] - min(vis), t[2] - min(col)
        dmin = min(col) - min(vis)
        n_bad += 1 if dmin < -1e-4 else 0
        print("%-22s %-28s %8.4f %8.4f %+9.4f %+9.1f%s"
              % (name, hit.replace("/World/TaskArea/", ""), sv, sc, dmin, 1000 * dmin,
                 "   ← 碰撞底面更低" if dmin < -1e-4 else ""))
    print("\n碰撞底面低于视觉底面的类：%d 个（这些类按视觉 sink 摆放起手就穿模）" % n_bad)
    return 0


if __name__ == "__main__":
    sys.exit(main())
