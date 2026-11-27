"""修补货资产里的**退化盒碰撞体**：`def Box` 写成了 `custom normal3f[] extent`，
既没有 UsdGeom.Box 真正读的 `extents`，`size` 也是自定义的 float ——
USD 认这个 prim 是个"零尺寸盒"，PhysX 给它建出来的形状半长全 0，
等于这个刚体**没有任何有效碰撞体**（幽灵体），一进仿真就穿桌自由落体。

怎么暴露出来的：reachable 判据里 96 孔板放哪儿都掉到 z ≈ −2（dz 恒等于 −3.0738，
皿和冻存管同位置却稳稳的）。差别的来源是这三台资产里
只有 `Cylinder` 写对了（radius/height + 真 extent），`Box` 全写错了：
皿有 cylinder 兜着、管有 cylinder 兜着，而孔板 22 个碰撞体**全是 Box**，一个都不成立。

为什么场景里的原件没事：`WellPlate96` 那批原件在 v04 里按 **prim 路径**逐条覆盖过
（改成 `Cube` + `size=2` + `xformOp:scale:physicalSize`，见 biolab_main_v04.usda）。
覆盖是写死在路径上的，v09 新加的副本 `WellPlate96_1` 走的是同一份资产、
但不是那条路径，所以拿回来的是没修的原始 Box —— 这正是"改资产"和"改场景某条路径"
的区别，也是补货这步真正的坑。

修法照 v04 那套已经验证能用的写法：Box → Cube（size=2 就是半边长 1），
半边长放进 `xformOp:scale:physicalSize`，extent 写成 schema 的 float3[]。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/fix_stock_colliders.py           # 只报要改什么
    D:/isaacsim-5.1.0/python.bat scripts/fix_stock_colliders.py --write   # 落盘并回读校验
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

try:
    from pxr import PhysxSchema  # 只有 --write 需要；pip 版离线 pxr 载不动它的 pyd
except ImportError:
    PhysxSchema = None

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FILES = ["cryovial_1-8ml.usda", "cell_dish_100.usda", "pcr_plate_96well.usda"]


def is_degenerate_box(prim):
    """Box 且没有 `extents`（UsdGeom.Box 真正读的那条）→ 零尺寸。"""
    if prim.GetTypeName() != "Box":
        return False
    ext = prim.GetAttribute("extents")
    if ext and ext.Get() not in (None, []):
        return False
    e = prim.GetAttribute("extent")
    v = e.Get() if e else None
    return bool(v) and len(v) == 2


def half_of(prim):
    v = prim.GetAttribute("extent").Get()
    return [abs(float(v[1][i] - v[0][i])) / 2.0 for i in range(3)]


def convert(prim, write, stage=None):
    """把一个退化 Box 变成 v04 那种 Cube + physicalSize，返回说明串。"""
    h = half_of(prim)
    if not all(x > 1e-9 for x in h):
        return "半边长全 0，跳过（这个碰撞体本来就没体积）%s" % h
    path = str(prim.GetPath())
    tr = prim.GetAttribute("xformOp:translate").Get()
    if not write:
        return "Box → Cube size=2 physicalSize=(%.5f, %.5f, %.5f)" % tuple(h)
    # extent/size 是 custom 的、类型还和 schema 不一样（normal3f[] / float），
    # 同名重建会被"已存在但类型不同"挡回来；Sdf.Layer 也没有 RemovePrimProperty。
    # 这两个碰撞体都是叶子 prim、没有子级，所以整条删掉再 Cube.Define 建回来最干净。
    if stage is None:
        raise SystemExit("convert(write=True) 需要传 stage")
    if PhysxSchema is None:
        raise SystemExit("离线 pxr 里没有 PhysxSchema，改用 SimulationApp 跑 --write")
    stage.RemovePrim(Sdf.Path(path))
    cube = UsdGeom.Cube.Define(stage, path)
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    PhysxSchema.PhysxCollisionAPI.Apply(prim)
    prim.CreateAttribute("physxCollision:contactOffset",
                         Sdf.ValueTypeNames.Float).Set(9.9999997e-05)
    if tr is not None:
        prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3).Set(tr)
    prim.CreateAttribute("xformOp:scale:physicalSize",
                         Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*h))
    order = ["xformOp:translate", "xformOp:scale:physicalSize"] if tr is not None \
        else ["xformOp:scale:physicalSize"]
    # SetXformOpOrder 收的是 UsdGeomXformOp 对象，不是字符串；
    # xformOpOrder 本身就是 uniform token[]，直接写 token 列表。
    # xformOpOrder 本身就是 token[]，直接写 token 列表（这套绑定把 List 类型都叫 Array）。
    prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray).Set(order)
    UsdGeom.Imageable(prim).MakeInvisible()
    return "已改成 Cube physicalSize=(%.5f, %.5f, %.5f)" % tuple(h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true", help="真改文件；不加只报数")
    ap.add_argument("--only", default=None, help="只处理文件名含这个串的资产")
    args = ap.parse_args()

    for fn in FILES:
        if args.only and args.only not in fn:
            continue
        path = os.path.join(PROJ, "assets", "objects", fn)
        if not os.path.exists(path):
            print("!! 没有资产文件", path)
            return 1
        st = Usd.Stage.Open(path)
        targets = [p for p in st.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)
                   and is_degenerate_box(p)]
        print("%-26s 退化盒碰撞体 %d 个" % (fn, len(targets)), end="")
        if not targets:
            print("（已经是好的）")
            continue
        if not args.write:
            for t in targets[:2]:
                print("   例 %s %s" % (str(t.GetPath()).split("/")[-1], convert(t, False)))
            print("   （共 %d 个，加 --write 才改）" % len(targets))
            continue
        print()
        layer = st.GetRootLayer()
        with Sdf.ChangeBlock():
            for t in targets:
                convert(t, True, stage=st)
        # 这个 USD 构建里 Sdf.Layer.Export() 必须给文件名，不跟 0.20 的无参重载。
        layer.Export(os.path.abspath(path))
        # 回读校验：重新打开，量包围盒，半边长必须和写进去的对得上
        st2 = Usd.Stage.Open(path)
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                  [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        bad = []
        for p in st2.Traverse():
            if not p.HasAPI(UsdPhysics.CollisionAPI) or p.GetTypeName() != "Cube":
                continue
            b = cache.ComputeWorldBound(p).ComputeAlignedRange()
            want = p.GetAttribute("xformOp:scale:physicalSize").Get()
            if b.IsEmpty():
                bad.append("%s 包围盒还是空的" % p.GetPath())
                continue
            got = [(b.GetMax()[i] - b.GetMin()[i]) / 2.0 for i in range(3)]
            if any(abs(got[i] - float(want[i])) > 1e-4 for i in range(3)):
                bad.append("%s 半边长对不上：写 %s 量到 %s"
                           % (p.GetPath(), list(want), [round(g, 5) for g in got]))
        print("   回读校验：%d 个 Cube 包围盒与 physicalSize 一致%s"
              % (len(targets), "" if not bad else " —— 但有 %d 个问题" % len(bad)))
        for m in bad[:5]:
            print("   !!", m)
        if bad:
            return 3
    if not args.write:
        print("（试跑，没动文件）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
