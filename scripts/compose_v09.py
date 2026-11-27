"""biolab_main_v08.usda + A 类工位 → biolab_main_v09.usda

为什么用"叠加一层"而不是改 v08：v08 本身是 v07 的 drive/限位修补层，
v07 又是 v03 的布局层 —— 这套版本链是唯一的回滚手段，新内容一律另起一层，
出问题直接退回 --scene biolab_main_v08.usda 就行。

写出来的层只做四件事：
  1. 把 assets/props/celllab/*.usda 引用进 /World/TaskArea/Stations/AClass/<Name>，
     落点/绕 Z 转角取自 build_stations_a_class.PLACEMENT（同一个数，别抄两遍）；
  2. 每台设备的 PhysicsArticulationRootAPI 现在**默认不加**（--art-root 1 才加）。
     原以为静态外壳 + 活动刚体必须打包成一个 articulation 才能把 drive 算稳，
     实测正好相反（outputs/probe_v09_stations.log）：打了 ART 根的 5 台设备
     create_articulation_view 全部拿不到 dof（Isaac 认为根上第一个刚体是子件，
     基座缺失 → 这条 reduced-coordinate 链建不起来），运行期写 USD
     targetPosition 读回 0.00；对照组离心盖**不是** articulation、双 body，
     同一条 USD 通道能把盖子转动。结论（outputs/probe_v09_body0.log）：不要 ART 根，
     改成"底座开 kinematic 刚体当 body0 + 关节写 physics:localPos0"，
     7 条关节全部由 USD drive 推到限位、静置零偏移；
  3. 每台补 extent（资产局部包围盒）和 ccl:station 的 customData，
     盘点脚本和语义层都靠这两样认这是工位而不是装饰；
  4. 往**只当起点**的工位锚点里补货（STOCK 那张表）。不补的话
     task_defs 推不出"从 25 位冻存盒取一支冻存管"这个初始状态，
     只能回退成"随便挑一支"，指令和场景互相矛盾。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/compose_v09.py            # 只打印计划
    D:/isaacsim-5.1.0/python.bat scripts/compose_v09.py --write    # 落 usda
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()

PROJ = os.path.dirname(HERE)
OUT = os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda")
BASE = "biolab_main_v08.usda"
PROP_REL = "../props/celllab"

import build_stations_a_class as bs  # noqa: E402

# 补货：把耗材实例铺进新工位的锚点。
# 为什么必须补：build_pick_place 要求"from 槽位里真的坐着东西"才推得出初始状态，
# 否则它回退到"随便挑一个该类实例"并警告（v09 首跑 spatial/01 就是这条），
# 写出来的指令自相矛盾 —— 任务说"从 25 位冻存盒里取一支冻存管"，
# 而那一格里什么都没有。
# 为什么用引用而不是拷贝子树：原语在 v03 里就是
#   def Xform "Cryovial" (prepend references = @../objects/cryovial_1-8ml.usda@)
# 复制一份引用 = 同一个网格、同一种物理，只有 translate 不同；
# 把 Mesh/Collision 子树抄进 v09 会让这份资产有两处真相。
# 只铺**当起点**的工位（co2 搁板/计数托/提筒位是终点，铺了反而让
# "放进这一格"开局就成立）。
# 第 4 项可以是锚点前缀（从头按序取 n 格），也可以是**显式格名列表**：
# 同一工位要铺两类以上耗材时必须指名，否则都从 _00 起铺 → 两样东西叠在
# 同一格里（皿和 96 孔板都是这么差点叠上的）。
STOCK = [
    # 类原语名（也是副本前缀） / 源资产 / 工位 / 锚点前缀或格名 / 铺几个
    ("Cryovial", "../objects/cryovial_1-8ml.usda", "CryoBox25", "BoxSlots_", 6),
    ("Dish100", "../objects/cell_dish_100.usda", "BiosafetyCabinet", "WorkSlots_", 2),
    # 96 孔板：套件里 object/wellplate_96 要求"板开局在安全柜台面上"，而全场只有
    # 一块、还被别的任务占着（task_defs 报"1 可用实例已用尽"）→ 补一块进空格。
    # WorkSlots_02 而不是 _00：皿已经占了 00/01，孔距 0.36 m，板 0.12×0.08 不互穿。
    ("WellPlate96", "../objects/pcr_plate_96well.usda", "BiosafetyCabinet",
     ["WorkSlots_02"], 1),
]


def class_sink(name):
    """该类原语的"原点在包围盒底面之上多少米"，从 v07 清单里读（同 build_stations 的依据）。

    锚点写的是**承托面**的高度，而 xformOp:translate 是刚体原点（PhysX 的
    rigid body position 就是它）—— 两者差的就是这个 sink。拿数据而不是拿
    "应该是 0"的直觉，Dish100 的原点在皿底还是皿口没人担保过。
    """
    import json
    with open(bs.INV, encoding="utf-8") as fh:
        inv = json.load(fh)
    for rec in inv["rigid_bodies"]:
        if rec["path"].endswith("/Stations/" + name):
            b = rec.get("bbox")
            if not b:
                raise SystemExit("%s 在清单里没有包围盒，算不出落点" % name)
            return rec["pos"][2] - b["min"][2], b["size"]
    raise SystemExit("清单 %s 里找不到 /Stations/%s，补货配置过期了" % (bs.INV, name))


def anchor_world(station, local_pos, rotz):
    """工位局部锚点 → 世界坐标（跟 compose 给设备本体用的同一套平移+绕 Z）。"""
    root, rz = bs.PLACEMENT[station]
    if abs(rz - rotz) > 1e-9:
        raise SystemExit("%s 的摆放角与补货配置不一致" % station)
    a = math.radians(rotz)
    c, s = math.cos(a), math.sin(a)
    return (root[0] + c * local_pos[0] - s * local_pos[1],
            root[1] + s * local_pos[0] + c * local_pos[1],
            root[2] + local_pos[2])


def stock_plan():
    """展开成 [(新原语名, 资产, 世界位置, 承托面锚点)]，顺序就是写出顺序。"""
    out, claimed, boxes = [], {}, []
    for cls, asset, station, want, n in STOCK:
        sink, size = class_sink(cls)
        spec = [f() for f in bs.STATIONS if f()["name"] == station][0]
        slots = spec["slots"]
        by_name = {s["name"]: s for s in slots}
        if isinstance(want, (list, tuple)):
            missing = [w for w in want if w not in by_name]
            if missing:
                raise SystemExit("%s 没有这些格子：%s（显式格名过期了）" % (station, missing))
            picks = [by_name[w] for w in want]
        else:
            picks = [s for s in slots if s["name"].startswith(want)][:n]
        if len(picks) < n:
            raise SystemExit("%s 的 %s 只有 %d 格，铺不下 %d 个"
                             % (station, want, len(picks), n))
        for i, slot in enumerate(picks, start=1):
            key = (station, slot["name"])
            if key in claimed:
                raise SystemExit("%s/%s 已被 %s 占了，补货配置把一格铺了两次"
                                 % (station, slot["name"], claimed[key]))
            claimed[key] = "%s_%d" % (cls, i)
            x, y, z = anchor_world(station, slot["pos"], bs.PLACEMENT[station][1])
            # 脚印重叠单独查：格子名不撞不等于不互穿（安全柜 0.36 m 孔距放得下
            # 皿和板，冻存盒 24 mm 孔距放不下两个 10.8 mm 直径以外的东西）。
            hw, hh = size[0] / 2.0, size[1] / 2.0
            for (ox, oy, osz, onm) in boxes:
                if (abs(x - ox) < (hw + osz[0] / 2.0 - 1e-6)
                        and abs(y - oy) < (hh + osz[1] / 2.0 - 1e-6)):
                    raise SystemExit("%s_%d 与 %s 在同一工位里脚印重叠，会互穿"
                                     % (cls, i, onm))
            boxes.append((x, y, size, "%s_%d" % (cls, i)))
            out.append(("%s_%d" % (cls, i), asset, (x, y, z + sink), slot["name"], z))
    return out



def local_extent(spec):
    """设备自身的局部包围盒（所有几何件的并），给 extent 用。"""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for p in spec["parts"]:
        a, b = p.local_box()
        if p.children:
            p_lo, p_hi = a, b
        else:
            p_lo = [a[i] - b[i] / 2.0 for i in range(3)]
            p_hi = [a[i] + b[i] / 2.0 for i in range(3)]
        for i in range(3):
            lo[i] = min(lo[i], p_lo[i])
            hi[i] = max(hi[i], p_hi[i])
    return lo, hi


def fmt(v):
    return "%.4f" % v


def build_text(art_root):
    lines = [
        "#usda 1.0",
        "(",
        "    metersPerUnit = 1",
        "    subLayers = [@./%s@]" % BASE,
        "    upAxis = \"Z\"",
        ")",
        "",
        "# A 类缺失工位（生物安全柜/CO2 培养箱/计数仪/冻存盒/液氮罐）",
        "# 由 scripts/build_stations_a_class.py 程序化建模，scripts/compose_v09.py 合成。",
        "over \"World\"",
        "{",
        "    over \"TaskArea\"",
        "    {",
        "        over \"Stations\"",
        "        {",
        # 这一层必须是 def 带类型，不能学 v07 写 over：只由 over-spec 生成的
        # prim 没有 concrete typeName，Usd.IsAbstract() 为真，而**默认遍历谓词
        # 会跳过 abstract prim** —— 2026-11-23 实测整个 AClass 子树在
        # stage.Traverse() 里一个都不出现（GetPrimAtPath 却能取到），
        # 也就是说 Isaac 的物理场景构建根本看不见这批工位。
        "            def Xform \"AClass\"",
        "            {",
    ]
    for f in bs.STATIONS:
        spec = f()
        name = spec["name"]
        pos, rotz = bs.PLACEMENT[name]
        lo, hi = local_extent(spec)
        lines += [
            "                def Xform \"%s\" (" % name,
            "                    customData = {",
            "                        dictionary \"ccl:station\" = {",
            "                            string cn = \"%s\"" % spec["cn"],
            "                            int joints = %d" % len(spec["joints"]),
            "                            int slots = %d" % len(spec["slots"]),
            "                        }",
            "                    }",
            ("                    prepend apiSchemas = [\"PhysicsArticulationRootAPI\"]"
             if art_root else "                    # 无 PhysicsArticulationRootAPI，见文件头第 2 条"),
            "                    references = @%s/%s.usda@</%s>" % (PROP_REL, name, name),
            "                )",
            "                {",
            "                    double3 xformOp:translate = (%s, %s, %s)"
            % (fmt(pos[0]), fmt(pos[1]), fmt(pos[2])),
        ]
        ops = ["\"xformOp:translate\""]
        if abs(rotz) > 1e-9:
            lines.append("                    float xformOp:rotateZ = %s" % fmt(rotz))
            ops.append("\"xformOp:rotateZ\"")
        lines += [
            "                    uniform token[] xformOpOrder = [%s]" % ", ".join(ops),
            "                    float3[] extent = [(%s, %s, %s), (%s, %s, %s)]"
            % (fmt(lo[0]), fmt(lo[1]), fmt(lo[2]), fmt(hi[0]), fmt(hi[1]), fmt(hi[2])),
            "                }",
            "",
        ]
    # 补货的副本要在 Stations 的花括号**里面**：USD 文本只认括号嵌套不认缩进，
    # 上一版把 stock_lines() 放在 "        }" 之后，副本就掉到 /World/TaskArea 下了。
    lines += ["            }"]
    lines += stock_lines()
    lines += ["        }", "    }", "}", ""]
    return "\n".join(lines)


def stock_lines():
    """/World/TaskArea/Stations 下的补货副本，缩进跟 AClass 那层平级。"""
    out = ["",
           "        # 工位里的耗材：把只当起点的锚点铺上东西（配置见文件上方 STOCK）",
           "        # 副本只带一个 translate，几何/物理/网格全从 @../objects/*.usda@ 引用过来"]
    for name, asset, pos, slot, _az in stock_plan():
        out += [
            "        def Xform \"%s\" (" % name,
            "            kind = \"component\"",
            "            customData = { string \"ccl:stockFor\" = \"%s\" }" % slot,
            "            prepend references = @%s@" % asset,
            "        )",
            "        {",
            "            double3 xformOp:translate = (%s, %s, %s)"
            % (fmt(pos[0]), fmt(pos[1]), fmt(pos[2])),
            "            uniform token[] xformOpOrder = [\"xformOp:translate\"]",
            "        }",
            "",
        ]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--art-root", default="0", choices=["0", "1"],
                    help="1=给每台工位补 PhysicsArticulationRootAPI（默认不补，理由见文件头）")
    args = ap.parse_args()

    text = build_text(args.art_root == "1")
    print("叠加层 %s：%d 行，%d 台设备（基线层 %s）"
          % (os.path.basename(OUT), text.count("\n"), len(bs.STATIONS), BASE))
    for f in bs.STATIONS:
        s = f()
        pos, rotz = bs.PLACEMENT[s["name"]]
        lo, hi = local_extent(s)
        print("   %-18s 落点(%6.3f,%6.3f,%6.3f) 局部包围盒 %.3f..%.3f × %.3f..%.3f × %.3f..%.3f"
              % (s["name"], pos[0], pos[1], pos[2], lo[0], hi[0], lo[1], hi[1],
                 lo[2], hi[2]))
    for name, _asset, pos, slot, az in stock_plan():
        print("   补货 %-12s 世界(%6.3f,%6.3f,%6.3f) 起于 %-14s 承托面 z=%.4f"
              % (name, pos[0], pos[1], pos[2], slot, az))
    if not args.write:
        print("（试跑，没写文件）")
        return 0
    if os.path.exists(OUT):
        os.remove(OUT)      # CreateNew/直接写都不覆盖已有层，这是本脚本的派生产物
    with open(OUT, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    print("→ %s" % OUT)

    # 立刻用 pxr 复核一遍能否解析、引用能不能落到正确的路径下
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.Open(OUT)
    if stage is None:
        raise SystemExit("v09 打不开，合成失败")
    n_rb = n_col = n_jt = n_slot = 0
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        sch = prim.GetAppliedSchemas()          # HasAPI 传字符串在这套 pxr 里不可靠
        if "PhysicsRigidBodyAPI" in sch:
            n_rb += 1
        if "PhysicsCollisionAPI" in sch:
            n_col += 1
        if prim.GetTypeName() in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
            n_jt += 1
        if p.startswith("/World/TaskArea/Stations/AClass") and "/SlotAnchors/" in p:
            n_slot += 1
    print("v09 复核：全场景刚体 %d、碰撞体 %d、关节 %d、新工位锚点 %d"
          % (n_rb, n_col, n_jt, n_slot))
    bad = []
    for f in bs.STATIONS:
        name = f()["name"]
        p = "/World/TaskArea/Stations/AClass/" + name
        prim = stage.GetPrimAtPath(p)
        if not prim.IsValid():
            bad.append(p + " 不存在")
            continue
        has_art = "PhysicsArticulationRootAPI" in prim.GetAppliedSchemas()
        if has_art != (args.art_root == "1"):
            bad.append("%s ArticulationRootAPI 状态=%s，与 --art-root %s 不符"
                       % (p, has_art, args.art_root))
        for (nm, _k, _ax, _lo, _hi, _d) in f()["joints"]:
            jp = p + "/Joints/" + nm
            jprim = stage.GetPrimAtPath(jp)
            if not jprim.IsValid():
                bad.append(jp + " 引用没落下来")
                continue
            # 这两样是驱动能不能落地的全部前提，合成完就地验一次，
            # 别等下一次 Isaac 实测（3 分钟一轮）才发现资产层退化了。
            for rel_name in ("physics:body0", "physics:body1"):
                rel = jprim.GetRelationship(rel_name)
                if not rel or not rel.GetTargets():
                    bad.append("%s 缺 %s（单侧关节在 PhysX 里不成约束）" % (jp, rel_name))
            lp0 = jprim.GetAttribute("physics:localPos0")
            if not lp0 or lp0.Get() is None:
                bad.append("%s 缺 physics:localPos0（PhysX 会 snap 件）" % jp)
    for name, _asset, pos, slot, az in stock_plan():
        p = "/World/TaskArea/Stations/" + name
        prim = stage.GetPrimAtPath(p)
        if not prim.IsValid():
            bad.append(p + " 补货副本引用没落下来")
            continue
        # 刚体在引用根上，碰撞体在它子树的 Collision 网格上（cryovial 是
        # Xform "Obj"[RB] → Cylinder"body_cyl"[Collision]）—— 上一版在同一个
        # prim 上同时找两个 API，8 个副本全被判成"不是带碰撞的刚体"，
        # 而 PhysX 认的正是这种父子分工。口径抄 survey_scene_assets.py。
        if "PhysicsRigidBodyAPI" not in prim.GetAppliedSchemas():
            bad.append("%s 引用过来没有刚体 API（%s）" % (p, list(prim.GetAppliedSchemas())))
        if not any(d.HasAPI(UsdPhysics.CollisionAPI) for d in Usd.PrimRange(prim)):
            bad.append("%s 子树里没有碰撞体，机械爪抓不到" % p)
        anc, anc_rb = prim.GetParent(), None
        while anc:
            if anc.HasAPI(UsdPhysics.RigidBodyAPI):
                anc_rb = str(anc.GetPath())
                break
            anc = anc.GetParent()
        if anc_rb:
            bad.append("%s 上面已经有个刚体 %s，PhysX 不会再把它当独立物体" % (p, anc_rb))
        rng = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                [UsdGeom.Tokens.default_]).ComputeWorldBound(prim) \
            .ComputeAlignedRange()
        gap = rng.GetMin()[2] - az
        print("   补货复核 %-12s 底面 z=%.4f 承托面 z=%.4f 差 %+0.4f m"
              % (name, rng.GetMin()[2], az, gap))
        if abs(gap) > 0.003:
            bad.append("%s 底面离承托面 %+.4f m（悬空或埋板）" % (p, gap))
    print("关节/根节点检查：%s" % ("全部通过" if not bad else "有问题：" + "；".join(bad)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
