"""场景可交互资产盘点 → outputs/task_suite/scene_inventory_<ver>.json

任务套件里写的每一条 prim 路径都必须能在这份清单里查到，否则就是凭记忆编的。
之前 auto_tube_pick.py 踩过的坑正是这个：脚本里写的
`RackPoolLarge/Tube_A01/TubeWithCap/Tube/Tube` 在 v06 里根本不存在。

只读 USD 不启仿真，所以拿到的是**作者写入的姿态**（可信），不是仿真回写姿态。
判"物体现在在哪"可以；判"物体被仿真成什么样"必须另走 eval_task_goal.py。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/survey_scene_assets.py [scene] [--out json]
"""
import argparse
import json
import math
import os
import sys
import warnings

warnings.filterwarnings("ignore")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 这些前缀下的刚体才算"任务里能拿的东西"；仪器外壳、房间、桌面单独归类。
FAMILIES = [
    ("tube_large", "/World/TaskArea/Objects/Tubes"),
    ("tube_centri", "/World/TaskArea/Objects/CentrifugeTubes"),
    ("tip", "/World/TaskArea/Objects/Tips"),
    ("station_consumable", "/World/TaskArea/Stations"),
    ("instrument", "/World/TaskArea/Equipment"),
    ("fixture", "/World/TaskArea/Fixtures"),
    ("other", "/World"),
]


def family_of(path):
    for name, prefix in FAMILIES:
        if path.startswith(prefix + "/") or path == prefix:
            return name
    return "other"


def is_sim_root(prim):
    """没有带 RigidBodyAPI 的祖先 —— PhysX 才会把它当独立刚体。"""
    a = prim.GetParent()
    while a:
        if a.HasAPI(UsdPhysics.RigidBodyAPI):
            return False
        a = a.GetParent()
    return True


def is_col_root(prim):
    """碰撞层自己不开仿真体，PhysX 取的是最上面那个 CollisionAPI 节点；
    只盘根节点，否则一个大桌子会连带它的子 mesh 一起被数成十几个碰撞体。"""
    a = prim.GetParent()
    while a:
        if a.HasAPI(UsdPhysics.CollisionAPI):
            return False
        a = a.GetParent()
    return True


AXIS_VEC = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}


def quat_rot(w, iv, v):
    """单位四元数 (w, (x,y,z)) 旋向量 v。

    自己算而不是调 Gf：这套 USD 构建里 Gf.Rotation 没绑 RotateVec，
    照名字猜一次就炸一次 —— 四元数旋转就三行，没有任何 API 可猜。
    """
    x, y, z = iv
    t = [2.0 * (y * v[2] - z * v[1]),
         2.0 * (z * v[0] - x * v[2]),
         2.0 * (x * v[1] - y * v[0])]
    cv = [y * t[2] - z * t[1], z * t[0] - x * t[2], x * t[1] - y * t[0]]
    return [v[i] + w * t[i] + cv[i] for i in range(3)]


def world_axis(prim, body0_path, stage, xf):
    """把 physics:axis 换成**世界系**单位向量，返回 (向量, 依据说明)。

    为什么要换算而不是照字母用：physics:axis 讲的是**关节系**里的轴，
    而关节系 = body0 的坐标系再套 physics:localRot0 —— 它跟世界的 X/Y/Z
    根本不是一回事。VortexGenie2 在 v03 里摆成 rotateXYZ=(0,0,90)，它的
    switch 写着 axis=X，PhysX 真正绕的却是世界 −Y/世界 Y；判据按字母投影
    等于打在垂直方向上，只剩转角大小、符号全丢。
    离心盖那条"实测 +180.00 对期望 −98.72"就是这么来的，不是驱动坏了。
    localRot0 在这套 USD 里是 GfQuatf，单位四元数 (1,0,0,0) 就是不平转。
    """
    a = prim.GetAttribute("physics:axis")
    tok = str(a.Get()) if a and a.Get() is not None else "X"
    v = list(AXIS_VEC.get(tok, AXIS_VEC["X"]))
    lr = prim.GetAttribute("physics:localRot0").Get() if \
        prim.GetAttribute("physics:localRot0") else None
    if lr is not None:
        w_, iv = float(lr.GetReal()), [float(c) for c in lr.GetImaginary()]
        if (w_, iv) != (1.0, [0.0, 0.0, 0.0]):
            v = quat_rot(w_, iv, v)
    if body0_path:
        src, note = stage.GetPrimAtPath(Sdf.Path(body0_path)), "body0"
    else:                       # 没有 body0 的关节（缺陷，见上面 bound 那条）
        src, note = prim, "joint_prim(无 body0)"
    m = xf.GetLocalToWorldTransform(src)
    w = m.TransformDir(Gf.Vec3d(*v))
    n = math.sqrt(sum(c * c for c in w))
    if n < 1e-9:
        return None, note
    return [round(float(c) / n, 6) for c in w], note


def ref_paths(prim):
    op = prim.GetMetadata("references")
    if not op:
        return []
    try:
        items = op.GetAddedItems()
    except Exception:
        return []
    out = []
    for it in items:
        s = str(it)
        out.append(s.split("@")[-2] if s.count("@") >= 2 else s)
    return out


def bbox_world(prim, xf):
    """世界系轴对齐包围盒，返回 {center,size,min,max}；取不到返回 None，不猜。

    为什么不只给 size：早先只存「prim 的世界平移 + 包围盒尺寸」，然后把
    z = pos_z − size_z/2 当成容器内底。这两者压根不是一个点 —— pos 是 prim
    自己的平移，包围盒中心是几何重心，网格原点扎在底面时能差一整层高。
    台面顶面就是被这个算错的（TableTop 报 z=0.005，实际工作面 0.859），
    皿内底同理差 11 mm —— 足够让 oracle 把枪头埋进底板，再一路顶穿桌子。

    别用 UsdGeom.Gprim(prim) 做前置判断：刚体根几乎都是 Xform，Gprim 的类型
    检查对它返回 False，会把所有对象都筛成"无包围盒"。
    """
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    try:
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    except Exception:
        return None
    if box.IsEmpty():
        return None
    a, b = box.GetMin(), box.GetMax()
    # Range3d 在这个 USD 构建里没绑 GetCenter，自己取中点，别再猜 API 名字。
    c = [(a[i] + b[i]) * 0.5 for i in range(3)]
    s = box.GetSize()
    if not all(math.isfinite(v) for v in list(a) + list(b) + list(s)):
        # 退化网格（空 points、法线全 NaN）会算出 NaN 包围盒。json.dump 会把
        # NaN 原样写出去，产出的是一个严格 JSON 解析器读不回来的清单文件，
        # 下游拿 node/其它语言一读就炸。这里当"量不出来"处理。
        return None
    rnd = lambda v: [round(float(x), 4) for x in v]
    return {"center": rnd(c), "size": rnd(s), "min": rnd(a), "max": rnd(b)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", nargs="?",
                    default=os.path.join(PROJ, "assets", "scenes", "biolab_main_v07.usda"))
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    scene = os.path.abspath(args.scene)
    ver = os.path.splitext(os.path.basename(scene))[0]
    out = args.out or os.path.join(PROJ, "outputs", "task_suite", "scene_inventory_%s.json" % ver)

    st = Usd.Stage.Open(scene)
    if not st:
        print("打不开：", scene)
        return 1
    mpu = UsdGeom.GetStageMetersPerUnit(st)
    up = UsdGeom.GetStageUpAxis(st)
    if abs(mpu - 1.0) > 1e-9 or up != "Z":
        print("!! 场景元数据不是 米/Z-up：metersPerUnit=%s upAxis=%s" % (mpu, up))
        print("   这条不拦，但所有尺寸和位姿都得按这个倍率重新解释。")

    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    inv = {
        "scene": os.path.relpath(scene, PROJ).replace("\\", "/"),
        "metersPerUnit": mpu,
        "upAxis": up,
        "rigid_bodies": [],
        "static_colliders": [],
        "slot_anchors": [],
        "articulations": [],
    }

    rb_count = nested = 0
    rb_paths = set()
    for prim in st.Traverse():
        path = str(prim.GetPath())
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        rb_count += 1
        rb_paths.add(path)
        if not is_sim_root(prim):
            nested += 1
            continue
        has_col = False
        for d in Usd.PrimRange(prim):
            if d.HasAPI(UsdPhysics.CollisionAPI):
                has_col = True
                break
        t = xf.GetLocalToWorldTransform(prim)
        p = t.ExtractTranslation()
        bb = bbox_world(prim, xf)
        inv["rigid_bodies"].append({
            "path": path,
            "family": family_of(path),
            "has_collision": bool(has_col),
            "pos": [round(float(p[0]), 4), round(float(p[1]), 4), round(float(p[2]), 4)],
            "size": bb["size"] if bb else None,
            "bbox": bb,
            "refs": ref_paths(prim),
            "nested_under_rb": False,
        })

    # 静态碰撞体（有 CollisionAPI 但不是刚体）：台面、地板、仪器外壳都在这类里。
    # 早先只盘刚体，"台面板"查无此项，于是"物体掉到 -19.5 m"到底是穿桌还是桌上
    # 本来就没板，光看刚体清单分不清。盘出来才知道：台面在，工作顶面 z=0.859，
    # 所以那是穿透，不是缺失 —— 两种病因的修法完全相反。
    for prim in st.Traverse():
        path = str(prim.GetPath())
        if path in rb_paths or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not is_sim_root(prim) or not is_col_root(prim):
            continue   # 挂在刚体或别的碰撞体下面的，已经算在祖先上了
        p = xf.GetLocalToWorldTransform(prim).ExtractTranslation()
        bb = bbox_world(prim, xf)
        inv["static_colliders"].append({
            "path": path,
            "type": str(prim.GetTypeName()),
            "family": family_of(path),
            "pos": [round(float(p[0]), 4), round(float(p[1]), 4), round(float(p[2]), 4)],
            "size": bb["size"] if bb else None,
            "bbox": bb,
        })

    # 被嵌套在别的刚体里的：PhysX 不单独仿真，任务判定不能引用它们。
    for prim in st.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and not is_sim_root(prim):
            inv.setdefault("ignored_nested_rigid_bodies", []).append(str(prim.GetPath()))

    # 槽位锚点：随机化初始化就靠这些。
    for prim in st.Traverse():
        path = str(prim.GetPath())
        if "/Slot" not in path and "SlotAnchors" not in path:
            continue
        if not prim.IsA(UsdGeom.Xformable):
            continue
        if list(prim.GetChildren()):
            continue  # 只收叶子锚点
        p = xf.GetLocalToWorldTransform(prim).ExtractTranslation()
        inv["slot_anchors"].append({
            "path": path,
            "group": os.path.dirname(path),
            "pos": [round(float(p[0]), 4), round(float(p[1]), 4), round(float(p[2]), 4)],
        })

    # 关节按"工位"归并：joint 挂在工位的 joints Scope 下，
    # 而 ArticulationRootAPI 在 worldBody 上，两者是兄弟不是父子，
    # 只扫 ART 子树会数出 0 个关节。
    def station_of(path):
        parts = path.split("/")
        return "/".join(parts[:5]) if len(parts) > 5 else path

    joints_by_station = {}
    for prim in st.Traverse():
        t = str(prim.GetTypeName())
        if not t.startswith("Physics") or not t.endswith("Joint"):
            continue
        path = str(prim.GetPath())
        station = station_of(path)
        # body0/body1 在 USD Physics 里是 Relationship，不是 Attribute。
        # 用 GetAttribute 读会得到 None，看起来像"关节没绑 body"，
        # 其实是读错了地方 —— 别再拿这个当结论。
        targets = []
        for name in ("physics:body0", "physics:body1"):
            rel = prim.GetRelationship(name)
            tg = rel.GetTargets() if rel else []
            targets.append(str(tg[0]) if tg else None)
        angular = t == "PhysicsRevoluteJoint"
        drive = "angular" if angular else ("linear" if t == "PhysicsPrismaticJoint" else None)
        # 世界系轴：判据要的是"PhysX 实际绕/沿哪个方向量"，不是资产里那个字母。
        aw, aw_from = (None, None)
        if drive:
            aw, aw_from = world_axis(prim, targets[0], st, xf)
        drv = {}
        if drive:
            for k in ("stiffness", "damping", "maxForce", "targetPosition", "targetVelocity"):
                if drive == "angular" and k in ("targetPosition", "targetVelocity"):
                    a = prim.GetAttribute("drive:angular:physics:%s" % k)
                elif drive == "linear" and k in ("targetPosition", "targetVelocity"):
                    a = prim.GetAttribute("drive:linear:physics:%s" % k)
                else:
                    a = prim.GetAttribute("drive:%s:physics:%s" % (drive, k))
                v = a.Get() if a else None
                drv[k] = None if v is None else round(float(v), 4)
        lo = prim.GetAttribute("physics:lowerLimit").Get()
        up = prim.GetAttribute("physics:upperLimit").Get()
        # localPos0：关节系相对 body0 的位置偏移。不写就等于命令 PhysX"关节系 = body0 系"，
        # 而锚点摆在 body1 原点时两者不同 —— PhysX 会把件**瞬移**过去
        # （日志：disjointed body transforms … snap objects together）。
        lp0 = prim.GetAttribute("physics:localPos0").Get()
        joints_by_station.setdefault(station, []).append({
            "path": path,
            "type": t,
            "axis": (lambda v: None if v is None else str(v))(
                prim.GetAttribute("physics:axis").Get()),
            "axis_world": aw,
            "axis_world_from": aw_from,
            "lower": None if lo is None else round(float(lo), 4),
            "upper": None if up is None else round(float(up), 4),
            "body0": targets[0],
            "body1": targets[1],
            # 关节必须两头都绑刚体才算"进了仿真图"。上一版把"只有 body1"也算合法
            # （理由是"静态外壳不是刚体，拿它当 body0 是非法引用"），实测反了：
            # 单侧的 Revolute/Prismatic 在 PhysX 里根本不成约束 —— 撤掉
            # ArticulationRootAPI 后 A 类那扇窗直接坠 203.024 m
            # （outputs/probe_v09_noart.log）。正确拓扑是底座开 kinematic 刚体当
            # body0 + 关节补 physics:localPos0，见 build_stations_a_class.emit_joints。
            # 所以 anchored_world 现在当**缺陷**报，不再算合法。
            "anchored_world": targets[0] is None and targets[1] is not None
                              and t != "PhysicsFixedJoint",
            "bound": (bool(targets[0] and targets[1])
                      or (t == "PhysicsFixedJoint" and bool(targets[1]))),
            "has_drive_api": drive is not None and bool(
                prim.GetAttribute("drive:%s:physics:type" % drive)),
            "localPos0": None if lp0 is None else [round(float(v), 4) for v in lp0],
            "drive": drv,
        })

    inv["joint_stations"] = [
        {"station": k, "n_joints": len(v), "n_bound": sum(1 for j in v if j["bound"]),
         "joints": v}
        for k, v in sorted(joints_by_station.items())
    ]

    for prim in st.Traverse():
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        path = str(prim.GetPath())
        p = xf.GetLocalToWorldTransform(prim).ExtractTranslation()
        station = station_of(path)
        js = joints_by_station.get(station, [])
        inv["articulations"].append({
            "path": path,
            "station": station,
            "family": family_of(path),
            "pos": [round(float(p[0]), 4), round(float(p[1]), 4), round(float(p[2]), 4)],
            "n_joints": len(js),
            "n_bound": sum(1 for j in js if j["bound"]),

        })

    fam = {}
    for rb in inv["rigid_bodies"]:
        fam[rb["family"]] = fam.get(rb["family"], 0) + 1
    inv["summary"] = {
        "rigid_bodies_total": rb_count,
        "sim_root_rigid_bodies": len(inv["rigid_bodies"]),
        "nested_ignored": nested,
        "by_family": fam,
        "slot_anchor_groups": len({a["group"] for a in inv["slot_anchors"]}),
        "slot_anchors_total": len(inv["slot_anchors"]),
        "articulations": len(inv["articulations"]),
        "collidable": sum(1 for rb in inv["rigid_bodies"] if rb["has_collision"]),
        "static_colliders": len(inv["static_colliders"]),
    }

    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(inv, f, ensure_ascii=False, indent=1)

    s = inv["summary"]
    print("场景:", inv["scene"])
    print("刚体 %d，其中可被 PhysX 单独仿真的独立根 %d（嵌套排除 %d），带碰撞 %d"
          % (s["rigid_bodies_total"], s["sim_root_rigid_bodies"], s["nested_ignored"], s["collidable"]))
    print("槽位锚点 %d 个，分 %d 组；关节树 %d 台" %
          (s["slot_anchors_total"], s["slot_anchor_groups"], s["articulations"]))
    print("分类:", json.dumps(s["by_family"], ensure_ascii=False))
    print("工位关节（USD 层：两头都绑刚体 = 关节进了仿真图；有 drive = 能被指令驱动）")
    bad_anchor, bad_lp0 = [], []
    for a in inv["joint_stations"]:
        n_drv = sum(1 for j in a["joints"] if j["has_drive_api"])
        print("   %-46s 关节 %d / 绑 body %d / 带 drive %d"
              % (a["station"], a["n_joints"], a["n_bound"], n_drv))
        for j in a["joints"]:
            if j["type"] == "PhysicsFixedJoint":
                continue
            if j["anchored_world"]:
                bad_anchor.append(j["path"])
            elif j["localPos0"] is None:
                bad_lp0.append(j["path"])
    if bad_anchor:
        print("!! %d 条活动关节只绑了 body1（锚在世界上）—— PhysX 里不成约束，"
              "件会直接掉：\n    %s" % (len(bad_anchor), "\n    ".join(bad_anchor)))
    if bad_lp0:
        print("!! %d 条关节没写 physics:localPos0 —— PhysX 会把件瞬移到关节系"
              "（disjointed body transforms）：\n    %s"
              % (len(bad_lp0), "\n    ".join(bad_lp0)))
    if not bad_anchor and not bad_lp0:
        print("   活动关节拓扑检查通过：全部双端绑刚体且写了 localPos0")
    # 字母轴 ≠ 世界轴的关节必须点名：拿 axis 的 X/Y/Z 直接当世界向量投影，
    # 在这些关节上要么打在垂直方向上（转角大小还在、符号是垃圾），要么连符号都反。
    # 判据 = 世界轴与"字母那一根的正方向"的点积，不到 0.99 就算不一致。
    def mismatch(j):
        if not j["axis_world"]:
            return None
        exp = [0.0, 0.0, 0.0]
        exp[{"X": 0, "Y": 1, "Z": 2}[j["axis"]]] = 1.0
        d = sum(a * b for a, b in zip(j["axis_world"], exp))
        return d if d < 0.99 else None
    skew = [(mismatch(j), j) for a in inv["joint_stations"] for j in a["joints"]
            if mismatch(j) is not None]
    if skew:
        print("!! %d 条关节的世界轴和 physics:axis 的字母不是同一根（摆放转角 + localRot0 造成的），"
              "判据只能用 axis_world：" % len(skew))
        for d, j in skew:
            i = max(range(3), key=lambda k: abs(j["axis_world"][k]))
            print("    %-52s axis=%s → 世界 %s%s  点积 %+0.3f%s"
                  % (j["path"].replace("/World/TaskArea/", ""), j["axis"],
                     "XYZ"[i], "" if j["axis_world"][i] > 0 else "-", d,
                     "（斜的，非轴对齐）" if abs(abs(j["axis_world"][i]) - 1) > 0.02 else ""))
    else:
        print("   所有活动关节的世界轴与 axis 字母一致")
    nosize = sum(1 for r in inv["rigid_bodies"] if not r["size"])
    if nosize:
        print("!! %d 个刚体取不到包围盒尺寸，随机化摆放的间距判据对这些不成立" % nosize)
    print("静态碰撞体（台面/地板/外壳，不是刚体但会接住东西）%d 个"
          % len(inv["static_colliders"]))
    # 按世界包围盒去重：同一个台面的 Visual 与 Collision 网格脚印常常一模一样，
    # 不去重前 12 行全是同一张桌子的不同壳。
    sc = [c for c in inv["static_colliders"] if c.get("bbox")]
    uniq, seen_box, dup = [], set(), 0
    for c in sorted(sc, key=lambda c: -c["bbox"]["size"][0] * c["bbox"]["size"][1]):
        key = tuple(c["bbox"]["min"]) + tuple(c["bbox"]["max"])
        if key in seen_box:
            dup += 1
            continue
        seen_box.add(key)
        uniq.append(c)
    print("去重后 %d 个不同形状（%d 个与其它网格共享同一包围盒）；按水平面积前 12："
          % (len(uniq), dup))
    for c in uniq[:12]:
        b = c["bbox"]
        print("   %-50s 脚印 %6.1f×%6.1f mm  顶 z=%.4f 底 z=%.4f"
              % (c["path"].replace("/World/", ""),
                 1000 * b["size"][0], 1000 * b["size"][1], b["max"][2], b["min"][2]))
    nosz = len(inv["static_colliders"]) - len(sc)
    if nosz:
        print("   （另有 %d 个静态碰撞体量不出包围盒，判空位时按零尺寸处理会漏判）" % nosz)
    print("→", os.path.relpath(out, PROJ).replace("\\", "/"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
