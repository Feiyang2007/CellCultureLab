"""把 celllab_suite_v01.yaml 展开成具体任务实例。

设计要点：
1. YAML 里只写正则，几何全部来自 survey_scene_assets.py 的清单。正则匹配到 0 个
   对象 = 规格写错了，这里直接报错退出，不会静默生成一条引用不存在 prim 的任务。
   （auto_tube_pick.py 里那条 RackPoolLarge/Tube_A01/TubeWithCap/Tube/Tube 路径
   就是因为没人做这一步校验，跟着一个不存在它的场景跑了很久。）
2. 成败容差不是拍脑袋写的，而是从同组槽位的最小间距推出来：取 0.4 倍最小间距，
   上限 12 mm。否则相邻槽位的判据会互相重叠，"放进 A3"和"放进 A4"分不开。
3. 同一 seed 必须复现同一批任务，否则基线之间没法比。

用法：
    python scripts/task_defs.py [--seed 0] [--out outputs/task_suite/tasks_v01.json]
"""
import argparse
import fnmatch
import json
import os
import random
import re
import sys

import yaml

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------------------
# 清单加载与解析
# ---------------------------------------------------------------------------
class Inventory:
    def __init__(self, path):
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        self.data = d
        self.bodies = {rb["path"]: rb for rb in d["rigid_bodies"]}
        self.groups = {}
        for a in d["slot_anchors"]:
            self.groups.setdefault(a["group"], []).append(a)
        self.joints = {}
        for s in d.get("joint_stations", []):
            for j in s["joints"]:
                self.joints[j["path"]] = j
        # 静态碰撞体：台面、地板、仪器外壳。只有这类和刚体能"接住"东西。
        # 早先只盘刚体，"台面板"查无此项，于是穿桌掉到 −19.5 m 被误诊成
        # "场景里没有桌面" —— 两种病因的修法完全相反。
        self.colliders = {c["path"]: c for c in d.get("static_colliders", [])}

    def objects(self, pattern):
        rx = re.compile(pattern)
        return [self.bodies[p] for p in sorted(self.bodies) if rx.match(p)]

    def slots(self, pattern):
        rx = re.compile(pattern)
        out = []
        for g, items in self.groups.items():
            for a in sorted(items, key=lambda x: x["path"]):
                if rx.match(a["path"]):
                    out.append(a)
        return out

    @staticmethod
    def min_spacing(slot_list):
        """同组槽位两两最近的水平距离；只 1 个槽位时返回 None。"""
        pts = [(a["pos"][0], a["pos"][1]) for a in slot_list]
        best = None
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                d = ((pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2) ** 0.5
                if best is None or d < best:
                    best = d
        return best


def tol_from_spacing(spacing):
    if spacing is None:
        return 0.010
    return round(min(0.4 * spacing, 0.012), 4)


def slot_label(s):
    """给语言指令用的槽位号：Slot_R03_C07 -> R03_C07；整只容器则用它的名字。"""
    if not s:
        return "?"
    name = s["path"].rsplit("/", 1)[-1]
    return name[5:] if name.startswith("Slot_") else name


def prim_label(path):
    """实例的短标签。50 mL 管的刚体叶子叫 Tube，直接取 basename 会 10 支全同名，
    所以往上一级找第一个不是通用名的那层。"""
    parts = [p for p in path.split("/") if p]
    generic = {"Tube", "Cap", "Obj", "Body", "Collision", "visual", "Visual"}
    for name in reversed(parts):
        if name not in generic:
            return name
    return parts[-1]


# ---------------------------------------------------------------------------
# 语言模板
# ---------------------------------------------------------------------------
def lang_open(j):
    return "打开%s。" % j["cn"]


def lang_close(j):
    return "把%s关上。" % j["cn"]


def lang_press(j):
    return "压下%s。" % j["cn"]


# ---------------------------------------------------------------------------
# 判据构造
# ---------------------------------------------------------------------------
def rest_center(slot_z, offset):
    return round(slot_z + offset, 4)


def goal_in_slot(inv, suite, obj_prim, slot, xy_tol, z_off, z_tol):
    """z_off 是"对象中心高出槽位锚点多少"，由实测推得（见 Builder.rest_offset）。
    不能写 stand_h/2：锚点不是槽底 —— 三组实测分别差 −49.6 / 0 / +13.5 mm，
    用半高当基准会让判据永远差那一截，之前两条 tip 任务 BROKEN 就是这个。"""
    z = rest_center(slot["pos"][2], z_off)
    return {
        "type": "in_slot",
        "object": obj_prim,
        "slot": slot["path"],
        "target_xyz": [slot["pos"][0], slot["pos"][1], z],
        "xy_tol_m": xy_tol,
        "z_off_m": round(z_off, 4),
        # 立正放好：中心落在实测安放高度附近；不判 z 就分不清
        # "放进了槽" 和 "躺在旁边"。
        "z_expect": z,
        "z_tol_m": z_tol,
        "oracle_pose_xyz": [slot["pos"][0], slot["pos"][1], z],
    }


def _bbox_of(rec, what):
    """取清单里的世界包围盒 (center, size, min)，缺失就报错而不是退回近似。

    为什么不让它退：早先只存「prim 平移 pos + 包围盒尺寸 size」，然后拿
    z = pos_z − size_z/2 当容器内底。pos 是 prim 自己的平移、包围盒中心是几何
    重心，网格原点扎在底面时两者差一整层高 —— 台面顶面就被这算法报成 0.8592
    （真值 0.8717），皿内底同理低 11 mm，足够让 oracle 把枪头埋进皿底板，
    再被求解器顶穿桌子（实测 −19.55 m ≈ 0.5·9.81·2²，正好 2 秒自由落体）。
    """
    bb = rec.get("bbox")
    if not bb:
        raise SystemExit("%s %s 的清单里没有世界包围盒，先重跑 survey_scene_assets.py"
                         % (what, rec["path"]))
    return bb["center"], bb["size"], bb["min"]


def origin_lift(rec):
    """刚体原点高出其包围盒底多少（米）。

    判据比的是 PhysX get_rigidbody_transformation 的 position，实测那等于刚体
    prim 的原点、不是几何中心（tips 组按原点算 dz=+0.000 全绿）。所以要"底面
    贴住某个面"，得把原点抬到 那个面 + 原点到盒底 的高度上。
    这里偏保守用 pos − min，而不是 size_z/2 —— 后者默认原点就在几何中心，
    而对导入网格这多半不成立，错了就是半个身位。
    """
    _c, _s, omin = _bbox_of(rec, "物体")
    return rec["pos"][2] - omin[2]


def goal_in_volume(inv, suite, obj_prim, vol_prim, obj_rb):
    """把"放进容器里"翻译成可测判据：物体底面搭在容器内底上方一点。

    三条各自踩过：
    1. 放包围盒中心 = 埋进容器自己的底板；
    2. 用 pos 推底面 = 拿 prim 平移当几何中心；
    3. 把几何中心当 PhysX 读数 = 错一整个「原点到盒底」。判据读
       get_rigidbody_transformation 的 position，实测那就是刚体 prim 的原点
       （tips 组按原点算 dz=+0.000 全绿；按盒中心算要差半截身长）。
       所以摆的是**原点高度** = 内底 + 原点到盒底那一段。
    """
    rb = inv.bodies.get(vol_prim)
    if not rb:
        raise SystemExit("目标体积 %s 不在清单里，不能当判据" % vol_prim)
    vc, vs, vmin = _bbox_of(rb, "容器")
    os_ = _bbox_of(obj_rb, "物体")[1]
    # 容器包围盒底是**外**底，内底还要再高一个壁厚（皿底约 1–2 mm，量不出来）。
    # 宁可悬空 3 mm 让物理自己把它放下来，绝不能埋进去：物体一旦跨着 10 mm 厚的
    # 桌板求解，分离方向可能选向下，中心过了板面就一路自由落体穿桌。
    floor_z = vmin[2] + 0.003
    lift = origin_lift(obj_rb)
    z = round(floor_z + lift, 4)
    # XY 容差 = "物体脚印整体不出容器口"，不是"中心在容器中心多少米内"。
    # 原来用 min(sx,sy)*0.5，等于允许物体一半身子悬在容器外还算成功。
    inner = min(vs[0], vs[1]) * 0.5 - max(os_[0], os_[1]) * 0.5
    if inner <= 0.0:
        return None
    xy_tol = round(inner, 4)
    cx, cy = vc[0], vc[1]
    return {
        "type": "in_volume",
        "object": obj_prim,
        "volume": vol_prim,
        "target_xyz": [cx, cy, z],
        "xy_tol_m": xy_tol,
        "z_expect": z,
        # z 只容 5 mm：皿才 11–20 mm 深，容差给到半个容器深就等于
        # "坐在皿底、躺在桌上、穿桌掉下去" 三种情况全判成功。
        "z_tol_m": 0.005,
        "floor_z": round(floor_z, 4),
        "container_h": round(vs[2], 4),
        "z_off_m": round(lift, 4),
        "z_off_src": "computed:容器外底+3mm+原点到盒底",
        "oracle_pose_xyz": [cx, cy, z],
    }



class Surface:
    """在一块静态台面（不是容器、不是槽位）上找真正空着的落点。

    为什么必须搜而不是编一个坐标：台面 prim 的包围盒中心是 (−1.220, −1.215)，
    那正好压在耗材堆里（东西实际占在 x −1.69~−1.13、y −2.77~−0.36 那一片）。
    oracle 往那儿放一个就会把周围的管子顶飞，reachable 报出来是判据 BROKEN，
    实际上是我的摆放冲突 —— 这种假阴性会把整套 well-formedness 的可信度拖没。
    所以按脚印在台面矩形里扫网格，取不压到任何已有包围盒、且离对象当前位置
    足够远（否则 unsolved 一上来就成立）的格子里最近的一个。
    """

    # 清单里有 67 个碰撞网格量不出包围盒（Mesh 型、points 为空）。它们多是
    # 架子的分片碰撞体，落在架子自己的位置上。按"零尺寸"处理等于把整个架子
    # 当空地，会选出埋管子的落点；宁可按 pos 周围这么大一圈当障碍，
    # 少找一块空地，不要多判一块假空地。
    MESHLESS_R = 0.060
    STEP = 0.020          # 扫网格步长，米
    CLEAR = 0.020         # 落点脚印与障碍之间最少留的空隙
    MIN_MOVE = 0.100      # 离对象当前位置至少挪这么远

    def __init__(self, inv, path):
        c = inv.colliders.get(path)
        if not c:
            raise SystemExit("台面 %s 不在清单的 static_colliders 里 —— "
                             "先重跑 survey_scene_assets.py" % path)
        bb = c.get("bbox")
        if not bb:
            raise SystemExit("台面 %s 量不出包围盒，不能当落点依据" % path)
        self.path = path
        self.top = bb["max"][2]
        self.x0, self.y0 = bb["min"][0], bb["min"][1]
        self.x1, self.y1 = bb["max"][0], bb["max"][1]
        self.obs = self._obstacles(inv, path)

    def _obstacles(self, inv, path):
        out = []
        for rec in list(inv.colliders.values()) + list(inv.bodies.values()):
            if rec["path"] == path:
                continue
            bb = rec.get("bbox")
            if bb:
                # 只挡「顶面高出台面」的东西：台面自己的裙板/视觉网格
                # （mesh_004 顶 0.8717 == 台面顶）整块压着台面，算进去就全占了。
                if bb["max"][2] <= self.top + 0.001 or bb["min"][2] > self.top + 0.25:
                    continue
                out.append((bb["min"][0], bb["min"][1], bb["max"][0], bb["max"][1]))
            else:
                p = rec.get("pos") or [0, 0, 0]
                if not (self.top < p[2] < self.top + 0.25):
                    continue
                r = self.MESHLESS_R
                out.append((p[0] - r, p[1] - r, p[0] + r, p[1] + r))
        return out

    def free_patch(self, sx, sy, cur_xy):
        """返回 (x, y)：装得下 sx×sy 脚印、不压障碍、离 cur_xy ≥ MIN_MOVE 的落点。

        离当前位置那条是 unsolved 下界的一部分：落点挑在物体脚边，
        "放到台面上"这条一开局就成立，白送一分。
        """
        mx = sx * 0.5 + self.CLEAR
        my = sy * 0.5 + self.CLEAR
        if not (self.x1 - self.x0 > 2 * mx and self.y1 - self.y0 > 2 * my):
            return None
        cand = []
        gx = self.x0 + mx
        while gx <= self.x1 - mx + 1e-9:
            gy = self.y0 + my
            while gy <= self.y1 - my + 1e-9:
                d2 = (gx - cur_xy[0]) ** 2 + (gy - cur_xy[1]) ** 2
                if d2 >= self.MIN_MOVE ** 2:
                    cand.append((d2, gx, gy))
                gy += self.STEP
            gx += self.STEP
        cand.sort()
        for _d2, gx, gy in cand:
            ax0, ay0 = gx - mx, gy - my
            ax1, ay1 = gx + mx, gy + my
            if all(ax1 < bx0 or ax0 > bx1 or ay1 < by0 or ay0 > by1
                   for bx0, by0, bx1, by1 in self.obs):
                return gx, gy
        return None

    def claim(self, xy, sx, sy):
        """把选中的落点登记成障碍。

        不登记会出真事故：两条不同模板各自独立搜空位，两个一样大的对象（两支
        15 mL 管）会搜到**同一个**格子，oracle 一放就把第二支顶开，判据全废。
        """
        mx = sx * 0.5 + self.CLEAR
        my = sy * 0.5 + self.CLEAR
        self.obs.append((xy[0] - mx, xy[1] - my, xy[0] + mx, xy[1] + my))

    def rest_origin_z(self, obj_rb):
        """物体放在台面上时**刚体原点**的高度。

        和 goal_in_volume 同一个口径：判据读的是 PhysX 原点、不是几何中心，
        所以原点多高 = 面多高 + 盒底到几何中心那一段。
        """
        return round(self.top + 0.002 + origin_lift(obj_rb), 4)


def goal_on_surface(inv, obj_prim, surf, obj_rb):
    """在台面现搜一个真空着的落点并写成判据；找不到返回 None，由调用方跳过。

    判据类型仍用 in_volume：评测器对它做的就是"XY 进容差 + z 进容差"，
    和台面情形完全一致，没必要为摆放面再开一条判据分支。
    """
    osz = _bbox_of(obj_rb, "物体")[1]
    # 用 pos 而不是包围盒中心当"当前位置"：判据读的就是 pos，
    # 离它 10 cm 才保证 unsolved 下界开局不成立。
    patch = surf.free_patch(osz[0], osz[1], (obj_rb["pos"][0], obj_rb["pos"][1]))
    if patch is None:
        return None
    surf.claim(patch, osz[0], osz[1])
    z = surf.rest_origin_z(obj_rb)
    return {
        "type": "in_volume",
        "object": obj_prim,
        "volume": surf.path,
        "on_surface": True,
        "target_xyz": [round(patch[0], 4), round(patch[1], 4), z],
        # 台面是实打实的空场，落点周围留了 CLEAR 的空，容差给到 30 mm：
        # 落定时被歪一下算失败，滑出半个脚印算失败。
        "xy_tol_m": 0.030,
        "z_expect": z,
        "z_tol_m": 0.005,
        "footprint_mm": [round(1000 * osz[0], 1), round(1000 * osz[1], 1)],
        "z_off_src": "computed:台面顶+2mm+盒底到几何中心",
        "oracle_pose_xyz": [round(patch[0], 4), round(patch[1], 4), z],
    }


def goal_joint(spec, state, axis="X"):
    """按 YAML 里给了 open_m 还是 open_deg 决定判角度还是判位移，
    而不是按 op 名字猜 —— press 既可能是按压平移键也可能是压杆转动。

    axis 是判据的必需项，不是装饰：绕 X 转 100° 和绕 Z 转 100° 在
    "取两个刚体姿态差、投影到轴上算带符号角"这套算法里量出来的数完全不同，
    少了它调用方只能猜（判据脚本原来默认 (1,0,0)），而 A 类的门、仓盖、
    前窗偏偏是绕 Z 的 —— 猜 X 就把开得到 0 读成"坏了"。
    值来自场景清单里的 physics:axis（PhysX 真正拿去用的那份），
    USD 在该属性缺省时的默认轴是 X，所以 None 也按 X 处理。
    """
    linear = "open_m" in spec
    if linear:
        return {"type": "joint_linear", "joint": spec["path"], "state": state,
                "axis": axis,
                "expect_m": spec["closed_m"] if state == "closed" else spec["open_m"],
                "tol_m": spec.get("tol_m", 0.002)}
    return {"type": "joint_angle", "joint": spec["path"], "state": state,
            "axis": axis,
            "expect_deg": spec["closed_deg"] if state == "closed" else spec["open_deg"],
            "tol_deg": spec.get("tol_deg", 20.0)}


# ---------------------------------------------------------------------------
# 套件展开
# ---------------------------------------------------------------------------
class Builder:
    def __init__(self, suite, inv, rng):
        self.suite = suite
        self.inv = inv
        self.rng = rng
        self.tasks = []
        self.warnings = []
        self._obj_cache = {}
        self._slot_cache = {}
        self._surf_cache = {}
        self._occ = None
        self._class_of = None
        self._rest = {}

    def objs(self, name):
        if name not in self._obj_cache:
            spec = self.suite["objects"][name]
            got = self.inv.objects(spec["prim_pattern"])
            if not got:
                raise SystemExit("对象类 %s 的正则 %r 在 v07 清单里匹配到 0 个刚体"
                                 % (name, spec["prim_pattern"]))
            self._obj_cache[name] = (spec, got)
        return self._obj_cache[name]

    # ---- 占用与实测安放高度 ------------------------------------------------
    def all_obj_rbs(self):
        if self._class_of is None:
            m = {}
            for name in self.suite["objects"]:
                for rb in self.objs(name)[1]:
                    m[rb["path"]] = name
            self._class_of = m
        return self._class_of

    def occupancy(self):
        """每个槽位现在被哪个刚体占着。

        目标位必须挑空位。之前只排除了"起点那一个槽"，于是把枪头放到一个
        已经插着枪头的孔里 —— 物理上会被挤飞，实测横向弹开 71 mm。
        这种任务不是难，是不可能，报出来的 0% 是规格错的。

        半径取**本组槽距的一半**，不能拍一个固定值：先前用 50 mm，
        结果 A 号管架 18 个小管位全被"隔壁 60 位架的管"判成已占用，
        整个场景看起来一格都不空。空槽到最近对象的距离恰好是一个槽距，
        所以一半就是能把"占这格"和"占邻格"分开的最大安全值。
        """
        if self._occ is None:
            occ = {}
            rbs = self.all_obj_rbs()
            pts = [(p, self.inv.bodies[p]["pos"]) for p in rbs]
            for grp, items in self.inv.groups.items():
                sp = Inventory.min_spacing(items)
                r = 0.020 if sp is None else min(0.5 * sp, 0.020)
                for a in items:
                    best, bd = None, None
                    for p, pos in pts:
                        if abs(pos[2] - a["pos"][2]) > 0.15:
                            continue
                        d = ((pos[0] - a["pos"][0]) ** 2
                             + (pos[1] - a["pos"][1]) ** 2) ** 0.5
                        if bd is None or d < bd:
                            best, bd = p, d
                    if best is not None and bd <= r:
                        occ[a["path"]] = best
            self._occ = occ
        return self._occ

    def rest_offset(self, group, obj_name):
        """这一类对象放进这一组槽位后，中心该高出锚点多少 —— 量出来，别推。

        实测结论（scripts/_probe_z.py）：**锚点不是槽底**。
          TubeRack 大管位  锚点 z=0.9319，管中心 z=0.8823 → 偏 −49.6 mm（锚点在架口）
          60 位离心管架    锚点 z=0.8719，管中心 z=0.8719 → 偏 0（锚点与中心齐平）
          枪头架           同一个 XY 有两套锚点，0.9119（管身中段）和 0.9286（接近口部），
                           而管中心在 0.9254
        所以 stand_h/2（= 断言"锚点在物体底面"）在三种组上分别错 50 / 72 / 61 mm，
        这就是前两条 tip 任务 BROKEN 的真正原因 —— 不是物理不稳，是判据高度写错。

        取样顺序：同组 → 等高他组（同款架子）→ 不等高他组（借用，标注）→ stand_h/2（假设）。
        """
        key = (group, obj_name)
        if key in self._rest:
            return self._rest[key]
        occ = self.occupancy()
        classes = self.all_obj_rbs()
        z_ref = self.inv.groups[group][0]["pos"][2]
        same_grp, same_h, other_h = [], [], []
        for a in self.inv.groups[group]:
            owner = occ.get(a["path"])
            if owner and classes.get(owner) == obj_name:
                same_grp.append(round(self.inv.bodies[owner]["pos"][2] - a["pos"][2], 4))
        for grp, items in self.inv.groups.items():
            if grp == group:
                continue
            for a in items:
                owner = occ.get(a["path"])
                if not owner or classes.get(owner) != obj_name:
                    continue
                d = round(self.inv.bodies[owner]["pos"][2] - a["pos"][2], 4)
                if abs(a["pos"][2] - z_ref) <= 0.005:
                    same_h.append(d)
                else:
                    other_h.append(d)
        for deltas, src in ((same_grp, "同组实测"), (same_h, "等高组实测"),
                            (other_h, "他组借用")):
            if src == "他组借用":
                # 借用是这一轮真踩过的坑：锚点的约定**每组不一样**（TubeRack 锚在架口、
                # 60 位架锚在与中心齐平、枪头架同一 XY 两套），把 A 组的
                # "中心−锚点"搬到 B 组，等于赌两组用同一套约定。皿 → CO2 下层搁板
                # 就是这样借到 z_off=0.0，把 20 mm 厚的皿中心放进了板子里，
                # 运行期被 PhysX 顶飞 14 m。改成用实测包围盒半高（锚点在面这一条
                # 是场景里所有搁板/槽位共用的几何事实，孔板借到的 +0.0101 正好等于它自己的半高）。
                hs = sorted(rb["size"][2] for rb in self.objs(obj_name)[1]
                            if rb.get("size") and rb["size"][2] > 1e-4)
                if hs:
                    off = round(hs[len(hs) // 2] * 0.5, 4)
                    self._rest[key] = (off, len(hs), "几何半高")
                    return self._rest[key]
                continue
            if deltas:
                off = sorted(deltas)[len(deltas) // 2]
                self._rest[key] = (off, len(deltas), src)
                return self._rest[key]
        off = round((self.suite["objects"][obj_name].get("stand_h") or 0.0) * 0.5, 4)
        self.warnings.append(
            "%s 放进 %s 的安放高度无实测样本（同类在任何槽位组里都没有现成实例），"
            "退回 stand_h/2 = %.4f m，这是假设不是测量"
            % (obj_name, group.rsplit("/", 1)[-1], off))
        self._rest[key] = (off, 0, "assumed")
        return self._rest[key]

    def recept(self, name):
        spec = self.suite["receptacles"][name]
        if spec["kind"] == "slot_group":
            if name not in self._slot_cache:
                got = self.inv.slots(spec["pattern"])
                if not got:
                    raise SystemExit("容器 %s 的正则 %r 在清单里匹配到 0 个槽位"
                                     % (name, spec["pattern"]))
                self._slot_cache[name] = (spec, got, self.inv.min_spacing(got))
            spec, slots, spacing = self._slot_cache[name]
            return spec, slots, tol_from_spacing(spacing)
        if spec["kind"] == "surface":
            # 台面不是刚体、也没有槽位，落点靠 Surface 现搜，这里只把对象传下去。
            if name not in self._surf_cache:
                try:
                    self._surf_cache[name] = Surface(self.inv, spec["prim"])
                except SystemExit as e:
                    raise SystemExit("容器 %s：%s" % (name, e))
            return spec, [{"path": spec["prim"], "surface": self._surf_cache[name]}], 0.030
        rb = self.inv.bodies.get(spec["prim"])
        if not rb:
            raise SystemExit("容器 %s 指向 %s，但该 prim 不是清单里的独立刚体"
                             % (name, spec["prim"]))
        if not rb.get("bbox"):
            raise SystemExit("容器 %s 指向 %s，但清单里没有它的世界包围盒，"
                             "内底高度无从推起 —— 重跑 survey_scene_assets.py"
                             % (name, spec["prim"]))
        return spec, [rb], min(rb["size"][0], rb["size"][1]) if rb.get("size") else None

    def home_slot(self, obj_name, recept_name):
        """找该对象当前实际占着的槽位。

        直接取 occupancy 的反向索引，不再自己算一遍距离 —— "这支管在哪一格"
        和"这一格还被谁占着"必须是同一个判据，否则会出现一格两主，
        语言指令里的起点位和实际初始态对不上。
        """
        spec, obj_rbs = self.objs(obj_name)
        _, slots, _tol = self.recept(recept_name)
        mine = {rb["path"] for rb in obj_rbs}
        out = {}
        for s in slots:
            owner = self.occupancy().get(s["path"])
            if owner in mine:
                out[owner] = s
        return out

    def add(self, suite_name, tpl_idx, language, init, goal, cn=""):
        self.tasks.append({
            "id": "%s_%02d" % (suite_name, len([t for t in self.tasks if t["id"].startswith(suite_name)])),
            "suite": suite_name,
            "template": tpl_idx,
            "cn": cn,
            "language": language,
            "init": init,
            "goal": goal,
        })

    def init_entry(self, obj_name, rb, slot):
        spec = self.suite["objects"][obj_name]
        return {
            "object": obj_name,
            "object_cn": spec["cn"],
            "prim": rb["path"],
            "label": prim_label(rb["path"]),
            "slot": slot["path"] if slot else None,
            "authored_pos": rb["pos"],
            "grasp_z_offset": spec.get("grasp_z"),
            "size": rb.get("size"),
        }

    # ---- pick_place -------------------------------------------------------
    def build_pick_place(self, suite_name, tpl, used):
        obj_name = tpl["object"]
        spec, obj_rbs = self.objs(obj_name)
        src_name = tpl.get("from")
        dst_name = tpl.get("to")
        if src_name:
            src_spec, src_slots, _ = self.recept(src_name)
        else:
            # Object 组：对象就从它在场景里的当前位置拿，不指定来源架。
            src_spec = {"cn": "台面上"}
            src_slots = []
        dst_spec, dst_slots, dst_tol = self.recept(dst_name)

        # accepts 必须真生效，否则会生成"把 100 mm 皿塞进 60 位板架"这种
        # 几何上判得出、物理上不可能对的任务，把成功率高估。
        # varies/varies_to 走 _soft：不兼容的组合直接跳过；
        # 写死的 from/to/object 组合不兼容则是规格错误，直接停。
        allow = dst_spec.get("accepts")
        if allow and obj_name not in allow:
            if tpl.get("_soft"):
                return False
            raise SystemExit("%s 不接受 %s（accepts=%s），模板写法有误"
                             % (dst_name, obj_name, allow))

        homes = self.home_slot(obj_name, src_name) if src_name else {}
        cands = [(rb, homes.get(rb["path"])) for rb in obj_rbs if rb["path"] in homes]
        if not cands:
            if src_name:
                self.warnings.append("%s/%s：%d 个对象都不在 %s 的槽位附近，"
                                     "这条初始状态没法从场景现状推出来"
                                     % (suite_name, obj_name, len(obj_rbs), src_name))
            cands = [(rb, None) for rb in obj_rbs]

        self.rng.shuffle(cands)
        occ = self.occupancy()
        for rb, src_slot in cands:
            if src_name == dst_name:
                # 起点这一格在"取出来"之后是空的，但它当终点等于任务开局即完成，
                # 所以照样排除；其余占用格一律排除。
                pool = [s for s in dst_slots
                        if s["path"] != (src_slot or {}).get("path")
                        and occ.get(s["path"]) in (None, rb["path"])]
            else:
                pool = [s for s in dst_slots if occ.get(s["path"]) in (None, rb["path"])]
            if not pool:
                self.warnings.append(
                    "%s/%s→%s：%s 组 %d 个槽位没有可用的空位（满载），"
                    "同架换槽这条在本场景作者态下放不下 —— 要么初始化留空位，"
                    "要么这条模板换目的地"
                    % (suite_name, obj_name, dst_name, dst_name, len(dst_slots)))
                return False
            dst_slot = self.rng.choice(pool)
            sid = slot_label(src_slot)
            dst_is_slot = bool(dst_slot.get("group"))
            did = slot_label(dst_slot) if dst_is_slot else None
            key = (rb["path"], did)
            if key in used:
                continue
            if not rb.get("size"):
                self.warnings.append("%s：%s 在清单里量不出包围盒，"
                                     "无法核对它和 %s 的尺寸是否相容"
                                     % (suite_name, rb["path"], dst_name))
            if dst_is_slot:
                z_off, _n, z_src = self.rest_offset(dst_slot["group"], obj_name)
                # 只有实测过的安放高度才配得上紧 z 容差。借用或假设的基准本身
                # 就有几厘米的不确定度，再拿 20 mm 去卡就是把"规格不知道"当成
                # "策略没做好"，所以这时把容差放宽到半个身位并如实标注来源。
                z_tol = (round(max(2 * dst_tol, 0.02), 4) if z_src in ("同组实测", "等高组实测")
                         else round(max(0.5 * (spec.get("stand_h") or 0.0), 0.03), 4))
                goal = goal_in_slot(self.inv, self.suite, rb["path"], dst_slot, dst_tol,
                                    z_off, z_tol)
                goal["z_off_src"] = z_src
            else:
                if dst_spec["kind"] == "surface":
                    goal = goal_on_surface(self.inv, rb["path"],
                                           dst_slot["surface"], rb)
                    if goal is None:
                        self.warnings.append(
                            "%s：台面上找不到能放下 %s（%.1f×%.1f mm 脚印，且离原位 "
                            "%.0f cm 以上）的空格，这条组合跳过"
                            % (suite_name, rb["path"], 1000 * (rb.get("size") or [0, 0])[0],
                               1000 * (rb.get("size") or [0, 0])[1], 100 * Surface.MIN_MOVE))
                        continue
                else:
                    goal = goal_in_volume(self.inv, self.suite, rb["path"],
                                          dst_slot["path"], rb)
                if goal is None:
                    # 装不下就不出这条，而不是出一条"判得出、做不成"的任务。
                    self.warnings.append(
                        "%s：%s 的脚印塞不进容器 %s（%.1f×%.1f mm 对 %.1f×%.1f mm），"
                        "这条组合跳过" % (suite_name, rb["path"], dst_name,
                                          1000 * (rb.get("size") or [0, 0])[0],
                                          1000 * (rb.get("size") or [0, 0])[1],
                                          1000 * dst_slots[0]["size"][0],
                                          1000 * dst_slots[0]["size"][1]))
                    continue
            used.add(key)
            # 台面不是容器，说"放进"是错的中文；这类要求落到一块空位上。
            if dst_spec["kind"] == "surface":
                into = "放到%s空着的地方" % dst_spec["cn"]
            elif did:
                into = "放进%s的第 %s 位" % (dst_spec["cn"], did)
            else:
                into = "放进%s" % dst_spec["cn"]
            if src_name and did:
                lang = "把%s从%s的第 %s 位取出来，%s。" % (
                    spec["cn"], src_spec["cn"], sid, into)
            elif src_name:
                lang = "把%s从%s取出来，%s。" % (spec["cn"], src_spec["cn"], into)
            else:
                lang = "把%s拿起来，%s。" % (spec["cn"], into)
            self.add(suite_name, tpl.get("_i", 0), lang,
                     [self.init_entry(obj_name, rb, src_slot)], goal,
                     cn="取放")
            return True
        self.warnings.append("%s/%s：%s 可用实例已用尽" % (suite_name, obj_name, len(cands)))
        return False

    # ---- joint ops --------------------------------------------------------
    def joint_axis(self, jspec):
        """判据轴取清单里的**世界系单位向量** axis_world，不取 physics:axis 的字母。

        字母那根是关节系的轴（关节系 = body0 系 ∘ localRot0），跟世界坐标不重合：
        本场景 13 条 v07 关节里有 12 条差得很远（VortexGenie2 摆成 90°，switch 写
        axis=X 实际绕世界 −Y）。判据照字母投影就是打在垂直方向上 —— 大小还在、
        符号作废。换算在 survey_scene_assets.world_axis 里做，理由和实测都在那儿。
        """
        j = self.inv.joints.get(jspec["path"])
        if j is None:
            raise SystemExit("关节 %s 指向 %s，但该路径不在场景清单里"
                             % (jspec.get("cn", "?"), jspec["path"]))
        aw = j.get("axis_world")
        if aw is None:
            raise SystemExit("%s 在清单里没有 axis_world —— 清单是旧版 survey 生成的，"
                             "先重跑 scripts/survey_scene_assets.py" % jspec["path"])
        return aw

    def build_joint_op(self, suite_name, tpl, kind):
        jspec = self.suite["joints"][tpl["joint"]]
        axis = self.joint_axis(jspec)
        state = {"open_lid": "open", "close_lid": "closed", "press": "pressed"}[kind]
        lang = {"open_lid": lang_open, "close_lid": lang_close, "press": lang_press}[kind](jspec)
        self.add(suite_name, tpl.get("_i", 0), lang, [], goal_joint(jspec, state, axis),
                 cn=jspec["cn"])
        return True


def expand(suite_def, inv, seed):
    rng = random.Random(seed)
    b = Builder(suite_def, inv, rng)
    out = []
    for sname, scfg in suite_def["suites"].items():
        used = set()
        made = 0
        tries = 0
        while made < scfg["n"] and tries < scfg["n"] * 12:
            tries += 1
            tpls = scfg["tasks"]
            tpl = tpls[rng.randrange(len(tpls))]
            if "steps" in tpl:
                ok = expand_chain(b, sname, tpl, used)
            elif "varies_to" in tpl:
                ok = b.build_pick_place(
                    sname, dict(tpl, to=rng.choice(tpl["varies_to"]), _soft=True), used)
            elif "varies" in tpl:
                choice = rng.choice(tpl["varies"])
                base = dict(tpl)
                base.pop("varies")
                base.update(choice) if isinstance(choice, dict) else base.update(object=choice)
                ok = b.build_pick_place(sname, dict(base, _soft=True), used)
            elif "op" in tpl:
                if tpl["op"] == "pick_place":
                    ok = b.build_pick_place(sname, tpl, used)
                else:
                    ok = b.build_joint_op(sname, tpl, tpl["op"])
            else:
                raise SystemExit("模板 %r 既没有 op 也没有 steps" % (tpl,))
            made += 1 if ok else 0
        out.extend([t for t in b.tasks if t["suite"] == sname])
    return b, out


def expand_chain(b, sname, tpl, used):
    """长程任务：逐步判据。每步自己抽实例，最后拼成 sequence。

    precondition_joint 必须展开成一个**真的、排在前面判的步骤**，
    不能只当注释留着 —— 否则任务等于要求往关着盖的离心机里插管，
    这条永远不可能成功，测出来的 0% 会被误读成策略差而不是规格错。
    """
    steps, langs, inits = [], [], []
    rng = b.rng
    for st in tpl["steps"]:
        pre = st.get("precondition_joint")
        if pre:
            jspec = b.suite["joints"][pre["joint"]]
            steps.append(goal_joint(jspec, pre.get("state", "open"), b.joint_axis(jspec)))
            langs.append(lang_open(jspec))
        probe = Builder(b.suite, b.inv, rng)
        before = len(probe.tasks)
        if "joint" in st and st["op"] != "pick_place":
            ok = probe.build_joint_op(sname, st, st["op"])
        else:
            ok = probe.build_pick_place(sname, st, used)
        if not ok or len(probe.tasks) <= before:
            return False
        t = probe.tasks[before]
        steps.append(t["goal"])
        langs.append(t["language"])
        inits.extend(t["init"])
    b.add(sname, tpl.get("_i", 0), " → ".join(langs),
          inits, {"type": "sequence", "steps": steps}, cn=tpl["cn"])
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default=os.path.join(PROJ, "tasks", "celllab_suite_v01.yaml"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    with open(args.suite, encoding="utf-8") as f:
        suite_def = yaml.safe_load(f)
    inv_path = os.path.join(PROJ, suite_def["inventory"])
    if not os.path.exists(inv_path):
        raise SystemExit("缺清单 %s —— 先跑 scripts/survey_scene_assets.py" % suite_def["inventory"])
    inv = Inventory(inv_path)

    b, tasks = expand(suite_def, inv, args.seed)

    # 各槽位组的占用率直接打出来：满载的架子是"同架换槽"类任务的硬上限，
    # 不印出来的话只能从"生成不出 10 条"这个间接症状反推。
    occ = b.occupancy()
    print("槽位组占用（空位数决定同架换槽任务能不能生成）：")
    for grp in sorted(b.inv.groups):
        items = b.inv.groups[grp]
        free = sum(1 for a in items if occ.get(a["path"]) is None)
        zmed = sorted(a["pos"][2] for a in items)[len(items) // 2]
        print("   %-62s %3d 槽 / %3d 空  z=%.3f"
              % (grp.replace("/World/TaskArea/", ""), len(items), free, zmed))

    out = args.out or os.path.join(PROJ, "outputs", "task_suite",
                                   "tasks_%s_seed%d.json" % (suite_def["version"], args.seed))
    payload = {
        "version": suite_def["version"],
        "seed": args.seed,
        "base_scene": suite_def["base_scene"],
        "worktop_z": suite_def.get("worktop_z"),
        "n_tasks": len(tasks),
        "by_suite": {},
        "tasks": tasks,
        "blocked_capabilities": suite_def.get("blocked", []),
        "difficulty": suite_def.get("difficulty", {}),
    }
    for t in tasks:
        payload["by_suite"][t["suite"]] = payload["by_suite"].get(t["suite"], 0) + 1

    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=1)

    print("套件 %s  seed=%d  共 %d 条" % (suite_def["version"], args.seed, len(tasks)))
    for k, v in payload["by_suite"].items():
        print("   %-13s %d  （规格要 %d）" % (k, v, suite_def["suites"][k]["n"]))
    print("实测安放高度（对象中心 − 槽位锚点，米）：")
    for (grp, oname), (off, n, src) in sorted(b._rest.items()):
        print("   %-13s → %-46s %+0.4f  %s（%d 样本）"
              % (oname, grp.replace("/World/TaskArea/", ""), off,
                 src or "假设 stand_h/2", n))
    if b.warnings:
        print("警告：")
        for w in sorted(set(b.warnings)):
            print("   -", w)
    print("→", os.path.relpath(out, PROJ).replace("\\", "/"))
    return 0 if all(payload["by_suite"].get(k, 0) >= v["n"]
                    for k, v in suite_def["suites"].items()) else 3


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    sys.exit(main())
