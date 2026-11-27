"""任务判据评测器：只问 PhysX，不读 USD 变换。

为什么位姿必须问 PhysX：
    docs/细胞培养子任务覆盖矩阵.md 第 13 条记过这个坑 —— 早先用 UsdGeom.XformCache
    的世界变换做差，得到 −5964 m、8153 km 这种不可能值，那读的是没回写的 xform。
    任务判据建在那种数上，报出来的成功率是假的。

三个模式：
    reachable  把对象摆到"做完了"的位置再判，判据必须为真。
               为假 = 判据写错或对象根本没进仿真，任务定义本身无效。
    unsolved   什么都不动（场景原样），第一步判据必须为假。
               为真 = 这条任务一开始就成功了，是白送的分数。
    live       对当前场景状态判全部步骤，给以后的策略 rollout 用。

    reachable + unsolved 一起过 = 这套任务规格是well-formed的，可以拿去评策略。
    这两个模式都不需要机器人会动，所以现在就能跑。

关节角度：这些仪器盖不是 articulation 里的关节链，而是两个独立刚体被一个
    joint 连着，所以读不到"关节角"这个数。这里改成读盖这个刚体的四元数，
    跟它在关盖静止姿态下做相对旋转，取绕关节轴的带符号角。能用现成的
    get_rigidbody_transformation 路径，不用赌 articulation view 的 API。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/eval_task_goal.py --mode reachable --gpu-cap
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", default="on", choices=["on", "off"])
parser.add_argument("--mode", default="reachable", choices=["reachable", "unsolved", "live", "policy"])
parser.add_argument("--tasks", default=None)
parser.add_argument("--scene", default=None)
parser.add_argument("--suite", default="v01")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--limit", type=int, default=0)
parser.add_argument("--only", default="",
                    help="只跑 id 或 suite 含这个子串的任务（长程单独验关节时用）")
parser.add_argument("--ids", default="",
                    help="逗号分隔的精确任务 id（分批取数用；给了就不看 --only）")
parser.add_argument("--settle", type=int, default=45,
                    help="摆放或恢复之后等多少帧让物理安定")
parser.add_argument("--gpu-cap", action="store_true")
parser.add_argument("--drive-wait", type=int, default=180,
                    help="长程链中间关节步最多等多少帧，到判据就提前交接（过驱动的滑盖 11 帧走不完）")
parser.add_argument("--out", default=None)
parser.add_argument("--record", default="",
                    help="非空=数据生成模式：把跑过的每条任务落成 LIBERO 式 hdf5（含图像），"
                         "这个值是输出目录。必须开着 RTX 渲染，别和 --no-render 一起用。")
parser.add_argument("--cam", default="global_tube",
                    help="记录用相机 prim 名的子串，场景里有 global_tube/global_pipette/global_centrifuge")
parser.add_argument("--cams", default="",
                    help="逗号分隔的多相机子串（覆盖 --cam）。第一路存 obs/image，"
                         "其余存 obs/image_<key>；任意一路当帧取不到图就整帧跳过，保证对齐。")
parser.add_argument("--rec-stride", type=int, default=5,
                    help="每多少帧存一张图（位置每帧都记）")
parser.add_argument("--transport", type=int, default=0,
                    help=">0=老师在仿真里按帧搬运（kinematic 抬-平移-落），每条取放 N 帧；"
                         "0=老写法 tl.stop() 瞬移（判据基线的口径，样本的 actions 会全是 0）。"
                         "取数据时必须显式给一个正数。")
parser.add_argument("--lift", type=float, default=0.06,
                    help="搬运弧线抬升多少米")
parser.add_argument("--no-render", action="store_true",
                    help="整个关掉 RTX 渲染器（SimulationApp 官方 renderer=None）。"
                         "判据读的是 physx interface 的刚体位姿，不需要出图；"
                         "v09 在台式机 5090D 上开着 RTX 会 Out of resource descriptors 卡死。")
parser.add_argument("--policy", default="replay", choices=["replay", "noop"],
                    help="--mode policy 用：replay=回放 hdf5 轨迹（物体题）；noop=什么都不做")
parser.add_argument("--replay", default=os.path.join("outputs", "demo_data"),
                    help="replay 策略的样本目录")
parser.add_argument("--robot", default="",
                    help="非空=启用一只 UR5e 跟随录制：1/2 或完整 prim 路径")
parser.add_argument("--robot-hover", type=float, default=0.10,
                    help="机械臂跟随物体时腕部高出物体多少米")
parser.add_argument("--robot-grasp", action="store_true",
                    help="手臂当主：走到物体上方、闭合夹爪、物体虚拟附着后带着走")
ARGS = parser.parse_args()
ARGS.headless = ARGS.headless == "on"
REC_CAMS = [s.strip() for s in ARGS.cams.split(",") if s.strip()] or None

_cfg = {"headless": ARGS.headless, "width": 320, "height": 240}
if ARGS.record and ARGS.no_render:
    # 相机取帧就是渲染管线的一路 annotator，关掉渲染器 get_rgba 永远回 None。
    print("!! --record 需要出图，忽略 --no-render")
    ARGS.no_render = False
if ARGS.no_render:
    _cfg["renderer"] = "None"
app = SimulationApp(_cfg)

import carb  # noqa: E402
import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

if ARGS.gpu_cap:
    cs = carb.settings.get_settings()
    for k, v in (("gpuTotalAggregatePairsCapacity", 16384),
                 ("gpuFoundLostAggregatePairsCapacity", 16384),
                 ("gpuMaxRigidContactCountEstimate", 262144),
                 ("gpuMaxPatchCountEstimate", 4194304),
                 ("gpuTensor2GlobalSize", 8388608),
                 ("gpuTensorContactCount", 8388608)):
        cs.set("/persistent/physics/" + k, v)


def raise_contact_capacity(stage, cap=16384):
    """抬 GPU 接触对容量 —— 必须写 **USD 属性**，carb 设置这条路是空的。

    为什么还在报：v01.3 首跑 `--gpu-cap`（carb `/persistent/physics/*`）照旧刷
    `foundLostAggregatePairsCapacity`，说明这些设置在 standalone 构建里对已经建好的
    PhysicsScene 不起作用；实测能落下来的是 probe 那条 —— 给 PhysicsScene prim
    打 `PhysxSceneAPI` 写 `physxScene:gpuTotal*AggregatePairsCapacity`
    （outputs/probe_v09_body0.log 回写 4096，日志里就没再出现该错误）。
    掉接触不是噪声：件会穿过台面、"关得回"变得不可复现，判据等于掷骰子。
    """
    try:
        from pxr import PhysxSchema
    except Exception:
        try:
            import PhysxSchema
        except Exception:
            PhysxSchema = None

    def set_attr(prim, name, value, vtype):
        if PhysxSchema is not None:
            api = PhysxSchema.PhysxSceneAPI.Apply(prim)
            meth = getattr(api, "Create" + name[0].upper() + name[1:] + "Attr", None)
            if meth is not None:
                meth(value)
                return prim.GetAttribute("physxScene:" + name).Get()
        prim.AddAppliedSchema("PhysxSceneAPI")
        a = prim.GetAttribute("physxScene:" + name)
        if not a:
            a = prim.CreateAttribute("physxScene:" + name, vtype)
        a.Set(value)
        return a.Get()

    scenes = [q.GetPath() for q in stage.Traverse() if q.GetTypeName() == "PhysicsScene"]
    if not scenes:
        return "场景里没有 PhysicsScene prim，接触容量抬不了"
    # collisionStackSize 是另一条独立的账：它按"每个接触对的窄相位工作量"分配，
    # 补货资产那 39 个退化盒一改成真碰撞体，开局 settle 阶段就报
    # `PxGpuDynamicsMemoryConfig::collisionStackSize buffer overflow … at least 484842704`
    # —— 幽灵体不产生接触，真盒体一互相穿插就把栈吃穿了。
    sizes = {"gpuTotalAggregatePairsCapacity": cap,
             "gpuFoundLostAggregatePairsCapacity": cap,
             "gpuCollisionStackSize": 536870912}
    got = {}
    for sp in scenes:
        p = stage.GetPrimAtPath(sp)
        got[str(sp)] = {nm: set_attr(p, nm, val, Sdf.ValueTypeNames.UInt)
                        for nm, val in sizes.items()}
    return got


# ---------------------------------------------------------------------------
# PhysX 位姿读取
# ---------------------------------------------------------------------------
class PhysX:
    def __init__(self, iface):
        self.iface = iface
        self.unreadable = set()
        self.quat_form_logged = False

    def pose(self, path):
        """返回 (pos, quat) 或 None。None 必须当成"这个 prim 不是被仿真的刚体"，
        不能当成"它在原点"。"""
        try:
            ret = self.iface.get_rigidbody_transformation(path)
        except Exception:
            self.unreadable.add(path)
            return None
        if not isinstance(ret, dict) or ret.get("ret_val") is False:
            self.unreadable.add(path)
            return None
        p, q = ret.get("position"), ret.get("rotation")
        if p is None:
            self.unreadable.add(path)
            return None
        if not self.quat_form_logged and q is not None:
            # 四元数的存储形式必须先看一眼再解释。上次就是照 (w,x,y,z) 直接解包，
            # 跑到第一个关节判据时 ValueError: not enough values to unpack
            # (expected 4, got 2) —— 说明这个 API 给的不是 4 元组。
            print("rotation 原始返回：type=%s repr=%s" % (type(q).__name__, repr(q)[:160]))
            self.quat_form_logged = True
        return tuple(float(x) for x in p), as_quat(q)


def as_quat(q):
    """把 get_rigidbody_transformation 给的姿态统一成 (w, x, y, z)，认不出就返回 None。

    宁缺毋滥：这里猜错分量顺序，关节角度就会算出一个看着很像真的的错数，
    比直接报 unmeasurable 危害大得多（这条在 docs 里已经栽过一次）。
    """
    if q is None:
        return None
    # carb.Float4 —— 实测 get_rigidbody_transformation 的 rotation 就是这个类型
    # （未旋转给 carb.Float4(0,0,0,1)，即 x/y/z/w 顺序）。字段名齐全，直接按名字取，
    # 顺序确定，不需要再猜。
    if all(hasattr(q, k) for k in "wxyz"):
        return (float(q.w), float(q.x), float(q.y), float(q.z))
    # Gf.Quatd / Gf.Quatf
    if hasattr(q, "GetReal") and hasattr(q, "GetImaginary"):
        im = q.GetImaginary()
        return (float(q.GetReal()), float(im[0]), float(im[1]), float(im[2]))
    try:
        seq = list(q)
    except TypeError:
        return None
    # (real, (x,y,z)) —— boost 把 Gf.Quatd 转成 python 时就是这个二元结构
    if len(seq) == 2 and isinstance(seq[1], (list, tuple)) and len(seq[1]) == 3:
        try:
            return (float(seq[0]),) + tuple(float(v) for v in seq[1])
        except (TypeError, ValueError):
            return None
    if len(seq) == 4 and all(isinstance(v, (int, float)) for v in seq):
        v = [float(x) for x in seq]
        n = math.sqrt(sum(x * x for x in v))
        if n < 1e-9:
            return None
        v = [x / n for x in v]
        # (w,x,y,z) 还是 (x,y,z,w)：静止盖子的四元数必有 |w|≈1，
        # 用"哪个端点更接近 ±1"判，并且只在两者明显分得开时才敢用。
        if abs(abs(v[0]) - abs(v[3])) < 0.05:
            return None
        return (v[3], v[0], v[1], v[2]) if abs(v[3]) > abs(v[0]) else (v[0], v[1], v[2], v[3])
    return None


def axis_swizzle(physx_pos, usd_pos):
    """用一次已知对照把 PhysX 分量的排列确定下来。"""
    cands = {
        "xyz": (0, 1, 2),
        "xzy": (0, 2, 1),
        "yxz": (1, 0, 2),
        "zxy": (2, 0, 1),
    }
    best, berr = None, None
    for name, idx in cands.items():
        err = sum((physx_pos[i] - usd_pos[j]) ** 2 for i, j in zip(idx, range(3))) ** 0.5
        if berr is None or err < berr:
            best, berr = idx, err
    return best, berr


def apply_swizzle(p, idx):
    return (p[idx[0]], p[idx[1]], p[idx[2]])


def quat_axis_angle(q0, q1, axis):
    """两个四元数之间的相对旋转，取绕给定世界轴的带符号分量（度）。"""
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    # qr = q1 * conj(q0)，conj(q0) = (w0, -v0)
    a = (w1, x1, y1, z1)
    b = (w0, -x0, -y0, -z0)
    wr = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3]
    xr = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2]
    yr = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1]
    zr = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
    ang = 2.0 * math.atan2(math.sqrt(max(0.0, xr * xr + yr * yr + zr * zr)), wr)
    ax = (xr, yr, zr)
    n = math.sqrt(sum(v * v for v in ax))
    if n < 1e-9:
        return 0.0
    sign = sum(a * b for a, b in zip(ax, axis)) / n
    return math.degrees(ang) * (1.0 if sign >= 0 else -1.0)


def goal_axis(goal):
    """判据用的**世界系**单位轴向量；读不出来返回 None，绝不猜。

    为什么绝不能猜 (1,0,0)：physics:axis 那个字母是关节系里的轴，关节系
    = body0 系再套 localRot0。2026-11-23 用 scripts/survey_scene_assets.py
    换算过本场景 13 条 v07 关节，**12 条的世界轴和字母不是同一根**
    （VortexGenie2 摆成 90°，switch 写 axis=X 实际绕世界 −Y；
    ThermalCycler 的盖同样是 −Y），只有 A 类那 7 条因为 localRot0 是单位
    四元数、工位没转，才恰好一致。打在垂直方向上的后果是转角大小还在、
    符号变成 0 → 被当成正的，于是"开到 −98.72°"和"翻到 +180°"读起来一样。
    轴由 task_defs 从清单的 axis_world 写进任务里。
    """
    ax = goal.get("axis")
    if ax is None:
        return None
    if isinstance(ax, str):        # 允许手写的任务用字母（只对本场景的 A 类成立）
        ax = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0),
              "Z": (0.0, 0.0, 1.0)}.get(ax.upper())
        if ax is None:
            return None
    n = math.sqrt(sum(float(c) * float(c) for c in ax))
    if n < 1e-9:
        return None
    return tuple(float(c) / n for c in ax)


# ---------------------------------------------------------------------------
# USD 摆放（只用于构造 oracle / 恢复初态）
# ---------------------------------------------------------------------------
class Placer:
    def __init__(self, stage):
        self.stage = stage
        self.saved = {}
        self.saved_drive = {}
        self.saved_kin = {}

    def _world_translate(self, prim):
        xf = UsdGeom.XformCache(Usd.TimeCode.Default())
        return xf.GetLocalToWorldTransform(prim).ExtractTranslation()

    def save(self, path):
        if path in self.saved:
            return
        prim = self.stage.GetPrimAtPath(path)
        if not prim:
            return
        op = prim.GetAttribute("xformOp:translate")
        self.saved[path] = op.Get() if op and op.IsValid() and op.Get() is not None else None

    def place_world(self, path, world_xyz):
        """把刚体挪到指定世界坐标。改的是它自己的 local translate，
        所以嵌套多深都不用手推父级矩阵。"""
        prim = self.stage.GetPrimAtPath(path)
        if not prim:
            return "prim 不存在"
        self.save(path)
        cur = self._world_translate(prim)
        delta = Gf.Vec3d(*world_xyz) - Gf.Vec3d(*cur)
        xf_attr = prim.GetAttribute("xformOp:translate")
        if xf_attr and xf_attr.IsValid() and xf_attr.Get() is not None:
            base = xf_attr.Get()
        else:
            xformable = UsdGeom.Xformable(prim)
            op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            base = op.Get() or Gf.Vec3d(0, 0, 0)
        xf_attr = prim.GetAttribute("xformOp:translate")
        xf_attr.Set(Gf.Vec3d(base[0] + delta[0], base[1] + delta[1], base[2] + delta[2]))
        return None

    def restore(self):
        for path, val in self.saved.items():
            prim = self.stage.GetPrimAtPath(path)
            if not prim:
                continue
            a = prim.GetAttribute("xformOp:translate")
            if not a:
                continue
            if val is None:
                # 原来没有 translate op 的，用 0 还原成"没被挪过"
                a.Set(Gf.Vec3d(0, 0, 0))
            else:
                a.Set(val)
        self.saved.clear()
        # oracle 驱动过的关节目标位也要还回去，否则上一题"开到 -98°"会串到下一题。
        # 键存 (关节路径, 属性名) 而不是拼接串：GetPrimAtPath 给的是 Prim，
        # 上一版直接对它 .Set() 是 AttributeError。
        for (jpath, an), val in self.saved_drive.items():
            prim = self.stage.GetPrimAtPath(jpath)
            a = prim.GetAttribute(an) if prim else None
            if a and a.IsValid():
                a.Set(0.0 if val is None else val)
        self.saved_drive.clear()
        # 搬运期间临时设的 kinematicEnabled 也要还回去，否则这道题之后
        # 这个物体就永远"悬着不掉"了 —— 下一题的静止态判据会读到位姿僵住的物体。
        for path, val in self.saved_kin.items():
            prim = self.stage.GetPrimAtPath(path)
            a = prim.GetAttribute("physics:kinematicEnabled") if prim else None
            if a and a.IsValid():
                a.Set(False if val is None else val)
        self.saved_kin.clear()

    def authored_world(self, path):
        prim = self.stage.GetPrimAtPath(path)
        if not prim:
            return None
        return list(self._world_translate(prim))


# ---------------------------------------------------------------------------
# 判据
# ---------------------------------------------------------------------------
class Judge:
    def __init__(self, physx, axis_idx, rest):
        self.px = physx
        self.axis_idx = axis_idx
        self.rest = rest          # path -> (pos, quat) 在关盖/静止时
        self.reached = set()      # (joint, state)：滚动过程中**曾经**到达过

    def pos(self, path):
        r = self.px.pose(path)
        if not r:
            return None
        return apply_swizzle(r[0], self.axis_idx)

    def quat(self, path):
        r = self.px.pose(path)
        return r[1] if r else None

    def _travel(self, goal, lid, ax):
        """沿世界轴走了多少米（相对静止姿态的平移分量）。"""
        r0 = self.rest.get(lid)
        p0 = r0[0] if isinstance(r0, tuple) and len(r0) == 2 else None
        p = self.pos(lid)
        if p0 is None or p is None:
            return None
        return sum((p[i] - p0[i]) * ax[i] for i in range(3))

    def observe(self, goal):
        """滚动中逐帧问一次：这根关节现在到没到判据要的状态。到了就 latch。"""
        t = goal.get("type")
        if t not in ("joint_angle", "joint_linear"):
            return
        lid = self.lid_of(goal["joint"])
        ax = goal_axis(goal)
        if lid is None or ax is None:
            return
        if t == "joint_angle":
            r0 = self.rest.get(lid)
            r0 = r0[1] if isinstance(r0, tuple) and len(r0) == 2 else r0
            q = self.quat(lid)
            if r0 is None or q is None:
                return
            ang = quat_axis_angle(r0, q, ax)
            # 见 eval() 里同一条注释：角度仍按大小判，符号照实记进结果。
            hit = abs(abs(ang) - abs(goal["expect_deg"])) <= goal["tol_deg"]
        else:
            tr = self._travel(goal, lid, ax)
            if tr is None:
                return
            hit = abs(tr - goal["expect_m"]) <= goal["tol_m"]
        if hit:
            self.reached.add((goal["joint"], goal.get("state")))

    def eval(self, goal, terminal=True):
        """terminal=False 表示这是长程链里的中间步：允许用逐帧 latch 兑现。
        默认 True —— 一条任务单独判成败时，判的就是"此刻是不是这个状态"。"""
        t = goal["type"]
        if t == "in_slot" or t == "in_volume":
            p = self.pos(goal["object"])
            if p is None:
                return {"ok": False, "result": "unmeasurable",
                        "why": "%s 不是 PhysX 里被仿真的刚体" % goal["object"]}
            tgt = goal["target_xyz"]
            dx, dy = abs(p[0] - tgt[0]), abs(p[1] - tgt[1])
            dz = p[2] - goal["z_expect"]
            xy_ok = dx <= goal["xy_tol_m"] and dy <= goal["xy_tol_m"]
            z_ok = abs(dz) <= goal["z_tol_m"]
            return {"ok": bool(xy_ok and z_ok), "result": "measured",
                    "pos": [round(v, 4) for v in p],
                    "dxy": [round(dx, 4), round(dy, 4)], "dz": round(dz, 4),
                    "xy_tol": goal["xy_tol_m"], "z_tol": goal["z_tol_m"]}
        if t == "joint_angle":
            p = self.pos(goal["joint"])
            lid = self.lid_of(goal["joint"])
            if not lid:
                return {"ok": False, "result": "unmeasurable",
                        "why": "关节 %s 找不到被连的盖子刚体" % goal["joint"]}
            ax = goal_axis(goal)
            if ax is None:
                return {"ok": False, "result": "unmeasurable",
                        "why": "任务里没有世界轴 axis —— 重跑 scripts/task_defs.py"
                               "（清单里的 axis_world 由 scripts/survey_scene_assets.py 换算）"}
            q = self.quat(lid)
            # rest[lid] 存的是 (pos, quat) 一对（当年顺手把整个 pose 塞进去了）。
            # 上一版直接把这对二元组当四元数传给 quat_axis_angle，
            # 于是 ValueError: not enough values to unpack (expected 4, got 2)
            # —— got 2 不是"四元数只有两个分量"，是拿错了容器。
            r0 = self.rest.get(lid)
            r0 = r0[1] if isinstance(r0, tuple) and len(r0) == 2 else r0
            if q is None or r0 is None:
                return {"ok": False, "result": "unmeasurable",
                        "why": "盖子 %s 读不到姿态（q=%s rest=%s）"
                               % (lid, bool(q), bool(r0))}
            ang = quat_axis_angle(r0, q, ax)
            # 仍按大小判：v07 那批 YAML 里的期望符号是在"垂直方向投影、符号已经作废"
            # 的旧口径下量的，现在换了世界轴，符号约定得逐条实测重定
            # （scripts/probe_v09_stations.py 补上 axis_world 后重跑一遍）。
            # 大小判据会放过"往反方向翻同样角度"的假完成，所以 signed 照实记出来。
            hit = abs(abs(ang) - abs(goal["expect_deg"])) <= goal["tol_deg"]
            # 长程链里"开盖→…→关盖"两步引用同一根轴，只看终态时永远互相否掉：
            # 关上的那一刻，"开过"这一步必然读成假。所以滚动时逐帧 latch，
            # **中间步**只要曾经到达过就算兑现（YAML 写的就是"逐步判据"）。
            # 但终步绝不能享受这个待遇：任务要求"结束时门是开的"，
            # 门开过又自己掉回 0° 是没做成，用 latch 判就成了 100% 通过的空头判据。
            ever = (goal["joint"], goal.get("state")) in self.reached
            ok = hit if terminal else (hit or ever)
            return {"ok": bool(ok), "result": "measured",
                    "angle_deg": round(ang, 2), "expect_deg": goal["expect_deg"],
                    "tol_deg": goal["tol_deg"], "lid": lid, "axis_world": list(ax),
                    "terminal": bool(terminal),
                    "terminal_hit": bool(hit), "ever_seen": bool(ever),
                    "caveat": "绕轴角由件四元数与世界轴推得；按大小判，符号见 angle_deg，"
                              "未与 articulation 关节读数交叉验证"}
        if t == "joint_linear":
            lid = self.lid_of(goal["joint"])
            ax = goal_axis(goal)
            if not lid:
                return {"ok": False, "result": "unmeasurable",
                        "why": "关节 %s 找不到被连的件刚体" % goal["joint"]}
            if ax is None:
                return {"ok": False, "result": "unmeasurable",
                        "why": "任务里没有世界轴 axis —— 重跑 scripts/task_defs.py"}
            tr = self._travel(goal, lid, ax)
            if tr is None:
                return {"ok": False, "result": "unmeasurable",
                        "why": "件 %s 读不到位置或没有静止基准" % lid}
            # 平移**按带符号**判：expect_m 本身就是带符号行程（sash +0.16、
            # pipette −0.008），世界轴也带符号，两者点积出来的 travel 方向明确，
            # 没有角度那条"符号约定还没重测"的包袱。
            hit = abs(tr - goal["expect_m"]) <= goal["tol_m"]
            ever = (goal["joint"], goal.get("state")) in self.reached
            ok = hit if terminal else (hit or ever)
            return {"ok": bool(ok), "result": "measured",
                    "travel_m": round(tr, 4), "expect_m": goal["expect_m"],
                    "tol_m": goal["tol_m"], "body": lid, "axis_world": list(ax),
                    "terminal": bool(terminal),
                    "terminal_hit": bool(hit), "ever_seen": bool(ever)}
        if t == "sequence":
            last = len(goal["steps"]) - 1
            # 只有最后一步是"任务结束时的状态"，前面的步骤靠逐帧 latch 兑现。
            subs = [self.eval(s, terminal=(i == last)) for i, s in enumerate(goal["steps"])]
            return {"ok": bool(subs[-1]["ok"]) if subs else False,
                    "result": "last_step",
                    "n_steps": len(subs),
                    "all_steps_ok": bool(subs) and all(s["ok"] for s in subs),
                    "first": subs[0] if subs else None,
                    "last": subs[-1] if subs else None,
                    "steps": subs}
        return {"ok": False, "result": "unknown_type", "why": t}

    @staticmethod
    def _joint_body(joint_path, stage, which):
        prim = stage.GetPrimAtPath(joint_path)
        if not prim:
            return None
        rel = prim.GetRelationship("physics:body%s" % which)
        tg = rel.GetTargets() if rel else []
        return str(tg[0]) if tg else None

    def set_stage(self, stage):
        self.stage = stage

    def lid_of(self, joint_path):
        if not hasattr(self, "stage"):
            return None
        return Judge._joint_body(joint_path, self.stage, "1")


CAM_BOX = {}


class Mover:
    """#24 的老师：把"取放"演成一条真轨迹，而不是一次瞬移。

    机制是 `probe_transport_api.py`（tr1.txt）量出来的，不是猜的：
      A 播放中只写 USD xform        → PhysX 只跟了 +25 mm（请求 +80 mm），动态刚体
                                       一边被同步一边被求解器拽回去；
      B 同时 physics:kinematicEnabled=True → +105 mm 全跟，位姿完全听 xform。
    所以搬运 = 短暂把物体变成运动学刚体（等价于"被夹爪握住"），沿抬-平移-落三段
    插值走，走完关掉 kinematic 让物理接手安定。6.1 里 `omni.physx.tensors` 不存在、
    `RigidPrim(prim_paths=…)` 的形参也不是这个名，USD 属性这条路不需要新 API。

    每帧都回调 tick()，所以取数模式（--record）下动作列天然非零。
    """

    def __init__(self, stage, placer, physx, tick):
        self.stage = stage
        self.placer = placer
        self.physx = physx
        self.tick = tick

    def arc(self, path, xyz, frames, lift, release_z=0.02):
        prim = self.stage.GetPrimAtPath(path)
        if not prim:
            return "prim 不存在"
        kin = prim.GetAttribute("physics:kinematicEnabled")
        if not kin or not kin.IsValid():
            return "没有 physics:kinematicEnabled，演不了搬运"
        r = self.physx.pose(path)
        start = list(r[0]) if r else list(self.placer.authored_world(path) or (0, 0, 0))
        if path not in self.placer.saved_kin:
            self.placer.saved_kin[path] = kin.Get()
        kin.Set(True)
        top0 = [start[0], start[1], start[2] + lift]
        top1 = [xyz[0], xyz[1], xyz[2] + lift]
        # 最后一段停在目标上方 release_z，剩下的让重力自己落 —— 直接把运动学体
        # 压进槽位再松手，等于亲手给它一条"从互穿里解算出来"的初速度：
        # spatial_01 这样甩到 [−5795, −4708]、object_00 甩到 [37757, −34552]
        # （rec7/rec8.txt），和第二节"穿透 → 弹飞"是同一个机制，只是这次是我造的。
        end = [xyz[0], xyz[1], xyz[2] + release_z]
        legs = [(start, top0), (top0, top1), (top1, end)]
        per = max(1, int(round(frames / float(len(legs)))))
        for a, b in legs:
            for i in range(1, per + 1):
                t = i / float(per)
                self.placer.place_world(path, [a[k] + (b[k] - a[k]) * t for k in range(3)])
                self.tick()
        kin.Set(False)
        return None

    def arc_arm(self, path, xyz, frames, lift, arm, hover, approach=12):
        """手臂当主：腕部走到物体上方 → 夹爪闭合 → 物体虚拟附着 → 腕部带着走。

        物体仍然是 kinematic 跟随（没有物理 fixed joint），但运动由手臂的 IK 产生，
        动作列（关节/EEF）就是手臂真下发的指令；这比"物体拖着手臂"更接近策略执行。
        """
        prim = self.stage.GetPrimAtPath(path)
        if not prim:
            return "prim 不存在"
        kin = prim.GetAttribute("physics:kinematicEnabled")
        if not kin or not kin.IsValid():
            return "没有 physics:kinematicEnabled"
        r = self.physx.pose(path)
        if not r:
            return "物体读不到位姿"
        start = list(r[0])
        w0 = arm.state()[2]
        grasp = [start[0], start[1], start[2] + hover]
        # 1) 空手靠近物体上方
        for i in range(1, approach + 1):
            t = i / float(approach)
            arm.step([w0[k] + (grasp[k] - w0[k]) * t for k in range(3)], 0.0)
            self.tick()
        # 2) 夹爪闭合
        for _ in range(5):
            arm.step(grasp, 0.9)
            self.tick()
        if path not in self.placer.saved_kin:
            self.placer.saved_kin[path] = kin.Get()
        kin.Set(True)
        w = arm.state()[2]
        r = self.physx.pose(path)
        off = [r[0][k] - w[k] for k in range(3)]
        # 3) 腕部沿抬-平移-落三段走，物体跟着腕部走
        top0 = [grasp[0], grasp[1], grasp[2] + lift]
        top1 = [xyz[0], xyz[1], xyz[2] + hover + lift]
        end = [xyz[0], xyz[1], xyz[2] + hover]
        legs = [(grasp, top0), (top0, top1), (top1, end)]
        per = max(1, int(round(frames / float(len(legs)))))
        for a, b in legs:
            for i in range(1, per + 1):
                t = i / float(per)
                arm.step([a[k] + (b[k] - a[k]) * t for k in range(3)], 0.9)
                self.tick()
                w = arm.state()[2]
                self.placer.place_world(path, [w[k] + off[k] for k in range(3)])
        # 4) 落位：显式收到 oracle 目标（补偿腕部 IK 的剩余误差），再松手。
        #    不这么做时，腕部差几厘米就会把物体压进台面，松手直接穿透掉下去。
        for _ in range(4):
            arm.step(end, 0.0)
            self.placer.place_world(path, list(xyz))
            self.tick()
        kin.Set(False)
        return None


def _oracle_steps(goal):
    """按原文顺序把 goal 摊成 [("object"|"joint", 子步骤), …]。

    老版是"先把所有物体瞬移完、再依次驱动关节"，对判据没影响（只看终态），
    但对轨迹是错的：链式任务"开盖 → 放管 → 关盖"必须按说的顺序演，
    否则样本里出现"盖还关着就把管放进托里"这种物理上不可能的一段。
    """
    out = []
    for sub in _walk_goals(goal):
        kind = "joint" if sub.get("type") in ("joint_angle", "joint_linear") else "object"
        out.append((kind, sub))
    return out


def task_prims(t):
    """任务里出现过的所有 prim 路径，按首次出现排序 —— 观测向量的列顺序就是它。

    不按 schema 逐字段挑（init/goal 的字段名各类型不一样，且长程是列表套列表），
    直接递归抓 `/World/` 开头的字符串。槽位锚点不是刚体，Recorder 里会被剔掉。
    """
    out, seen = [], set()

    def walk(x):
        if isinstance(x, str):
            if x.startswith("/World/") and x not in seen:
                seen.add(x)
                out.append(x)
        elif isinstance(x, dict):
            for v in x.values():
                walk(v)
        elif isinstance(x, (list, tuple)):
            for v in x:
                walk(v)
    walk(t)
    return out


class Recorder:
    """#21 的取数器：判据跑一条任务的同时，把 (语言, 逐帧位姿, 逐帧动作, 图像) 攒下来。

    动作 = 被看物体的**本帧位移**（米）。这是 oracle 当老师时的控制量代理，
    不是关节力矩 —— 现在的老师是"直接把物体摆到位 + 把 drive 目标推到限位"，
    所以管线先通在这一层，动作空间换成真控制量是接策略那一步的事。
    """

    def __init__(self, stage, physx, watch, cam_key, stride, cams=None, robot=None):
        import numpy as np
        self.np = np
        # 不在构造时用 physx.pose 探"谁是刚体"：Recorder 是在 tl.play() 之后、
        # 一次 app.update() 都没跑的时候建的，那一刻 physx interface 还没有任何变换，
        # 全员读不到 → 上一版把 watch 直接筛空，样本一帧没写（rec1.txt）。
        # 改成第一帧到达时再筛。
        self.watch = list(watch)
        self.stride = max(1, stride)
        self.i = 0
        self.pos, self.act, self.rgb = [], [], {}
        self.robot = robot
        self.rob_q, self.rob_qd, self.rob_p, self.rob_quat = [], [], [], []
        self.rob_qt, self.rob_grip = [], []
        self.dropped = []
        self.bad_frames = 0
        self.maxabs = 0.0
        self.prev = None
        self.freeze_after = 5      # 前 5 帧只探列，不落盘：第 0 帧读数是过期的
        self.decided = False
        self.buf = []
        self.cam_key = cam_key
        self.res = (240, 320)
        # 关节 prim 自己没有刚体位姿，PhysX 读不到；换成它的 body1（活动件）。
        # 不换的话 cnt_lid / 盖 / 旋钮这些列每道题都在第 0 帧被整列筛掉。
        mapped = []
        for p in self.watch:
            prim = stage.GetPrimAtPath(p)
            if prim and prim.HasRelationship("physics:body1"):
                tg = prim.GetRelationship("physics:body1").GetTargets()
                if tg:
                    p = str(tg[0])
            if p not in mapped:
                mapped.append(p)
        self.watch = mapped
        self.cam_keys = [c for c in (cams or [cam_key]) if c]
        from pxr import UsdGeom
        all_cams = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdGeom.Camera)]
        from isaacsim.sensors.camera import Camera
        # 6.1 的形参是 prim_path；5.x 文档里的 camera_prim_path 会 TypeError。
        # 相机跨任务复用：每题新建一个 Camera 会多一个 render product，
        # 40 题连跑时这类没释放的东西正是"跑到第 32 题卡死"的嫌疑之一。
        self.cams = []
        for idx, key in enumerate(self.cam_keys):
            hit = [c for c in all_cams if key in c]
            if not hit:
                print("   !! 记录模式找不到含 %r 的相机（场景里：%s）"
                      % (key, ",".join(c.rsplit("/", 1)[-1] for c in all_cams[:8])))
                continue
            path = hit[0]
            cam = CAM_BOX.get(path)
            if cam is None:
                cam = Camera(prim_path=path, name="rec_cam_%d" % idx,
                             resolution=self.res[::-1])
                cam.initialize()
                CAM_BOX[path] = cam
            if idx == 0:
                name = "primary"
            elif "robot1_wrist" in key:
                name = "wrist1"
            elif "robot2_wrist" in key:
                name = "wrist2"
            else:
                name = key.replace("global_", "").replace("_camera", "")[:20]
            self.cams.append((name, cam))
        if not self.cams:
            print("   !! 记录模式一个相机都没找到，样本不会带图")

    def _push(self, row):
        if not row:
            return
        if not all(math.isfinite(float(v)) for v in row):
            # 位姿中途变 NaN 的帧整帧丢掉（宁可少一帧，不要一行毒）。
            self.bad_frames += 1
            return
        self.maxabs = max([self.maxabs] + [abs(float(v)) for v in row])
        if self.prev is not None:
            self.act.append([b - a for a, b in zip(self.prev, row)])
        self.prev = row
        self.pos.append(row)

    def step(self, physx):
        # 逐列读位姿。前 freeze_after 帧只做"这列读得到吗"的投票：
        # 只在第 0 帧筛会把"物理刚起来、读数还没写回来"的列误删。
        vals = [(p, physx.pose(p)) for p in self.watch]
        if not self.decided:
            ok, flat = [], []
            for p, r in vals:
                good = bool(r) and all(
                    math.isfinite(float(v)) and abs(float(v)) < 1e3 for v in r[0])
                ok.append(good)
                flat.extend(list(r[0]) if good else [0.0, 0.0, 0.0])
            self.buf.append((ok, flat))
            if len(self.buf) >= self.freeze_after:
                n = len(self.watch)
                keep = [j for j in range(n) if all(b[0][j] for b in self.buf)]
                self.dropped.extend(self.watch[j] for j in range(n) if j not in keep)
                self.watch = [self.watch[j] for j in keep]
                self.decided = True
                if self.watch:
                    for b_ok, b_flat in self.buf:
                        self._push([b_flat[3 * j + k] for j in keep for k in range(3)])
                else:
                    print("   !! 前 %d 帧没有一列读得稳，样本不写" % self.freeze_after)
                self.buf = []
        else:
            row = []
            for p, r in vals:
                row.extend(list(r[0]) if r else [0.0, 0.0, 0.0])
            self._push(row)
        if self.cams and self.i % self.stride == 0:
            # 多相机同一帧一起取：任意一路没准备好就整帧跳过，保证各路帧号对齐。
            shots = []
            for name, cam in self.cams:
                img = None
                try:
                    img = cam.get_rgba()
                except Exception:
                    img = None
                a = self.np.asarray(img) if img is not None else None
                if a is None or a.ndim != 3 or a.shape[0] != self.res[0]:
                    shots = []
                    break
                shots.append((name, a[..., :3].astype(self.np.uint8).copy()))
            for name, im in shots:
                self.rgb.setdefault(name, []).append(im)
        if self.robot is not None:
            q, qd, p, quat = self.robot.state()
            self.rob_q.append(q)
            self.rob_qd.append(qd)
            self.rob_p.append(p)
            self.rob_quat.append(quat)
            self.rob_qt.append(self.np.asarray(self.robot.q_target,
                                               dtype=self.np.float64).copy())
            self.rob_grip.append(float(self.robot.grip_target))
        self.i += 1

    def save(self, path, meta):
        import h5py
        if not self.pos or not self.watch:
            print("   !! 一帧观测都没攒到（相机或刚体读取没通），%s 不写" % os.path.basename(path))
            return False
        if self.maxabs > 1e3:
            # 甩飞过的轨迹（spatial_01/object_00 第一版就是这么甩到几千米的）
            # 落盘就是投毒：判据会报 BROKEN，但样本文件长得跟正常数据一样。
            print("   !! 轨迹里有 |坐标| > 1e3 m 的甩飞（最大 %.3g m），%s 不写"
                  % (self.maxabs, os.path.basename(path)))
            return False
        os.makedirs(os.path.dirname(path), exist_ok=True)
        np = self.np
        with h5py.File(path, "w") as f:
            d = f.create_group("data")
            act = (np.array(self.act, dtype=np.float32) if self.act
                   else np.zeros((0, 3 * len(self.watch)), dtype=np.float32))
            d.create_dataset("obs/agent_pos", data=np.array(self.pos, dtype=np.float32))
            d.create_dataset("actions", data=act)
            img_keys = []
            for name, frames in self.rgb.items():
                if not frames:
                    continue
                key = "obs/image" if name == "primary" else "obs/image_" + name
                d.create_dataset(key, data=np.stack(frames), compression="gzip")
                img_keys.append(key)
            if self.robot is not None and len(self.rob_p) > 1:
                rob_pos = np.asarray(self.rob_p, dtype=np.float32)
                rob_quat = np.asarray(self.rob_quat, dtype=np.float64)
                d.create_dataset("obs/joint_positions",
                                 data=np.asarray(self.rob_q, dtype=np.float32))
                d.create_dataset("obs/joint_velocities",
                                 data=np.asarray(self.rob_qd, dtype=np.float32))
                d.create_dataset("obs/eef_pos", data=rob_pos)
                d.create_dataset("obs/eef_quat", data=rob_quat.astype(np.float32))
                qa = np.asarray(self.rob_qt, dtype=np.float32)
                grip = np.asarray(self.rob_grip, dtype=np.float32).reshape(-1, 1)
                d.create_dataset("actions_joint",
                                 data=np.concatenate([qa[:, :6], grip], axis=1))
                dp = np.diff(rob_pos, axis=0)
                dr = _quat_delta_axisangle(rob_quat[:-1], rob_quat[1:])
                d.create_dataset("actions_eef",
                                 data=np.concatenate([dp, dr, grip[1:]], axis=1))
                d.attrs["robot_prim"] = self.robot.prim_path
                d.attrs["action_space_eef"] =                     "delta xyz (m), axis-angle (rad), gripper target (rad)"
                d.attrs["action_space_joint"] =                     "6 arm joint targets (rad) + gripper target (rad)"
            d.attrs["obs/agent_pos_names"] = json.dumps(
                [p.rsplit("/", 1)[-1] for p in self.watch], ensure_ascii=False)
            d.attrs["language_instruction"] = meta.get("language", "")
            d.attrs["task_name"] = meta.get("id", "")
            d.attrs["suite_name"] = meta.get("suite", "")
            d.attrs["image_keys"] = json.dumps(img_keys)
            d.attrs["sim_backend"] = "isaacsim " + meta.get("isaac", "?")
            d.attrs["scene"] = meta.get("scene", "")
            d.attrs["success_verdict"] = meta.get("verdict", "")
            d.attrs["oracle"] = "物体瞬移 + drive targetPosition（老师是解析解，不是策略）" \
                if meta.get("transport", 0) <= 0 else \
                "kinematic 按帧搬运（抬-平移-落）+ drive targetPosition（老师是解析解，不是策略）"
            d.attrs["transport_frames"] = int(meta.get("transport", 0))
            d.attrs["dropped_cols"] = json.dumps(self.dropped, ensure_ascii=False)
            d.attrs["dropped_frames"] = int(self.bad_frames)
        n_img = sum(len(v) for v in self.rgb.values()) if self.rgb else 0
        print("   样本 → %s（%d 帧位置 / %d 张图 / %d 路相机 / %d 列物体%s）"
              % (os.path.relpath(path, PROJ).replace("\\", "/"),
                 len(self.pos), n_img, len([v for v in self.rgb.values() if v]),
                 len(self.watch),
                 ("，丢 %d 帧 NaN、%d 列读数不可信" % (self.bad_frames, len(self.dropped)))
                 if (self.bad_frames or self.dropped) else ""))


def _load_replay(stage, t, root, seed):
    """把一条样本的 obs/agent_pos 回放成运动学轨迹：列名 → prim 路径。

    只回放**连杆位姿**：物体题够用；关节链题还要把关节目标也回放，
    否则连杆被 set 回去也会被 drive 拉回（long_horizon_02 实测 fail）。
    """
    import h5py
    seed_tag = "" if seed == 0 else "_seed%d" % seed
    path = os.path.join(PROJ, root, "reachable_%s%s.hdf5" % (t["id"], seed_tag))
    if not os.path.exists(path):
        return None
    with h5py.File(path, "r") as f:
        d = f["data"]
        arr = d["obs/agent_pos"][:]
        names = json.loads(d.attrs["obs/agent_pos_names"])
        rob = d["actions_joint"][:] if "actions_joint" in d else None
    cand = []
    for p in task_prims(t):
        prim = stage.GetPrimAtPath(p)
        if prim and prim.HasRelationship("physics:body1"):
            tg = prim.GetRelationship("physics:body1").GetTargets()
            if tg:
                p = str(tg[0])
        if p not in cand:
            cand.append(p)
    by_base = {}
    for p in cand:
        by_base.setdefault(p.rsplit("/", 1)[-1], []).append(p)
    prims = []
    for n in names:
        lst = by_base.get(n) or []
        if not lst:
            print("   !! 回放列 %s 找不到对应 prim" % n)
            return None
        prims.append(lst.pop(0))
    print("   回放 %s：%d 帧 / %d 列 / 机器人动作 %s"
          % (os.path.basename(path), arr.shape[0], len(prims),
             "有" if rob is not None else "无"))
    return prims, arr, rob


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def _quat_delta_axisangle(q0, q1):
    """wxyz 四元数序列的逐帧世界系旋转增量（轴角，弧度）。"""
    out = np.zeros((len(q0), 3), dtype=np.float64)
    for i, (a, b) in enumerate(zip(q0, q1)):
        a = np.asarray(a, dtype=np.float64)
        b = np.asarray(b, dtype=np.float64)
        a /= max(np.linalg.norm(a), 1e-12)
        b /= max(np.linalg.norm(b), 1e-12)
        r = _quat_mul(b, np.array([a[0], -a[1], -a[2], -a[3]]))
        if r[0] < 0:
            r = -r
        v = r[1:]
        s = float(np.linalg.norm(v))
        if s < 1e-8:
            continue
        ang = 2.0 * math.atan2(s, r[0])
        out[i] = v / s * ang
    return out


def main():
    tpath = ARGS.tasks or os.path.join(
        PROJ, "outputs", "task_suite", "tasks_%s_seed%d.json" % (ARGS.suite, ARGS.seed))
    if not os.path.exists(tpath):
        print("缺任务文件 %s —— 先跑 scripts/task_defs.py" % tpath)
        app.close()
        return 2
    with open(tpath, encoding="utf-8") as f:
        td = json.load(f)

    scene = ARGS.scene or os.path.join(PROJ, td["base_scene"])
    # 这行是防呆：--suite 决定读哪个任务文件，而场景藏在文件的 base_scene 里。
    # 2026-11-24 就吃过亏——--suite v01 拉的是 v07 的老任务集，日志里一声不吭，
    # 对着一个根本没打开的场景判读了一轮。
    print("任务文件=%s  suite=%s  seed=%d  →  场景=%s"
          % (os.path.basename(tpath), ARGS.suite, ARGS.seed,
             os.path.relpath(scene, PROJ).replace(os.sep, "/")))
    if ARGS.scene and os.path.abspath(scene) != os.path.abspath(
            os.path.join(PROJ, td["base_scene"])):
        print("  注意：--scene 盖掉了任务集自带的 %s" % td["base_scene"])
    ctx = omni.usd.get_context()
    if not ctx.open_stage(scene):
        print("打不开场景", scene)
        app.close()
        return 2
    stage = ctx.get_stage()
    tl = omni.timeline.get_timeline_interface()
    physx = PhysX(omni.physx.get_physx_interface())
    placer = Placer(stage)
    j = Judge(physx, (0, 1, 2), {})
    j.set_stage(stage)
    if ARGS.gpu_cap:
        print("接触容量抬升（USD 属性通道）：", raise_contact_capacity(stage))

    tasks = td["tasks"]
    sel_tag = ""
    if ARGS.ids:
        want = [s.strip() for s in ARGS.ids.split(",") if s.strip()]
        known = {t["id"] for t in td["tasks"]}
        missing = [w for w in want if w not in known]
        if missing:
            print("!! --ids 里有任务文件不认识的 id：%s" % ",".join(missing))
            app.close()
            return 2
        wanted = set(want)
        tasks = [t for t in td["tasks"] if t["id"] in wanted]
        sel_tag = "_" + ARGS.ids.replace(",", "-")[:60].replace(os.sep, "_")
        print("--ids 选中 %d/%d 题：%s" % (
            len(tasks), len(td["tasks"]), ",".join(t["id"] for t in tasks)[:200]))
    elif ARGS.only:
        # 第三条件（goal 正文含这个子串）是给"只跑涉及某个工位/物体的题"留的后门，
        # 但它让 --only object 把 long_horizon 也捞进来了（goal 里全是
        # /World/TaskArea/Objects/…）。所以选中多少题必须打出来，别默默跑完 40 题
        # 还以为是 10 题的量。
        tasks = [t for t in tasks
                 if ARGS.only in t["id"] or ARGS.only in t["suite"]
                 or ARGS.only in json.dumps(t["goal"])]
        sel_tag = "_" + ARGS.only.replace(os.sep, "_")
        print("--only=%s 选中 %d/%d 题：%s" % (
            ARGS.only, len(tasks), len(td["tasks"]),
            ",".join(t["id"] for t in tasks)[:200]))
    if ARGS.limit:
        tasks = tasks[:ARGS.limit]

    # --- 启动一次，确认场景能进仿真 ---
    tl.stop()
    tl.play()
    for _ in range(12):
        app.update()

    # 分量排列：拿一个任务对象的 PhysX 读数和 USD 作者位姿比一次定下来。
    probe = None
    for t in tasks:
        if t["init"]:
            probe = t["init"][0]
            break
    if probe:
        r = physx.pose(probe["prim"])
        if r:
            idx, err = axis_swizzle(r[0], probe["authored_pos"])
            j.axis_idx = idx
            print("PhysX 分量排列判为 %s（与 USD 作者位姿差 %.4f m）"
                  % ("".join("xyz"[i] for i in idx), err))
            if err > 0.05:
                print("!! 差得超过 5 cm：这个对象可能被场景里别的东西挤开过，"
                      "或者作者位姿本身就不等于仿真起始位姿。判据阈值先别信。")
        else:
            print("!! 探针对象 %s 在 PhysX 里读不到 —— 后面的判据会大面积 unmeasurable"
                  % probe["prim"])

    # 记录所有活动件的静止姿态（关盖/缩回），关节判据以此为基准。
    # joint_linear 也要记：平移判据量的是"沿世界轴走了多少"，
    # 上一版只给 joint_angle 存 rest，两条 sash/pipette 的 linear 任务
    # 根本没有零点可比。
    seen_joints = set()
    miss = []
    for t in tasks:
        for g in _walk_goals(t["goal"]):
            if g["type"] not in ("joint_angle", "joint_linear"):
                continue
            seen_joints.add(g["joint"])
            lid = Judge._joint_body(g["joint"], stage, "1")
            if not lid:
                miss.append("%s 关节没有 body1" % g["joint"])
                continue
            if lid in j.rest:
                continue
            r = physx.pose(lid)
            if not r:
                miss.append("盖子 %s 在 PhysX 里读不到（不是被仿真的独立刚体？）" % lid)
                continue
            if not r[1]:
                miss.append("盖子 %s 只读到位姿读不到姿态" % lid)
                continue
            j.rest[lid] = (apply_swizzle(r[0], j.axis_idx), r[1])
    print("已存静止姿态的盖子：%d / %d 个关节" % (len(j.rest), len(seen_joints)))
    for m in miss:
        print("   !! " + m)

    ARM = None
    ARM_PATH = None
    ARM_OBJ = {"path": None, "grip": 0.0}
    if ARGS.robot:
        import _robot_ik
        rp = ARGS.robot
        if rp in ("1", "2"):
            rp = "/World/Robot/Robot%dMount/ur5e" % int(rp)
        ARM_PATH = rp
        ARM = _robot_ik.ArmFollower(rp)
        print("机械臂跟随：%s（hover %.3f m）" % (rp, ARGS.robot_hover))

    rows = []
    REC = None

    def tick():
        """所有需要出样本的滚动都走这里：推进一帧 + 顺手记观测。"""
        if ARM is not None and ARM_OBJ["path"]:
            r = physx.pose(ARM_OBJ["path"])
            if r:
                ARM.step([r[0][0], r[0][1], r[0][2] + ARGS.robot_hover],
                         ARM_OBJ["grip"])
        app.update()
        if REC is not None:
            REC.step(physx)

    MOVER = Mover(stage, placer, physx, tick)

    for t in tasks:
        goal = t["goal"]
        # latch 是"这一题滚动过程中到过什么"，不是"这台场景到过什么"。
        # 上一版跨题不清空：第 1 题把离心盖开到 -98° 之后，第 2 题的
        # "开过盖"什么都不做就兑现了 —— reachable 通过率里混进了这种白捡的。
        j.reached.clear()
        REC = None
        if ARGS.mode == "reachable":
            placer.restore()
            tl.stop()
            for _ in range(6):
                app.update()
            if ARM is not None:
                ARM.reset()
            steps = _oracle_steps(goal)
            if ARGS.transport <= 0:
                # 老口径（判据 40 题基线就是这么来的）：所有取放在仿真外瞬移。
                # 轨迹样本必须给 --transport > 0，否则 actions 全 0（6L 那两条的毛病）。
                errs = _oracle_place(placer, goal)
                if errs:
                    print("   oracle 摆不动：", "; ".join(errs))
            tl.play()
            if ARM is not None:
                for _ in range(3):
                    app.update()
                # tl.stop() 会让 articulation 的物理视图失效（链路位姿永远停在旧值），
                # 必须在 play 之后重建视图，否则 IK 是在追一个不存在的位姿。
                ARM = _robot_ik.ArmFollower(ARM_PATH)
            if ARGS.record:
                REC = Recorder(stage, physx, task_prims(t), ARGS.cam, ARGS.rec_stride,
                               cams=REC_CAMS, robot=ARM)
            for i, (kind, sg) in enumerate(steps):
                last = (i == len(steps) - 1)
                if ARM is not None:
                    ARM_OBJ["path"] = (None if ARGS.robot_grasp else
                                       (sg.get("object") if kind == "object" else None))
                    ARM_OBJ["grip"] = 0.9 if kind == "object" else 0.0
                if kind == "object":
                    if ARGS.transport > 0 and sg.get("oracle_pose_xyz"):
                        if ARM is not None and ARGS.robot_grasp:
                            e = MOVER.arc_arm(sg["object"], sg["oracle_pose_xyz"],
                                              ARGS.transport, ARGS.lift, ARM,
                                              ARGS.robot_hover)
                        else:
                            e = MOVER.arc(sg["object"], sg["oracle_pose_xyz"],
                                          ARGS.transport, ARGS.lift)
                        if e:
                            # 不偷偷退回瞬移：演不动就让它演不动，判据老实话报 BROKEN，
                            # 比混进一条"看着是轨迹其实是跳变"的样本干净。
                            print("   oracle 搬不动：", sg["object"], e)
                        if ARM is not None:
                            # 搬完松手：后面的 settle 帧夹爪是开的，grip 动作才有变化
                            ARM_OBJ["grip"] = 0.0
                else:
                    # 逐条按顺序驱动：一条链里"开盖…关盖"引用同一根轴，
                    # 一次性把目标位设成最后那个值，中间那步在物理上从没发生过，
                    # latch 就永远是假 —— 那不是任务做不成，是 oracle 没演完。
                    e = _oracle_drive(placer, sg)
                    if e:
                        print("   oracle 驱动失败：", sg.get("joint"), e)
                # 中间关节步"到判据就交接，没到最多等 --drive-wait 帧"。为什么固定 11 帧
                # 不行：sash 的 drive k=200 c=40 m=0.8 → ζ≈1.6 是过驱动，11 帧只走到 +0.12
                # （期望 +0.16），判据读成"没开到位"——那不是场景不行，是 oracle 没演完
                # （probe 同一条给 90 帧就到限位，probe_v09_body0.json）。
                n = ARGS.settle if last else (ARGS.drive_wait if kind == "joint" else 6)
                for _ in range(n):
                    tick()
                    j.observe(sg)
                    if (not last and kind == "joint"
                            and (sg["joint"], sg.get("state")) in j.reached):
                        break
        elif ARGS.mode == "policy":
            placer.restore()
            tl.stop()
            for _ in range(6):
                app.update()
            if ARM is not None:
                ARM.reset()
            tl.play()
            traj = None
            if ARGS.policy == "replay":
                traj = _load_replay(stage, t, ARGS.replay, ARGS.seed)
            if traj is not None:
                prims, arr, rob = traj
                if rob is not None and ARM is not None:
                    for _ in range(3):
                        app.update()
                    ARM = _robot_ik.ArmFollower(ARM_PATH)
                saved = {}
                for path in prims:
                    prim = stage.GetPrimAtPath(path)
                    a = prim.GetAttribute("physics:kinematicEnabled") if prim else None
                    if a and a.IsValid():
                        saved[path] = a.Get()
                        a.Set(True)
                off = None
                attached = False
                if rob is not None and ARM is not None:
                    kin0 = stage.GetPrimAtPath(prims[0]).GetAttribute("physics:kinematicEnabled")
                for k in range(arr.shape[0]):
                    if rob is not None and ARM is not None:
                        ARM.apply_action(rob[k])
                        if not attached and rob[k, 6] > 0.5:
                            w = ARM.state()[2]
                            off = arr[k, 0:3] - w
                            attached = True
                            if kin0 and kin0.IsValid():
                                saved[prims[0]] = kin0.Get()
                                kin0.Set(True)
                    for col, path in enumerate(prims):
                        if attached and col == 0:
                            w = ARM.state()[2]
                            placer.place_world(path, [float(w[i] + off[i]) for i in range(3)])
                        else:
                            placer.place_world(path,
                                               [float(v) for v in arr[k, 3 * col:3 * col + 3]])
                    tick()
                    for sg in _walk_goals(goal):
                        j.observe(sg)
                for path, v in saved.items():
                    prim = stage.GetPrimAtPath(path)
                    if prim:
                        prim.GetAttribute("physics:kinematicEnabled").Set(v)
            for _ in range(ARGS.settle):
                tick()
                for sg in _walk_goals(goal):
                    j.observe(sg)
        elif ARGS.mode == "unsolved":
            placer.restore()
            tl.stop()
            for _ in range(6):
                app.update()
            if ARM is not None:
                ARM.reset()
            tl.play()
            if ARM is not None:
                for _ in range(3):
                    app.update()
                # tl.stop() 会让 articulation 的物理视图失效（链路位姿永远停在旧值），
                # 必须在 play 之后重建视图，否则 IK 是在追一个不存在的位姿。
                ARM = _robot_ik.ArmFollower(ARM_PATH)
            if ARGS.record:
                REC = Recorder(stage, physx, task_prims(t), ARGS.cam, ARGS.rec_stride,
                               cams=REC_CAMS, robot=ARM)
            for _ in range(ARGS.settle):
                tick()
        res = j.eval(goal)
        if ARGS.mode == "reachable":
            want = True
            if res["result"] == "last_step":
                # 链式任务不能只看末步：上一版把 result="last_step" 当成
                # "不是 measured"，于是 10 道长程题一个判读都没产出。
                subs = res.get("steps") or []
                check = bool(subs) and all(s["result"] == "measured" and s["ok"] for s in subs)
                verdict = "well-formed" if check else (
                    "BROKEN" if subs and all(s["result"] == "measured" for s in subs)
                    else "unmeasurable-step")
                res["n_unmeasured"] = sum(1 for s in subs if s["result"] != "measured")
            else:
                check = res["result"] == "measured" and res["ok"] is True
                verdict = "well-formed" if check else ("BROKEN" if res["result"] == "measured" else res["result"])
        elif ARGS.mode == "unsolved":
            first = res.get("first") or res
            want = False
            check = first["result"] == "measured" and first["ok"] is False
            verdict = "well-formed" if check else ("TRIVIALLY-ALREADY-DONE"
                                                   if first["result"] == "measured" else first["result"])
        elif ARGS.mode == "policy":
            if res["result"] == "last_step":
                subs = res.get("steps") or []
                check = bool(subs) and all(s["result"] == "measured" and s["ok"] for s in subs)
            else:
                check = res["result"] == "measured" and res["ok"] is True
            verdict = "success" if check else "fail"
        else:
            verdict = "ok" if res["ok"] else "fail"
        rows.append({"id": t["id"], "suite": t["suite"], "language": t["language"],
                     "verdict": verdict, "detail": res})
        print("%-18s %-14s %-22s %s" % (t["id"], t["suite"], verdict,
                                        json.dumps(res, ensure_ascii=False)[:150]))
        if REC is not None:
            if not REC.watch:
                print("   !! 这道题涉及的 prim 在 PhysX 里一个都读不到，样本不写")
            else:
                # seed 0 沿用旧文件名（已有 26 条样本不搬家）；其他 seed 加后缀防覆盖。
                seed_tag = "" if ARGS.seed == 0 else "_seed%d" % ARGS.seed
                REC.save(os.path.join(ARGS.record,
                                      "%s_%s%s.hdf5" % (ARGS.mode, t["id"], seed_tag)),
                         {"id": t["id"], "suite": t["suite"], "language": t["language"],
                          "verdict": verdict, "scene": os.path.basename(scene),
                          "transport": ARGS.transport})

    placer.restore()
    tl.stop()

    tally = {}
    for r in rows:
        tally[r["verdict"]] = tally.get(r["verdict"], 0) + 1
    out = ARGS.out or os.path.join(
        PROJ, "outputs", "task_suite",
        # 分块跑（--only）时四块写的是同一个文件名，后一块把前一块整个盖掉，
        # 40 题的合并表就只能在日志里拼。名字里带上 only 的标签，一块一份。
        "eval_%s_%s_seed%d%s.json" % (ARGS.mode, ARGS.suite, ARGS.seed, sel_tag))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"mode": ARGS.mode, "scene": os.path.basename(scene),
                   "n": len(rows), "tally": tally,
                   "unreadable": sorted(physx.unreadable)[:60],
                   "rows": rows}, f, ensure_ascii=False, indent=1)
    print()
    print("模式 %s  判读汇总：%s" % (ARGS.mode, json.dumps(tally, ensure_ascii=False)))
    print("→", os.path.relpath(out, PROJ).replace("\\", "/"))
    app.close()
    return 0


def _oracle_drive(placer, goal):
    """关节类步骤的 oracle：把 drive 目标位直接设成判据期望值。

    没有这一步，reachable 模式对 7 条关节谓据永远报假 —— 因为它只搬物体、
    从来不动关节，"摆到做完了的位置"这个前提对盖子根本不成立。
    角位移都用 USD 里已存的带符号值：YAML 的 open_deg 就是按限位方向写的
    （5430 盖 lower=-98.72 upper=0，开盖就是负方向），不再自己判符号。

    目标位存在 Placer 里跟着 restore 一起还原：上一题把 5430 盖开到 -98°，
    下一题的判据不涉及这个盖，但它物理上还开着 —— 脏状态会串题。
    """
    prim = placer.stage.GetPrimAtPath(goal["joint"])
    if not prim:
        return "关节 prim 不存在"
    if "expect_m" in goal:
        attr, val = "drive:linear:physics:targetPosition", goal["expect_m"]
    else:
        attr, val = "drive:angular:physics:targetPosition", goal["expect_deg"]
    a = prim.GetAttribute(attr)
    if not a or not a.IsValid():
        return "关节 %s 没有 %s，drive 没建起来" % (goal["joint"], attr)
    if not math.isfinite(float(val)):
        # 实测代价（drive1.txt）：往 targetPosition 写 inf 会刷满 1000 条 PhysX error，
        # 接着 "Cuda context manager error, simulation will be stopped"，整轮进程报废。
        # 任务文件的期望值现在是有限的，这条只挡生成侧将来塞进 ±inf 限位的情况。
        return "关节 %s 的期望值 %r 不是有限数，拒绝驱动" % (goal["joint"], val)
    key = (goal["joint"], attr)
    if key not in placer.saved_drive:
        placer.saved_drive[key] = a.Get()
    a.Set(float(val))
    return None


def _walk_goals(g):
    if g["type"] == "sequence":
        for s in g["steps"]:
            yield s
    else:
        yield g


def _oracle_place(placer, goal):
    """把 in_slot/in_volume 的对象挪到 oracle 位姿。返回错误列表。

    只搬物体：物体必须在 tl.stop() 时挪（播放中 PhysX 会回写 xform，边播边挪
    等于没挪），而关节驱动器要在 tl.play() 之后才生效，两者时序相反，
    所以关节另走 _oracle_joint_steps + _oracle_drive。
    """
    errs = []
    for sub in _walk_goals(goal):
        if sub.get("oracle_pose_xyz"):
            e = placer.place_world(sub["object"], sub["oracle_pose_xyz"])
            if e:
                errs.append("%s: %s" % (sub["object"], e))
    return errs


def _oracle_joint_steps(goal):
    return [s for s in _walk_goals(goal)
            if s.get("type") in ("joint_angle", "joint_linear")]


if __name__ == "__main__":
    sys.exit(main())
