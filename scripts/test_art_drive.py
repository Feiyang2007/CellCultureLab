"""关节驱动通道试车：运行期写 drive:angular:physics:targetPosition，盖子到底动不动。

为什么要单独试（#19）：
    判据脚本对关节类谓据的 oracle 就是往这个属性上写期望值
    （eval_task_goal.py 的 _oracle_drive），而 A 批之前这批关节的 drive 刚度全是 0，
    写进去也没有力。A 批把 stiffness/damping 补上之后，"运行期写 USD 属性 → PhysX
    真转起来" 这条链路还没被直接验过。
    数据生成闭环（#21）全靠它：oracle 演示得是真动出来的，不能是贴过去的。

怎么算通过：
    对每个工位的每个非焊点关节，把目标位推到"离当前目标最远的那个限位"，跑几秒，
    读关节端刚体（physics:body1）的 PhysX 位姿。转角 > 5° 或平移 > 0.01 m 即通。
    同时把 drive 的 stiffness/damping/maxForce 一并打出来 —— 真不动的时候要先分清
    是"没刚度"还是"属性写不进"，这两条修法完全不同。

跑法（台式机，纯物理约 30 s）：
    C:\\Users\\73\\IsaacLab\\.venv\\Scripts\\python.exe -u scripts\\test_art_drive.py
    ... --station CentriMini        # 只看某一个工位
"""
import argparse
import math
import os
import sys
import time

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--station", default="all")
parser.add_argument("--seconds", type=float, default=5.0)
ARGS = parser.parse_args()

app = SimulationApp({"headless": True, "renderer": "None", "width": 320, "height": 240})

import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdPhysics  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda")
STATIONS = ["CentriMini", "ThermalCycler", "VortexGenie2"]
if ARGS.station != "all":
    STATIONS = [ARGS.station]


def pose_of(iface, path):
    """端刚体的 (平移, 四元数列表)；读不到返回 None（= 它不是被仿真的刚体）。"""
    r = iface.get_rigidbody_transformation(path)
    if not isinstance(r, dict) or r.get("ret_val") is False or r.get("position") is None:
        return None
    q = r.get("rotation")
    seq = None
    if q is not None:
        if hasattr(q, "GetReal"):
            seq = [float(q.GetReal())] + [float(x) for x in q.GetImaginary()]
        else:
            try:
                seq = [float(x) for x in q]
            except (TypeError, ValueError):
                seq = None
    return tuple(float(x) for x in r["position"]), seq


def quat_deg(q0, q1):
    """两个四元数之间的夹角（度）。口径 (w,x,y,z) 还是 (x,y,z,w) 不影响点积，
    所以这里不用先判排列 —— 这点在判据脚本里曾经踩过 unpack 的坑。"""
    if not q0 or not q1 or len(q0) != 4 or len(q1) != 4:
        return None
    n0 = math.sqrt(sum(x * x for x in q0))
    n1 = math.sqrt(sum(x * x for x in q1))
    if n0 == 0 or n1 == 0:
        return None
    d = sum(a * b for a, b in zip(q0, q1)) / (n0 * n1)
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, abs(d)))))


def drive_target(joint):
    """返回 (属性名, 目标值, 说明)：把目标推到离当前目标最远的限位。"""
    lin = joint.IsA(UsdPhysics.PrismaticJoint)
    attr = "drive:linear:physics:targetPosition" if lin else "drive:angular:physics:targetPosition"
    a = joint.GetAttribute(attr)
    lo = joint.GetAttribute("physics:lowerLimit")
    hi = joint.GetAttribute("physics:upperLimit")
    if not a or not a.IsValid() or not lo or not hi:
        return None
    lo_v, hi_v = lo.Get(), hi.Get()
    # 限位必须有作者值：UsdPhysics 的默认限位是 ±FLT_MAX，拿它当目标位等于给关节
    # 下一条 3e38 的指令，PhysX 会把这一树直接算崩。
    if lo_v is None or hi_v is None or not lo.HasValue() or not hi.HasValue():
        return None
    if not (math.isfinite(float(lo_v)) and math.isfinite(float(hi_v))):
        # VortexGenie2/platform_pivot 的作者限位就是 ±inf（MJCF 没写 range）。
        # 实测：给它写 targetPosition=inf 之后 1000 条 PhysX error 刷满、
        # CUDA context 被重建、整轮直接死（drive1.txt）。无限位 = 无法定义"最远"。
        return None
    cur = a.Get()
    cands = [(attr, float(hi_v), "upperLimit"), (attr, float(lo_v), "lowerLimit")]
    if cur is not None:
        cands.sort(key=lambda t: -abs(t[1] - float(cur)))
    return cands[0]


def main():
    ctx = omni.usd.get_context()
    if not ctx.open_stage(SCENE):
        print("打不开场景", SCENE)
        app.close()
        return 2
    stage = ctx.get_stage()
    iface = omni.physx.get_physx_interface()
    tl = omni.timeline.get_timeline_interface()
    tl.stop()
    tl.play()
    # 先让站点落稳：焊点修好之后站点是撑在台面上的，settle 一两秒再量，
    # 不然初始位姿本身就带 1 cm 级别的下沉，判不出驱动位移。
    for _ in range(60):
        app.update()
    time.sleep(1.5)
    for _ in range(60):
        app.update()

    plan = []
    for s in STATIONS:
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/TaskArea/Stations/" + s)):
            if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
                continue
            if prim.GetName().startswith("rootJoint"):
                continue
            rel = prim.GetRelationship("physics:body1")
            body = str(rel.GetTargets()[0]) if rel and rel.GetTargets() else None
            tgt = drive_target(prim)
            kind = "prismatic" if prim.IsA(UsdPhysics.PrismaticJoint) else "revolute"
            print("%-14s %-24s %-9s 端刚体=%-14s 刚度=%-8s 阻尼=%-6s 最大力=%-8s 限位=%s~%s %s" % (
                s, prim.GetName(), kind, (body or "-").split("/")[-1],
                _v(prim, "drive:angular:physics:stiffness") or _v(prim, "drive:linear:physics:stiffness"),
                _v(prim, "drive:angular:physics:damping") or _v(prim, "drive:linear:physics:damping"),
                _v(prim, "drive:angular:physics:maxForce") or _v(prim, "drive:linear:physics:maxForce"),
                _v(prim, "physics:lowerLimit"), _v(prim, "physics:upperLimit"),
                "目标=%s→%.4f" % (tgt[2], tgt[1]) if tgt else "写不了目标（属性或限位缺）"))
            if body and tgt:
                plan.append((s, prim, tgt, body))

    print("\n可驱动的关节 %d 个：写目标位，再跑 %.0f s" % (len(plan), ARGS.seconds))
    before, written = {}, []
    for s, prim, (attr, val, why), body in plan:
        try:
            prim.GetAttribute(attr).Set(float(val))
        except Exception as exc:
            print("  写不进 %s：%s" % (prim.GetPath(), str(exc).encode("ascii", "backslashreplace").decode("ascii")))
            continue
        written.append((s, prim, attr, body))
        before[body] = pose_of(iface, body)
    t0 = time.time()
    while time.time() - t0 < ARGS.seconds:
        app.update()

    moved = 0
    for s, prim, attr, body in written:
        p0, p1 = before.get(body), pose_of(iface, body)
        if not p0 or not p1:
            print("  %-14s %-24s 端刚体位姿读不到（%s -> %s）" % (
                s, prim.GetName(), bool(p0), bool(p1)))
            continue
        d = (Gf.Vec3d(*p1[0]) - Gf.Vec3d(*p0[0])).GetLength()
        ang = quat_deg(p0[1], p1[1])
        ok = d > 0.01 or (ang is not None and ang > 5.0)
        moved += 1 if ok else 0
        print("  %-14s %-24s 平移 %.4f m  转角 %-9s %s" % (
            s, prim.GetName(), d, "-" if ang is None else "%.2f deg" % ang,
            "动了" if ok else "没动"))
    print("\n汇总：%d/%d 个关节在运行期写 targetPosition 之后真的动了"
          % (moved, len(written)))
    tl.stop()
    app.close()
    return 0


def _v(prim, name):
    a = prim.GetAttribute(name)
    return a.Get() if a and a.HasValue() else "-"


if __name__ == "__main__":
    sys.exit(main())
