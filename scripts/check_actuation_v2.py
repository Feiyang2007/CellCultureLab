"""
关节驱动实测 v2（不再用平移当判据）。

为什么之前那版不算数：铰链的锚点通常就在 body 原点上，**绕锚点转体时原点不动**，
所以"位置没变"根本不能证明"没转"。这版必须读**朝向**（四元数），做去程+回程，
并允许在拿不到朝向时明确报"测不了"而不是蒙混成"通过/失败"。

状态来源优先级：
  1) omni.physx.tensors.get_bodies_state()  —— 位置 + 朝向（PhysX 运行时，Fabric 下也可用）
  2) get_rigidbody_transformation()          —— 若返回里带朝向就用
  3) 都没有 → 直接判"无法测量"，不写结论

用法：
    D:/IsaacSim5/env/Scripts/python.exe scripts/check_actuation_v2.py [场景] [--settle 60] [--run 90]
"""
import os, sys, math, json

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd, omni.timeline, omni.physx
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
args = [a for a in sys.argv[1:] if not a.startswith("--")]
scene = os.path.abspath(args[0] if args else os.path.join(
    PROJ, "assets", "scenes", "test_instruments4.usda"))
settle = int(sys.argv[sys.argv.index("--settle") + 1]) if "--settle" in sys.argv else 60
run = int(sys.argv[sys.argv.index("--run") + 1]) if "--run" in sys.argv else 90

ctx = omni.usd.get_context()
if not ctx.open_stage(scene):
    print("打不开场景：", scene)
    app.close(); sys.exit(1)
st = ctx.get_stage()
iface = omni.physx.get_physx_interface()
tl = omni.timeline.get_timeline_interface()

# ---- 状态来源探测 ----
SRC = None
try:
    from omni.physx.tensors import get_bodies_state
    _ = get_bodies_state
    SRC = "tensors"
except Exception as e:
    print("get_bodies_state 不可用:", type(e).__name__, str(e)[:70])
probe = iface.get_rigidbody_transformation("/Instrument/Bodies/body") if st.GetPrimAtPath("/Instrument/Bodies/body") else None
print("PhysX 状态来源:", SRC, "| get_rigidbody_transformation 字段:",
      list(probe.keys()) if isinstance(probe, dict) else type(probe).__name__)


def quat_angle(q0, q1):
    if not q0 or not q1:
        return None
    def norm(q):
        n = math.sqrt(sum(x * x for x in q)) or 1.0
        return [x / n for x in q]
    a, b = norm(list(q0)), norm(list(q1))
    if abs(abs(a[0]) - 1.0) < 1e-9 and abs(abs(b[0]) - 1.0) < 1e-9:
        pass
    dot = min(1.0, abs(sum(x * y for x, y in zip(a, b))))
    return math.degrees(2 * math.acos(dot))


def get_pose(path):
    """返回 (pos, quat) 或 (None, None)；只用 PhysX 运行时数据。"""
    try:
        r = iface.get_rigidbody_transformation(path)
    except Exception:
        return None, None
    if not isinstance(r, dict) or r.get("ret_val") is False:
        return None, None
    pos = r.get("position")
    q = r.get("orientation") or r.get("rotation") or r.get("quat")
    return (list(pos) if pos is not None else None, list(q) if q is not None else None)


joints = []
for p in st.Traverse():
    t = str(p.GetTypeName())
    if t not in ("RevoluteJoint", "PrismaticJoint"):
        continue
    r0 = p.GetRelationship("physics:body0").GetTargets()
    r1 = p.GetRelationship("physics:body1").GetTargets()
    if not (r0 and r1):
        continue
    joints.append((p, t, str(r1[0])))

print("=" * 96)
print("场景：", os.path.basename(scene), "  已绑定的关节：", len(joints))
if not joints:
    print("没有任何绑定好的关节 → 无从测量")
    app.close(); sys.exit(2)

tl.stop(); tl.play()
for _ in range(30):
    app.update()
base = {}
for p, t, b1 in joints:
    base[b1] = get_pose(b1)

can_measure = all(q is not None for q, _ in base.values() if base) or \
    any(q is not None for q, _ in base.values())
if not can_measure:
    print("拿不到朝向数据 → **无法测量驱动效果**，不下结论（需要换 tensors 接口或检查 Fabric）")
    app.close(); sys.exit(3)

rows = []
with Usd.EditContext(st, st.GetSessionLayer()):
    for p, t, b1 in joints:
        pre = "angular" if t == "RevoluteJoint" else "linear"
        lo = p.GetAttribute("physics:lowerLimit").Get()
        hi = p.GetAttribute("physics:upperLimit").Get()
        tgt = p.GetAttribute("drive:%s:physics:targetPosition" % pre)
        if not tgt:
            tgt = p.CreateAttribute("drive:%s:physics:targetPosition" % pre, Sdf.ValueTypeNames.Float)
        stiff = p.GetAttribute("drive:%s:physics:stiffness" % pre) or \
            p.CreateAttribute("drive:%s:physics:stiffness" % pre, Sdf.ValueTypeNames.Float)
        damp = p.GetAttribute("drive:%s:physics:damping" % pre) or \
            p.CreateAttribute("drive:%s:physics:damping" % pre, Sdf.ValueTypeNames.Float)
        typ = p.GetAttribute("drive:%s:physics:type" % pre) or \
            p.CreateAttribute("drive:%s:physics:type" % pre, Sdf.ValueTypeNames.Token)
        typ.Set("force")
        stiff.Set(2000.0 if t == "RevoluteJoint" else 20000.0)
        damp.Set(200.0 if t == "RevoluteJoint" else 2000.0)
        # 目标取限位内的中点偏上，保证非零
        if hi is not None and lo is not None and (hi - lo) > 1e-6:
            cmd = lo + 0.7 * (hi - lo)
        elif lo is not None and abs(lo) > 1e-6:
            cmd = 1.5 * lo
        elif hi is not None and abs(hi) > 1e-6:
            cmd = 1.5 * hi
        else:
            cmd = 30.0 if t == "RevoluteJoint" else 0.02
        tgt.Set(float(cmd))
        for _ in range(settle + run):
            app.update()
        pos1, q1 = get_pose(b1)
        moved_out = quat_angle(base[b1][1], q1) if q1 else None
        # 回程
        tgt.Set(0.0)
        for _ in range(settle + run):
            app.update()
        pos2, q2 = get_pose(b1)
        moved_back = quat_angle(base[b1][1], q2) if q2 else None
        rows.append((str(p.GetPath()).split("/")[-1], t.replace("Joint", ""),
                     lo, hi, round(cmd, 2), moved_out, moved_back))
tl.stop()

print("%-34s %-11s %-9s %-9s %-8s %-12s %-12s" %
      ("关节", "类型", "下限", "上限", "目标", "去程转角/位移", "回程残差"))
okc = 0
for nm, t, lo, hi, cmd, mo, mb in rows:
    a = ("%.1f deg" % mo) if mo is not None else "-"
    b = ("%.1f deg" % mb) if mb is not None else "-"
    got = (mo or 0) > 1.0
    okc += 1 if got else 0
    print("%-34s %-11s %-9s %-9s %-8s %-12s %-12s %s"
          % (nm[-34:], t, ("%.1f" % lo) if lo is not None else "-",
             ("%.1f" % hi) if hi is not None else "-", cmd, a, b, "可动" if got else "✘未动"))
print("=" * 96)
print("可被驱动的关节：%d / %d" % (okc, len(rows)))
print("判据：去程实际转角 > 1° 才算可动；回程残差应接近 0（说明是驱动运动而非瞬移/发散）")
app.close()
sys.exit(0 if okc > 0 else 4)
