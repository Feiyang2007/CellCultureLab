"""
关节行程实测：给每个关节下驱动指令，跑仿真，看它**真的动了多少**。

为什么需要：只证"场景不塌"是不够的——师兄要的是"确实能进行交互任务"，
而"盖子能开"这件事必须量出来：指令目标 vs 实际转角。

做法：
  · 先读一次 PhysX 返回结构（有的版本带旋转，有的只给位置）；
  · 没有旋转就从 body1 的位置变化反推是否真在动（位移方向要符合轴向）；
  · 每个关节单独驱动，其余保持 0，避免互相干扰；
  · 力驱动需要刚度，缺驱动参数的关节临时补一个够用的刚度/阻尼（只改会话层，不落盘）。

用法：
    D:/IsaacSim5/env/Scripts/python.exe scripts/check_joint_range.py [场景] [--settle 60] [--run 120]
"""
import os, sys, math

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd, omni.timeline, omni.physx
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
args = [a for a in sys.argv[1:] if not a.startswith("--")]
scene = os.path.abspath(args[0] if args else os.path.join(
    PROJ, "assets", "scenes", "test_instruments2.usda"))
settle = int(sys.argv[sys.argv.index("--settle") + 1]) if "--settle" in sys.argv else 60
run = int(sys.argv[sys.argv.index("--run") + 1]) if "--run" in sys.argv else 120

ctx = omni.usd.get_context()
if not ctx.open_stage(scene):
    print("打不开：", scene); app.close(); sys.exit(1)
st = ctx.get_stage()
iface = omni.physx.get_physx_interface()
tl = omni.timeline.get_timeline_interface()


def pose(path):
    r = iface.get_rigidbody_transformation(path)
    if not isinstance(r, dict) or r.get("ret_val") is False:
        return None
    return r


def get_pos(p):
    v = p.get("position")
    if v is None:
        return None
    return [float(v[0]), float(v[1]), float(v[2])]


def get_rot(p):
    for k in ("orientation", "rotation", "quat"):
        v = p.get(k)
        if v is not None:
            return [float(x) for x in v]
    return None


def angle_from_quat(q0, q1):
    """两个四元数（w,x,y,z 或 x,y,z,w 都试）之间的夹角。"""
    if not q0 or not q1 or len(q0) != 4:
        return None
    def norm(q):
        n = math.sqrt(sum(x * x for x in q))
        return [x / n for x in q] if n else q
    a, b = norm(q0), norm(q1)
    # 统一成 (x,y,z,w)
    if abs(abs(a[0]) - 1.0) < 1e-3:      # 第一个分量接近 ±1 → 是 w 开头
        a = a[1:] + a[0:1]
        b = b[1:] + b[0:1]
    dot = min(1.0, abs(sum(x * y for x, y in zip(a, b))))
    return math.degrees(2 * math.acos(dot))


joints = []
for p in st.Traverse():
    t = str(p.GetTypeName())
    if t in ("RevoluteJoint", "PrismaticJoint"):
        r0 = p.GetRelationship("physics:body0").GetTargets()
        r1 = p.GetRelationship("physics:body1").GetTargets()
        if r0 and r1:
            joints.append((p, t, str(r1[0])))

print("=" * 96)
print("场景：", os.path.basename(scene), "  已绑 body 的关节：", len(joints))

tl.stop(); tl.play()
for _ in range(settle):
    app.update()

base0 = {}
for p, t, b1 in joints:
    q = pose(b1)
    base0[b1] = (get_pos(q), get_rot(q)) if q else (None, None)

rows = []
with Usd.EditContext(st, st.GetSessionLayer()):
    for p, t, b1 in joints:
        pre = "angular" if t == "RevoluteJoint" else "linear"
        lo = p.GetAttribute("physics:lowerLimit").Get()
        hi = p.GetAttribute("physics:upperLimit").Get()
        axis = p.GetAttribute("physics:axis").Get()
        tp = p.GetAttribute("drive:%s:physics:targetPosition" % pre)
        if not tp:
            tp = p.CreateAttribute("drive:%s:physics:targetPosition" % pre, Sdf.ValueTypeNames.Float)
        stiff = p.GetAttribute("drive:%s:physics:stiffness" % pre) or \
            p.CreateAttribute("drive:%s:physics:stiffness" % pre, Sdf.ValueTypeNames.Float)
        damp = p.GetAttribute("drive:%s:physics:damping" % pre) or \
            p.CreateAttribute("drive:%s:physics:damping" % pre, Sdf.ValueTypeNames.Float)
        ty = p.GetAttribute("drive:%s:physics:type" % pre) or \
            p.CreateAttribute("drive:%s:physics:type" % pre, Sdf.ValueTypeNames.Token)
        ty.Set("force")
        stiff.Set(8000.0 if t == "RevoluteJoint" else 30000.0)
        damp.Set(600.0 if t == "RevoluteJoint" else 3000.0)

        # 目标：优先往上限位走；无限位就往 +30° / +20mm
        if t == "RevoluteJoint":
            cmd = 30.0
            if hi is not None and hi > lo + 1e-6:
                cmd = max(min(hi, 60.0), (hi + (lo or 0)) / 2.0)
            elif lo is not None and lo < -1e-6:
                cmd = min(lo, -30.0)
        else:
            cmd = 0.03
            if hi is not None and hi > 1e-6:
                cmd = hi * 0.8
        tp.Set(float(cmd))

        for _ in range(run):
            app.update()
        now = pose(b1)
        moved = None
        ang = None
        if now and base0.get(b1) and base0[b1][0]:
            p0, r0v = base0[b1]
            p1 = get_pos(now)
            moved = math.dist(p0, p1)
            ang = angle_from_quat(r0v, get_rot(now))
        # 复位，避免影响下一个关节
        tp.Set(0.0)
        for _ in range(settle):
            app.update()
        base1 = pose(b1)
        if base1:
            base0[b1] = (get_pos(base1), get_rot(base1))
        rows.append((str(p.GetPath()).replace("/Instruments/", "").replace("/World/TaskArea/", ""),
                     t, axis, lo, hi, cmd, moved, ang))

tl.stop()
print()
print("%-52s %-14s %-3s %-9s %-9s %8s %8s %8s" %
      ("关节", "类型", "轴", "下限", "上限", "指令", "位移m", "转角deg"))
for path, t, axis, lo, hi, cmd, mv, ang in rows:
    print("%-52s %-14s %-3s %-9s %-9s %8.2f %8.4f %8s"
          % (path[-52:], t.replace("Physics", "").replace("Joint", ""), axis or "-",
             ("%.1f" % lo) if lo is not None else "-",
             ("%.1f" % hi) if hi is not None else "-",
             cmd, (mv if mv is not None else -1),
             ("%.1f" % ang) if ang is not None else "-"))
ok = sum(1 for r in rows if (r[6] or 0) > 0.001 or (r[7] or 0) > 0.5)
print()
print("能被驱动（有明显位移或转角）的关节：%d / %d" % (ok, len(rows)))
app.close()
