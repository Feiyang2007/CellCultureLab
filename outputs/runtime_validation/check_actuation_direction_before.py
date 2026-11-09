"""
方向性驱动判定：同一关节分别给 +目标 与 −目标，看实际转角**是否随指令反号**。
重力塌开只会单向偏，驱动才会双向跟随 —— 这是上一版测法（只测有无角度变化）分不清的。

Signed angle: delta = q_after · q_base⁻¹，取向量部分点乘该关节的**世界轴向**（取自 MuJoCo 真值），
符号即转向。同时打印 body1 是否被误设运动学、质量、限位，便于判"不动"的原因。

用法：
    D:/IsaacSim5/env/Scripts/python.exe scripts/check_actuation_direction.py [场景] [--settle 60] [--run 90]
"""
import os, sys, math, json, re

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd, omni.timeline, omni.physx
from pxr import Usd, UsdPhysics, Sdf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRUTH = os.path.join(PROJ, "upstream", "truth")
args = [a for a in sys.argv[1:] if not a.startswith("--")]
scene = os.path.abspath(args[0] if args else os.path.join(
    PROJ, "assets", "scenes", "test_instruments4.usda"))
settle = int(sys.argv[sys.argv.index("--settle") + 1]) if "--settle" in sys.argv else 60
run = int(sys.argv[sys.argv.index("--run") + 1]) if "--run" in sys.argv else 90


def san(s):
    s = re.sub(r"[^0-9A-Za-z_]", "_", str(s or "x"))
    return "_" + s if s[:1].isdigit() else s


# 关节世界轴向查表：USD 里的关节名是 san(parent_jointname)
AXIS = {}
MASS_OF = {}
for f in [x for x in os.listdir(TRUTH) if x.endswith(".json")]:
    T = json.load(open(os.path.join(TRUTH, f), encoding="utf-8"))
    bn = {b["id"]: b for b in T["bodies"]}
    for j in T["joints"]:
        parent = bn.get(j["body_id"], {}).get("name", "")
        AXIS[san(parent + "_" + j["name"])] = j["world_axis"]

ctx = omni.usd.get_context()
if not ctx.open_stage(scene):
    print("打不开：", scene)
    app.close(); sys.exit(1)
st = ctx.get_stage()
iface = omni.physx.get_physx_interface()
tl = omni.timeline.get_timeline_interface()

joints = []
for p in st.Traverse():
    t = str(p.GetTypeName())
    if t not in ("RevoluteJoint", "PrismaticJoint"):
        continue
    r0 = p.GetRelationship("physics:body0").GetTargets()
    r1 = p.GetRelationship("physics:body1").GetTargets()
    if r0 and r1:
        joints.append((p, t, str(r1[0])))
print("=" * 104)
print("场景：", os.path.basename(scene), " 关节：", len(joints),
      " 已索引世界轴向：", len(AXIS))


def pose(path):
    r = iface.get_rigidbody_transformation(path)
    if not isinstance(r, dict) or r.get("ret_val") is False:
        return None, None
    p = r.get("position")
    q = r.get("orientation") or r.get("rotation") or r.get("quat")
    return (list(p) if p is not None else None, list(q) if q is not None else None)


def step(n):
    for _ in range(n):
        app.update()


def signed(q0, q1, axis):
    """delta 向量部分 · 世界轴向（小角度近似，够判方向）"""
    if not q0 or not q1 or not axis:
        return None
    a, b = list(q0), list(q1)
    if abs(abs(a[0]) - 1) < 1e-9 and abs(a[1]) < 1e-9:
        pass
    n0 = math.sqrt(sum(x * x for x in a)); a = [x / n0 for x in a]
    n1 = math.sqrt(sum(x * x for x in b)); b = [x / n1 for x in b]
    w0, x0, y0, z0 = a
    w1, x1, y1, z1 = b
    dw = w1 * w0 + x1 * x0 + y1 * y0 + z1 * z0
    dx = w1 * x0 - x1 * w0 - (y1 * z0 - z1 * y0)
    dy = w1 * y0 - y1 * w0 - (z1 * x0 - x1 * z0)
    dz = w1 * z0 - z1 * w0 - (x1 * y0 - y1 * x0)
    vx, vy, vz = dx, dy, dz
    nn = math.sqrt(vx * vx + vy * vy + vz * vz)
    ang = 2 * math.degrees(math.asin(min(1.0, nn)))
    if abs(dw) < 1:
        ang = 2 * math.degrees(math.acos(max(-1.0, min(1.0, dw)))) * (1 if nn > 0 else 0)
    s = (vx * axis[0] + vy * axis[1] + vz * axis[2]) / (nn or 1.0)
    return ang * (1 if s >= 0 else -1)


tl.stop(); tl.play(); step(30)
rows = []
with Usd.EditContext(st, st.GetSessionLayer()):
    for p, t, b1 in joints:
        pre = "angular" if t == "RevoluteJoint" else "linear"
        axis = AXIS.get(str(p.GetPath()).split("/")[-1])
        if axis is None:
            for k, v in AXIS.items():
                if k.endswith(san(p.GetName())) or p.GetName() == k:
                    axis = v; break
        b1p = st.GetPrimAtPath(b1)
        kin = b1p.GetAttribute("physics:kinematicEnabled").Get() if b1p else None
        mass = b1p.GetAttribute("physics:mass").Get() if b1p else None
        lo = p.GetAttribute("physics:lowerLimit").Get()
        hi = p.GetAttribute("physics:upperLimit").Get()
        tgt = p.GetAttribute("drive:%s:physics:targetPosition" % pre) or \
            p.CreateAttribute("drive:%s:physics:targetPosition" % pre, Sdf.ValueTypeNames.Float)
        stf = p.GetAttribute("drive:%s:physics:stiffness" % pre) or \
            p.CreateAttribute("drive:%s:physics:stiffness" % pre, Sdf.ValueTypeNames.Float)
        dmp = p.GetAttribute("drive:%s:physics:damping" % pre) or \
            p.CreateAttribute("drive:%s:physics:damping" % pre, Sdf.ValueTypeNames.Float)
        ty = p.GetAttribute("drive:%s:physics:type" % pre) or \
            p.CreateAttribute("drive:%s:physics:type" % pre, Sdf.ValueTypeNames.Token)
        ty.Set("force"); stf.Set(3000.0 if t == "RevoluteJoint" else 30000.0)
        dmp.Set(300.0 if t == "RevoluteJoint" else 3000.0)
        mag = 20.0 if t == "RevoluteJoint" else 0.02
        if lo is not None and hi is not None and (hi - lo) > 4 * mag:
            mag = min(mag, 0.25 * (hi - lo))
        base_p, base_q = pose(b1)
        tgt.Set(+mag); step(settle + run); _, qp = pose(b1)
        a_pos = signed(base_q, qp, axis)
        tgt.Set(-mag); step(settle + run); _, qm = pose(b1)
        a_neg = signed(base_q, qm, axis)
        tgt.Set(0.0); step(30)
        driven = (a_pos is not None and a_neg is not None and
                  abs(a_pos) > 1.0 and abs(a_neg) > 1.0 and (a_pos * a_neg) < 0)
        oneway = (a_pos is not None and a_neg is not None and
                  abs(a_pos) > 1.0 and (a_pos * a_neg) > 0)
        verdict = "驱动成立" if driven else ("单向(疑重力/摩擦偏置)" if oneway else "✘不动")
        rows.append((str(p.GetPath()).split("/")[-1][:34], t.replace("Joint", ""),
                     kin, mass, a_pos, a_neg, verdict))

tl.stop()
print("%-34s %-9s %-5s %-7s %-11s %-11s %s" %
      ("关节", "类型", "运动学", "质量kg", "+目标转角", "−目标转角", "判定"))
for nm, t, kin, mass, ap, an, v in rows:
    print("%-34s %-9s %-5s %-7s %-11s %-11s %s"
          % (nm, t, "是★" if kin else "否", ("%.3f" % mass) if mass else "?",
             ("%+.1f" % ap) if ap is not None else "-",
             ("%+.1f" % an) if an is not None else "-", v))
nd = sum(1 for r in rows if r[6] == "驱动成立")
print("=" * 104)
print("结论：真正由 drive 驱动（转向随指令反号）的关节 %d / %d" % (nd, len(rows)))
app.close()
sys.exit(0 if nd else 4)
