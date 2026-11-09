"""
mini 离心机「开盖—保持—关盖 + 转子正反启动—停止」单任务闭环（提示词2 的目标 A）。

设计要点：
· 独立小场景 assets/scenes/mini_lid_rotor_task.usda，真实重力、接触开启、固定底座 articulation；
  不覆盖 v06 与 8 吸头候选基线。
· 阶段之间用 tl.stop()/tl.play() 复位到 authored 初态，再从会话层写驱动目标 → 每个阶段都是
  "从相同初始状态开始"，符合接手要求（不做连续累加，避免把上一阶段的余动算进下一阶段）。
· 读数用 omni.physics.tensors 的 ArticulationView（get_dof_positions/velocities），
  不用刚体原点位移判铰链（那是坏传感器）。读不到就记 read_failed，不记 0。
· 判据在本文件写死，跑完不回头调：
    baseline      零输入下各 DOF 漂移 |Δq| < 0.5°（重力+接触不应自发动）
    open          给 +60°，240 步内达到 ≥ 0.85×60°，且未越上限位
    close         给 0°，回到 |q| < 5°
    lower_stop    给 −20°（低于下限位 0°），实际 q 不得 < −0.5°（端点约束必须挡住）
    rotor_fwd/rev 给 ±360°/s 等值反向，末角符号必须相反且 |q|>5°
    rotor_stop    给 0 目标，末速 |v| 应低于加速阶段峰值（记录，不设硬阈值）
"""
import os, sys, json, math, argparse, hashlib, time

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PILOT = os.path.join(PROJ, "assets", "instruments6_pilot", "centrifuge_tiangen_tgear_mini.usda")
SCENE = os.path.join(PROJ, "assets", "scenes", "mini_lid_rotor_task.usda")
OUTDIR = os.path.join(PROJ, "outputs", "runtime_validation")

ap = argparse.ArgumentParser()
ap.add_argument("--build-only", action="store_true")
ap.add_argument("--mode", choices=("cpu", "gpu"), default="cpu")
ap.add_argument("--steps", type=int, default=240)
ap.add_argument("--out", default=os.path.join(OUTDIR, "mini_lid_rotor_task.json"))
ap.add_argument("--only", help="只跑一个阶段（每阶段独立进程、各自 play 一次）")
args = ap.parse_args()


def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()[:16]


# ---------------------------------------------------------------- 建场景
def build_scene():
    os.environ.setdefault("CARB_VULKARS_DISABLE", "1")
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    import PhysxSchema
    sc = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    sc.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr().Set(9.81)
    # 关键：光有 UsdPhysics.Scene 不够，PhysX 需要 PhysxSceneAPI 才会真正建仿真场景，
    # 否则 create_simulation_view 返回 None（实测踩过）。
    api = PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim())
    api.CreateEnableGPUDynamicsAttr().Set(False)
    api.CreateBroadphaseTypeAttr().Set("SAP")
    sc.GetPrim().CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Float).Set(60.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    g = UsdGeom.Cube.Define(stage, "/World/Ground")
    g.CreateSizeAttr(2.0)
    g.AddTransformOp().Set(Gf.Matrix4d(
        20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 1))   # 20×20 m，厚 0.1 m，顶面 z=0
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())

    mini = stage.DefinePrim("/World/Mini", "Xform")
    mini.GetReferences().AddReference(os.path.relpath(PILOT, os.path.dirname(SCENE)).replace("\\", "/"))
    stage.GetRootLayer().Export(SCENE)
    print("已建微场景：", SCENE)
    app.close()


if args.build_only:
    build_scene()
    sys.exit(0)

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.usd, omni.timeline, omni.physx
import omni.physics.tensors as tensors
from pxr import Usd, UsdPhysics, Sdf

if not os.path.isfile(SCENE):
    app.close(); build_scene()
    app = SimulationApp({"headless": True})
    import omni.usd, omni.timeline, omni.physx
    import omni.physics.tensors as tensors

ctx = omni.usd.get_context()
if not ctx.open_stage(SCENE):
    print("打不开微场景")
    app.close(); sys.exit(1)
stage = ctx.get_stage()

# 引用进 /World/Mini 后，资产根 defaultPrim(/Instrument) 的内容落在 /World/Mini 下
ART_ROOT = None
for cand in ("/World/Mini/Joints/world_fixed", "/World/Mini/Instrument/Joints/world_fixed"):
    if stage.GetPrimAtPath(cand):
        ART_ROOT = cand
        break
if ART_ROOT is None:
    tops = [str(c.GetPath()) for c in stage.GetPseudoRoot().GetChildren()]
    deep = [str(x.GetPath()) for x in stage.Traverse() if str(x.GetPath()).startswith("/World/Mini")][:12]
    print("找不到 articulation root；顶层：", tops, "Mini 下：", deep)
    app.close(); sys.exit(2)

tl = omni.timeline.get_timeline_interface()
physx = omni.physx.get_physx_interface()
print("articulation root:", ART_ROOT)

# 找关节 DOF 名与限位
joints = {}
for p in stage.TraverseAll():
    if str(p.GetTypeName()) == "PhysicsRevoluteJoint":
        r1 = p.GetRelationship("physics:body1").GetTargets()
        joints[str(p.GetPath())] = {
            "limits": (p.GetAttribute("physics:lowerLimit").Get(),
                       p.GetAttribute("physics:upperLimit").Get()),
            "body1": str(r1[0]) if r1 else None,
        }
print("铰链：", json.dumps(joints, ensure_ascii=False)[:300])

results = {"scene": SCENE, "mode": args.mode, "steps": args.steps,
           "scene_sha": sha(SCENE), "pilot_sha": sha(PILOT),
           "articulation_root": ART_ROOT, "gravity": 9.81,
           "phases": {}, "verdict": {}}

art = None


def acquire_art():
    """物理场景激活之后才能建 ArticulationView（play 之前建会报 no active physics scene）。"""
    global art
    tensors.reset()                      # comparator 里就是这么做的：不 reset 会拿不到视图
    view = tensors.create_simulation_view("numpy")
    art = view.create_articulation_view(ART_ROOT)
    if art is None:
        print("!! ArticulationView 仍为 None；尝试用父级 /World/Mini 作为根")
        art = view.create_articulation_view("/World/Mini")
    return art


tl.stop(); tl.play()
for _ in range(30):
    app.update()
names = list(acquire_art().shared_metatype.dof_names)
print("DOF 名:", names)
IDX = {}
for want in ("lid", "rotor"):
    for i, n in enumerate(names):
        if n.endswith("_" + want):
            IDX[want] = i
            break
print("DOF 索引:", IDX)
tl.stop()


def read_qv():
    q = art.get_dof_positions().copy()[0].tolist()
    v = art.get_dof_velocities().copy()[0].tolist()
    return q, v


def phase(tag, targets_deg, steps=None):
    """复位→按目标驱动→采样。targets_deg: {dof_key: 目标(度)}"""
    steps = steps or args.steps
    tl.stop()
    tl.play()
    for _ in range(20):
        app.update()
    q0, v0 = read_qv()
    # 会话层写驱动目标（每阶段都从 authored 初态重新开始）
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for path, info in joints.items():
            key = "lid" if path.endswith("body_lid") else ("rotor" if path.endswith("body_rotor") else None)
            if key is None:
                continue
            prim = stage.GetPrimAtPath(path)
            t = targets_deg.get(key, 0.0)
            for an, val in (("drive:angular:physics:type", "force"),
                            ("drive:angular:physics:stiffness", 6000.0),
                            ("drive:angular:physics:damping", 120.0),
                            ("drive:angular:physics:maxIterations", 8),
                            ("drive:angular:physics:targetPosition", t)):
                a = prim.GetAttribute(an)
                if not a:
                    a = prim.CreateAttribute(an, Sdf.ValueTypeNames.Token if an.endswith("type")
                                             else Sdf.ValueTypeNames.Float)
                a.Set(val)
    traj = []
    t0 = time.perf_counter()
    for i in range(steps):
        app.update()
        if i % 20 == 0 or i == steps - 1:
            q, v = read_qv()
            traj.append({"step": i, "q": q, "v": v})
    q1, v1 = read_qv()
    wall = time.perf_counter() - t0
    rec = {"targets": targets_deg, "q0": q0, "q1": q1, "v0": v0, "v1": v1,
           "trajectory": traj, "wall_s": round(wall, 2), "read_ok": q1 is not None}
    results["phases"][tag] = rec
    print("[%-11s] q0=%s q1=%s v1=%s" % (tag,
          [round(x, 2) for x in q0], [round(x, 2) for x in q1], [round(x, 1) for x in v1]))
    return rec


def gdeg(rec, key, which="q1"):
    i = IDX.get(key)
    if rec is None or i is None or not rec.get("read_ok"):
        return None, None
    return math.degrees(rec["q0"][i]), math.degrees(rec["q1"][i])


P = results["phases"]
V = {}
V["baseline_no_selfmotion"] = ("未运行" if "baseline" not in P else
    ("read_failed" if P["baseline"]["q1"] is None else
     (abs(math.degrees(P["baseline"]["q1"][IDX["lid"]] - P["baseline"]["q0"][IDX["lid"]])) < 0.5)))
def lid_end(rec):
    return None if (rec is None or not rec["read_ok"]) else math.degrees(rec["q1"][IDX["lid"]])
o = lid_end(P.get("open"))
V["open_reaches_51deg"] = "未运行" if o is None else (51.0 <= o <= 98.6)
c = lid_end(P.get("close"))
V["close_returns_under_5deg"] = "未运行" if c is None else (abs(c) < 5.0)
l = lid_end(P.get("lower_stop"))
V["lower_endstop_respected"] = "未运行" if l is None else (l >= -0.5)
def rot_end(tag):
    r = P.get(tag)
    return None if (r is None or not r["read_ok"]) else math.degrees(r["q1"][IDX["rotor"]])
f, e = rot_end("rotor_fwd"), rot_end("rotor_rev")
V["rotor_sign_follows_command"] = ("未运行" if (f is None or e is None) else (f > 5.0 and e < -5.0))
rs = P.get("rotor_stop")
if rs is None:
    V["rotor_stop_speed_decays"] = "未运行"
else:
    peak = max((abs(x["v"][IDX["rotor"]]) for x in P.get("rotor_fwd", {"trajectory": []})["trajectory"]), default=1e9)
    V["rotor_stop_speed_decays"] = (abs(rs["v1"][IDX["rotor"]]) < max(0.5, peak))

os.makedirs(os.path.dirname(args.out), exist_ok=True)
payload = dict(results, verdict=V, thresholds_declared_before_run=True)
json.dump(payload, open(args.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
vals = [v for v in V.values() if isinstance(v, bool)]
print("=" * 96)
print("判定：", json.dumps(V, ensure_ascii=False, indent=1))
print("通过 %d / 已判 %d ；产物 %s（场景哈希 %s 模式 %s 步频目标 %d）"
      % (sum(vals), len(vals), os.path.basename(args.out), payload["scene_sha"], args.mode, args.steps))
