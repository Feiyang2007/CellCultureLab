"""
只读读数链路验证（补充提示 §七）：一次物理启动、一个固定底座 articulation、两个 DOF、
连续几十个真实物理步的有限 q/v、锚点约束、干净退出。**不判任务、不动 root、不改已通过资产。**

严格复用已跑通的 compare_mini_physics.py 的做法：
  iface.reset_simulation() → tensors.reset() → open_stage → 会话层设 gravity/PhysxSceneAPI
  → print("MARK before_start", flush=True)
  iface.start_simulation() → iface.update_simulation(dt, 0) → create_simulation_view
  → create_articulation_view("/Mini/Joints/world_fixed") → 校验 count/fixed_base/max_dofs/名称
  → 逐步 iface.update_simulation(dt, t) 读 art.get_dof_positions/velocities
  → finally view.invalidate(); iface.reset_simulation()

状态只允许四类，且区分"未测"与"读失败"：
  chain_pass_no_motion / chain_pass_with_response / init_failed / crash_or_timeout
"""
import os, sys, json, math, hashlib, traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCENE = ROOT / "assets" / "scenes" / "test_mini_articulation.usda"
ASSET = ROOT / "assets" / "instruments6_pilot" / "centrifuge_tiangen_tgear_mini.usda"
OUT = ROOT / "outputs" / "runtime_validation" / "mini_readback_chain.json"
STEPS = 60
DT = 1.0 / 240.0
ART_ROOT = "/Mini/Joints/world_fixed"

report = {"scope": "read-only runtime readback chain; NOT task acceptance",
          "scene": str(SCENE), "scene_sha256": None, "asset_sha256": None,
          "dt": DT, "steps_requested": STEPS, "steps_read": 0,
          "time_steps_per_second": round(1 / DT), "gpu_dynamics": False,
          "broadphase": "MBP", "gravity_magnitude": 9.81,
          "first_failure_layer": None, "status": None,
          "dof_names": None, "lid_rotor_mapping": None,
          "finite_all": None, "max_abs_dq_deg": 0.0, "max_joint_position_error_m": 0.0,
          "errors": [], "trajectory_head": []}

for p, key in ((SCENE, "scene_sha256"), (ASSET, "asset_sha256")):
    if not p.is_file():
        report["status"] = "init_failed"
        report["first_failure_layer"] = "missing_file:%s" % p.name
        OUT.parent.mkdir(parents=True, exist_ok=True)
        OUT.write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
        print("缺文件：", p)
        sys.exit(1)
    report[key] = hashlib.sha256(p.read_bytes()).hexdigest()

os.environ["TEMP"] = os.environ["TMP"] = str(ROOT / "outputs" / "runtime_validation" / "_chain_tmp")
os.makedirs(os.environ["TEMP"], exist_ok=True)

from isaacsim import SimulationApp
app = SimulationApp({"headless": True,
                     "extra_args": ["--portable-root", str(Path.home() / "CellCultureLabRuntime"),
                                    "--/app/settings/persistent=false",
                                    "--/renderer/multiGpu/enabled=false"]})
import numpy as np
import omni.usd, omni.physx
import omni.physics.tensors as tensors
from pxr import UsdPhysics, UsdGeom, PhysxSchema, Gf
sys.path.insert(0, str(ROOT / "scripts"))
from runtime_joint_math import joint_constraint_error

iface = None
view = None
written = []


def flush_report():
    """必须在 app.close() 之前落盘：关闭阶段会原生退出，写在后面就永远不执行。"""
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    written.append(True)

try:
    ctx = omni.usd.get_context()
    print("MARK after_iface", flush=True)
    iface = omni.physx.get_physx_interface()
    iface.reset_simulation()
    tensors.reset()
    print("MARK before_open", flush=True)
    if not ctx.open_stage(str(SCENE)):
        raise RuntimeError("open_stage_failed")
    stage = ctx.get_stage()
    report["composition_errors"] = []
    print("MARK after_open", flush=True)
    scene_prim = stage.GetPrimAtPath("/physicsScene")
    if not scene_prim:
        raise RuntimeError("no_physicsScene_in_scene")
    scene_prim = UsdPhysics.Scene(scene_prim)
    scene_prim.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene_prim.CreateGravityMagnitudeAttr(9.81)
    ps = PhysxSchema.PhysxSceneAPI.Apply(scene_prim.GetPrim())
    ps.CreateEnableGPUDynamicsAttr(False)
    ps.CreateBroadphaseTypeAttr("MBP")
    attr_tps = ps.CreateTimeStepsPerSecondAttr(round(1 / DT))
    report["time_steps_per_second_readback"] = attr_tps.Get()

    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    report["revolute_joint_count"] = len(joints)
    report["rigid_body_count"] = sum(1 for p in stage.Traverse()
                                     if p.HasAPI(UsdPhysics.RigidBodyAPI))

    iface.start_simulation()
    print("MARK after_start", flush=True)
    iface.update_simulation(DT, 0)
    report["first_failure_layer"] = "after_start_simulation"

    print("MARK before_view", flush=True)
    view = tensors.create_simulation_view("numpy")
    if view is None:
        raise RuntimeError("simulation_view_is_None")
    print("MARK after_view", flush=True)
    art = view.create_articulation_view(ART_ROOT)
    if art is None:
        raise RuntimeError("articulation_view_is_None")
    report["first_failure_layer"] = "articulation_invariants"
    if art.count != 1:
        raise RuntimeError("articulation_count=%d" % art.count)
    if not art.shared_metatype.fixed_base:
        raise RuntimeError("not_fixed_base")
    if art.max_dofs != 2:
        raise RuntimeError("max_dofs=%d" % art.max_dofs)
    print("MARK after_invariants", flush=True)
    names = list(art.shared_metatype.dof_names)
    report["dof_names"] = names
    mapping = {}
    for want in ("lid", "rotor"):
        hit = [i for i, n in enumerate(names) if n.endswith("_" + want)]
        if not hit:
            raise RuntimeError("no_dof_for_%s" % want)
        mapping[want] = hit[0]
    report["lid_rotor_mapping"] = mapping

    print("MARK before_bodies", flush=True)
    bodies = view.create_rigid_body_view("/Mini/Bodies/*")
    q0 = art.get_dof_positions().copy()[0].tolist()
    if not all(math.isfinite(x) for x in q0):
        raise RuntimeError("initial_q_not_finite")

    print("MARK before_loop", flush=True)
    prev = q0
    for step in range(STEPS + 1):
        q = art.get_dof_positions().copy()[0].tolist()
        v = art.get_dof_velocities().copy()[0].tolist()
        if not (all(math.isfinite(x) for x in q) and all(math.isfinite(x) for x in v)):
            raise RuntimeError("non_finite_read_at_step_%d" % step)
        report["steps_read"] = step + 1
        if step % 5 == 0:
            print("MARK step_%d" % step, flush=True)
        report["max_abs_dq_deg"] = max(report["max_abs_dq_deg"],
                                       max(abs(math.degrees(a - b)) for a, b in zip(q, prev)))
        prev = q
        if step == 0 or step % 20 == 0:
            report["trajectory_head"].append({"step": step, "q_deg": [round(math.degrees(x), 6) for x in q],
                                              "v_rad_s": [round(x, 6) for x in v]})
        poses = dict(zip(bodies.prim_paths, bodies.get_transforms().copy().tolist()))
        xyzw = lambda x: [*x.GetImaginary(), x.GetReal()]
        for j in joints:
            b0 = str(j.GetRelationship("physics:body0").GetTargets()[0])
            b1 = str(j.GetRelationship("physics:body1").GetTargets()[0])
            if b0 not in poses or b1 not in poses:
                raise RuntimeError("body_not_in_runtime_view:%s|%s" % (b0, b1))
            err = joint_constraint_error(
                "RevoluteJoint", poses[b0], poses[b1],
                list(j.GetAttribute("physics:localPos0").Get()),
                list(j.GetAttribute("physics:localPos1").Get()),
                xyzw(j.GetAttribute("physics:localRot0").Get()),
                xyzw(j.GetAttribute("physics:localRot1").Get()), "X")
            report["max_joint_position_error_m"] = max(report["max_joint_position_error_m"],
                                                        err["position_m"])
        if step < STEPS:
            # 只读：不写 q/v、不注入驱动，只推进真实物理步
            iface.update_simulation(DT, (step + 1) * DT)

    moved = report["max_abs_dq_deg"] > 0.05
    report["status"] = "chain_pass_with_response" if moved else "chain_pass_no_motion"
    report["first_failure_layer"] = None
    flush_report()
except Exception as e:
    report["status"] = "init_failed"
    report["errors"].append("%s: %s" % (type(e).__name__, str(e)[:200]))
    report["traceback_tail"] = traceback.format_exc().splitlines()[-6:]
    flush_report()
finally:
    try:
        if view is not None:
            view.invalidate()
        if iface is not None:
            iface.reset_simulation()
    except Exception as e:
        report["cleanup_error"] = str(e)[:160]
    app.close()

print(json.dumps({k: report[k] for k in
                  ("status", "first_failure_layer", "steps_read", "dof_names", "lid_rotor_mapping",
                   "max_abs_dq_deg", "max_joint_position_error_m", "time_steps_per_second_readback",
                   "errors") if k in report}, ensure_ascii=False, indent=1))
print("报告：", OUT)
