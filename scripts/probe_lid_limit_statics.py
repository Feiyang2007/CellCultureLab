"""盖子压在上下限位附近时「指令力矩到底要多大才动」的静态/起跳实测。

动机（不猜原因，先量）：--sequence lid_loop --gravity real 里，盖子在 brake 段撞到
上限位 q=1.712 rad 后，close 段给 −0.05 N·m（≈ 参考在 q=0.7 处实测重力矩 9.53e-3 N·m 的 5 倍）
连续 180 步位移精确为 0；给 −2.0 N·m 则一步到位关到 q=0。每步 wake_up() 也无效，
所以「PhysX 休眠」这条已经排除，剩下的候选是：该角度下的重力矩比 q=0.7 处大得多、
限位求解的保持力、或指令没送达。这三者都能用「零输入测当地重力加速度 + 力矩阈值扫描」分开。

方法沿用 3.3.0 参考协议（同一资产、同一读路、标量力矩、240 Hz）：
  每个工况：reset_simulation → tensors.reset → open_stage → 配好场景 → start_simulation
  → 注入 (q, v=0) → 跑 5 步 → 读 q/v → 用 Δv/Δt 反算当地角加速度与力矩
  torque=0 的行直接给出当地重力矩；非零行给出「是否起离限位」。
干净性核对：整个矩阵跑完再重复第一个工况一次，比较两次读数（基线可重复性），
不一致就说明同进程多工况互相污染，本脚本的结论作废，改成每工况一个进程。
"""
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "assets/scenes/mini_closed_loop_v2.usda"
OUT = ROOT / "outputs/runtime_validation/mini_lid_limit_statics.json"
DT = 1.0 / 240.0
NSTEPS = 5

os.environ["TEMP"] = os.environ["TMP"] = str(ROOT / "outputs/runtime_validation/_statics_tmp")
os.makedirs(os.environ["TEMP"], exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True,
                     "extra_args": ["--portable-root", str(Path.home() / "CellCultureLabRuntime"),
                                    "--/app/settings/persistent=false",
                                    "--/renderer/multiGpu/enabled=false"]})
import numpy as np
import omni.usd, omni.physx
import omni.physics.tensors as tensors
from pxr import UsdPhysics, PhysxSchema, Gf

import argparse
ap2 = argparse.ArgumentParser()
ap2.add_argument("--curve", action="store_true",
                 help="2 步短采样，测各角度的重力矩曲线")
ap2.add_argument("--fine", action="store_true",
                 help="在限位前后一点点角度上扫描，用来分开「限位接触保持力」与「当地重力矩」")
a2 = ap2.parse_args()   # 必须真读 sys.argv；之前写 parse_args([]) 让 --fine 静默失效
ref = json.loads((ROOT / "upstream/physics_reference/centrifuge_tiangen_tgear_mini.json").read_text(encoding="utf-8"))
probe = {(t["joint"], t["force"], tuple(t["gravity"])): t for t in ref["trials"]}
I_LID = 0.001 / probe[("lid", 0.001, (0.0, 0.0, 0.0))]["initial_acceleration"][0]

ANGLES = {"q0.7_mid": 0.700, "q_upper_1.712": 1.712, "q_lower_0.0": 0.0}
TORQUES = [0.0, -0.01, -0.05, -0.10, -0.20, -1.0, -2.0, +0.05, +2.0]
GRAVS = {"none": 0.0, "real": 9.81}
if a2.fine and not a2.curve:
    ANGLES = {"just_inside_1.700": 1.700, "just_inside_1.710": 1.710,
              "at_stop_1.712": 1.712, "mid_0.700": 0.700, "just_above_lower_0.010": 0.010}
    TORQUES = [0.0, -0.02, -0.05, -0.075, -0.10]
    GRAVS = {"real": 9.81, "none": 0.0}
    OUT = ROOT / "outputs/runtime_validation/mini_lid_limit_statics_fine.json"
elif a2.curve:
    # 只走 2 步，角度还来不及变就采数 → 得到 τ_g(θ) 曲线，不被限位投影污染
    NSTEPS = 2
    ANGLES = {("q%.2f" % q): q for q in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,
                            0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7]}
    TORQUES = [0.0]
    GRAVS = {"real": 9.81, "none": 0.0}
    OUT = ROOT / "outputs/runtime_validation/mini_lid_gravity_torque_curve.json"

ctx = omni.usd.get_context()
iface = omni.physx.get_physx_interface()
report = {"scope": "local gravity torque + breakaway threshold sweep at joint limits; not task acceptance",
          "preset": "curve" if a2.curve else ("fine" if a2.fine else "coarse"),
          "scene": str(SCENE), "dt": DT, "n_steps": NSTEPS, "I_lid_from_330": I_LID,
          "rows": [], "repeat_check": None}


def one_case(angle_key, grav_key, torque):
    iface.reset_simulation()
    tensors.reset()
    if not ctx.open_stage(str(SCENE)):
        raise RuntimeError("open_stage_failed")
    stage = ctx.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    sp = stage.GetPrimAtPath("/physicsScene")
    sc = UsdPhysics.Scene(sp)
    sc.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr(GRAVS[grav_key])
    ps = PhysxSchema.PhysxSceneAPI.Apply(sp)
    ps.CreateEnableGPUDynamicsAttr(False)
    ps.CreateBroadphaseTypeAttr("MBP")
    ps.CreateTimeStepsPerSecondAttr(round(1 / DT))
    iface.start_simulation()
    iface.update_simulation(DT, 0)
    view = tensors.create_simulation_view("numpy")
    try:
        art = view.create_articulation_view("/Mini/Joints/world_fixed")
        if art is None or art.count != 1 or art.max_dofs != 2:
            raise RuntimeError("articulation_not_found")
        names = list(art.shared_metatype.dof_names)
        li = next(i for i, n in enumerate(names) if n.endswith("_lid"))
        q = np.zeros((1, art.max_dofs), dtype=np.float32)
        v = q.copy()
        f = q.copy()
        q[0, li] = ANGLES[angle_key]
        f[0, li] = torque
        art.set_dof_positions(q, np.array([0], dtype=np.uint32))
        art.set_dof_velocities(v, np.array([0], dtype=np.uint32))
        q0 = art.get_dof_positions().copy()[0][li]
        for i in range(NSTEPS):
            art.set_dof_actuation_forces(f, np.array([0], dtype=np.uint32))
            iface.update_simulation(DT, (i + 1) * DT)
        q1 = art.get_dof_positions().copy()[0][li]
        v1 = art.get_dof_velocities().copy()[0][li]
        a = v1 / (NSTEPS * DT)
        return {"angle": angle_key, "q_in_rad": round(float(q0), 6), "gravity": grav_key,
                "torque_Nm": torque, "dq_rad": round(float(q1 - q0), 8),
                "v_end_rad_s": round(float(v1), 6),
                "accel_from_v_rad_s2": round(float(a), 4),
                "implied_net_torque_Nm": round(float(I_LID * a), 8),
                "moved_off_stop": bool(abs(float(q1 - q0)) > 1e-6)}
    finally:
        view.invalidate()
        iface.reset_simulation()


try:
    for grav in GRAVS:
        for ang in ANGLES:
            for tq in TORQUES:
                if grav == "none" and tq != 0.0 and not ang.endswith("1.712") and not ang.endswith("1.710"):
                    continue                      # 零重力只用来对照限位/接触保持力
                row = one_case(ang, grav, tq)
                report["rows"].append(row)
                print("ROW " + json.dumps(row), flush=True)
    first = report["rows"][0]
    again = one_case(first["angle"], first["gravity"], first["torque_Nm"])
    report["repeat_check"] = {"first": first, "last": again,
                              "identical": all(first[k] == again[k] for k in
                                               ("q_in_rad", "dq_rad", "v_end_rad_s", "moved_off_stop"))}
    print("REPEAT " + json.dumps(report["repeat_check"]), flush=True)
    zero_g = [r for r in report["rows"] if r["torque_Nm"] == 0.0 and r["gravity"] == "real"]
    report["local_gravity_torque_Nm"] = {r["angle"]: r["implied_net_torque_Nm"] for r in zero_g}
except Exception as exc:
    report["error"] = "%s: %s" % (type(exc).__name__, exc)
finally:
    OUT.write_text(json.dumps(report, indent=1, ensure_ascii=False), encoding="utf-8")
    print("STATICICS_WRITTEN", OUT, flush=True)
    app.close()
