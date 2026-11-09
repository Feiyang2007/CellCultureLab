"""mini 离心机盖子/转子连续闭环 v2（补充提示 §四 缺口修正 + §五 收尾的"先一个动作"）。

与 v1（scripts/run_mini_lid_task.py，Codex 审阅快照 sha256 de1442e466e86fac...）的关系：
v1 的五个确定缺口按补充提示逐条修正，v1 文件本体不改，作为旧证据保留。

  §四.1 阶段真的被调度：SEGMENTS 注册表 + --sequence/--only 真实生效；未跑的阶段记
       "not_scheduled"，读失败记 "read_failed"，两者都不算 0、也不算通过。
  §四.2 生命周期照抄已跑通的 compare_mini_physics.py：
       reset_simulation → tensors.reset → open_stage → 会话层配置 → start_simulation →
       update_simulation(dt,0) → create_simulation_view → create_articulation_view →
       不变量校验（count/fixed_base/max_dof 名/有限）→ 连续真实物理步 → finally 失效/复位/关 app。
       中途不再 tl.stop()/tl.play()，不重建句柄，不用 app.update() 次数冒充物理步。
  §四.3 控制语义：只用 set_dof_actuation_forces（关节标量力矩），与 3.3.0 参考
       scalar_joint_protocol 同类型、同单位（N·m）；不再写 targetPosition 冒充 °/s 转速控制，
       不再注入 stiffness=6000/damping=120。资产自带的 drive 刚度/阻尼=0，会读回并写进报告，
       证明"没有注入驱动器"。开—保持—关、转动—滑行—制动都在**一次连续仿真**里做，
       段间不复位（复位会抹掉要测的状态）；每个段边界用数值证明状态是接着的。
  §四.4 场景配置：timeStepsPerSecond 用 PhysxSchema 正式 API 的 UInt 属性并读回；
       地面顶面按实测 mini 最低点留余量放置（不再靠注释说"顶面=0"）；
       导出后重新打开并检查引用可解析、组合无错、刚体/关节数量符合预期。
  §四.5 异常退出：每一步 MARK 立刻落盘（外部 .log + 段级 JSON 增量写），
       崩溃时保留"最后一个阶段标记 + 首条致命日志"，不把原生异常升级为已定位根因。

判据（跑之前写死，跑完不回改）来自 upstream/physics_reference/centrifuge_tiangen_tgear_mini.json
（MuJoCo 3.3.0，original_version_matched=true）：
  盖子有效惯量 I_lid = 0.001/0.9478327 = 1.05504e-3 kg·m²
  转子有效惯量 I_rotor = 0.001/65.503701 = 1.52663e-5 kg·m²
  ±0.001 N·m 作用 60 步 @240 Hz：盖子 Δq=±0.030113 rad(±1.7254°)、末速±0.236958 rad/s；
                              转子 Δq=±2.081107 rad(±119.24°)、末速±16.375925 rad/s
  零输入+重力 9.81：盖子初始角加速度 +9.034952 rad/s²（即重力矩 9.53e-3 N·m，方向朝开）
  关节 armature=damping=frictionloss 全 0 → 零输入滑行不应减速
"""
import argparse
import hashlib
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PILOT = ROOT / "assets/instruments6_pilot/centrifuge_tiangen_tgear_mini.usda"
REF_FILE = ROOT / "upstream/physics_reference/centrifuge_tiangen_tgear_mini.json"
OUTDIR = ROOT / "outputs/runtime_validation"
SCENE = ROOT / "assets/scenes/mini_closed_loop_v2.usda"
DT = 1.0 / 240.0

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--build-only", action="store_true", help="只建/校验场景，不跑物理")
ap.add_argument("--sequence", choices=("lid_loop", "lid_task", "lid_qstatic", "lid_slow", "rotor_loop"),
                default="lid_loop")
ap.add_argument("--gravity", choices=("none", "real"), default="none",
                help="none=与 3.3.0 标量力矩协议同条件；real=场景真实重力 9.81")
ap.add_argument("--mode", choices=("cpu", "gpu"), default="cpu", help="真的切换 PhysX GPU dynamics 并读回校验")
ap.add_argument("--only", help="逗号分隔的段名，只跑这些段（仍是一次连续仿真内的子集）")
ap.add_argument("--report", type=Path, default=None)
ap.add_argument("--ground-clearance", type=float, default=0.001, help="地面顶面与 mini 最低点的间隙 m")
ap.add_argument("--wake", choices=("off", "each_step"), default="off",
                help="每步显式 wake_up()；用于判定「压到限位后力矩无效」是否由 PhysX 休眠造成")
ap.add_argument("--close-torque", type=float, default=None, help="覆盖段表里 close 段的盖子力矩 N·m（不传就用段表定值）")
ap.add_argument("--label", default=None, help="报告文件名后缀，避免不同配置互相覆盖")
ap.add_argument("--record-steps", choices=("sampled", "full"), default="full",
                help="full=每个物理步都记 t/步号/推进前 q,v/实际施加力矩向量/推进后 q,v（float32 原样转记）；sampled=旧的分隔采样（不足以做全轨迹逐点对照）")
args = ap.parse_args()

if args.report is None:
    # 文件名必须把配置差异写全，否则后一次跑会悄悄覆盖前一次的证据
    tag = args.label or ("mini_%s_%s_%s%s%s" % (
        args.sequence, args.gravity, args.mode,
        "_wake" if args.wake == "each_step" else "",
        "" if args.close_torque is None else "_ct%g" % args.close_torque))
    args.report = OUTDIR / (tag + ".json")
LOG_PATH = args.report.with_suffix(".log")


def sha16(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]


STEP_FH = [None]
STEP_COUNT = 0


def step_record(meta):
    """逐步记录（jsonl）：保留张量原始读数与实际写入缓冲区的数值，不额外四舍五入。

    注意精度表述：底层 `get_dof_positions/velocities` 与力矩缓冲区都是 **float32**，
    转成 Python float / JSON 只是不再人为截位，并**不会增加底层精度**。
    上一版每 20 步采一次且没存力矩，1140 步只有 62 条，撑不起全轨迹逐点对照。"""
    path = args.report.with_suffix(".steps.jsonl")
    STEP_FH[0] = open(path, "w", encoding="utf-8")
    STEP_FH[0].write(json.dumps({"kind": "header", **meta}, ensure_ascii=False) + chr(10))
    return path


def step_write(rec):
    if STEP_FH[0] is not None:
        STEP_FH[0].write(json.dumps(rec, ensure_ascii=False) + chr(10))


def nz(x, nd=6):
    """非有限值一律写 null（读取失败不能伪装成 0，也不能把 NaN 写进 JSON）。"""
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return round(v, nd) if math.isfinite(v) else None


MARKS = []


def mark(layer, **extra):
    """崩溃前最后一个标记就是定位信息，所以每条都立刻落盘。"""
    MARKS.append({"layer": layer, "t": round(time.perf_counter() - T0, 3),
                  "wall": time.strftime("%H:%M:%S"), **extra})
    print("MARK " + layer + " " + json.dumps(extra, ensure_ascii=False, default=str), flush=True)
    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(MARKS[-1]["wall"] + " " + MARKS[-1]["layer"] + " " +
                json.dumps(extra, ensure_ascii=False, default=str) + "\n")



def composition_errors(stage):
    """组合错误必须来自真实的 prim index 局部错误集合。

    之前用 GetMetadata("referenceErrors") 不可靠（只在个别情况下挂元数据），
    而 hasattr 判不出来就写 0/空，会让"以后真的缺引用"也蒙混通过。
    现在：取到就报；取不到写 missing_api_*，绝不填 0。
    """
    # 本安装的 localErrors 挂在 **prim** 上（Usd.Stage 没有 GetPrimIndex，实测 AttributeError），
    # 沿用项目里 check_v5_independent.py / build_c1000_lock_pilot.py 已经用过的写法逐 prim 收。
    if not any(hasattr(p, "GetPrimIndex") for p in stage.Traverse()):
        return {"status": "missing_api_prim_GetPrimIndex"}
    try:
        errs = [str(e) for p in stage.Traverse() for e in p.GetPrimIndex().localErrors]
    except Exception as exc:
        return {"status": "error", "detail": "%s: %s" % (type(exc).__name__, exc)}
    hard = [e for e in errs if ("Could not open asset" in e or "unresolved" in e.lower()
                                or "error" in e.split("|")[0].lower())]
    return {"status": "ok", "count": len(errs), "hard_count": len(hard), "hard": hard[:8],
            "sample": errs[:4]}

T0 = time.perf_counter()
OUTDIR.mkdir(parents=True, exist_ok=True)
LOG_PATH.write_text("", encoding="utf-8")

# ---------------------------------------------------------------- 参考值与判据
ref = json.loads(REF_FILE.read_text(encoding="utf-8"))
assert ref["mujoco_version"] == "3.3.0" and ref["original_version_matched"] is True, "参考版本不是原版本"
jl = {j["name"]: j for j in ref["joints"]}
# 键必须带上 test_armature_override：参考里 (rotor, +0.001, 零重力) 出现两次
# （trial5 原生、trial7 把 armature 改成 0.1），只按三元素建字典会让后者悄悄覆盖前者，
# 于是 I_rotor 被当成 0.1 kg·m²（其实是 1.53e-5），"与 3.3.0 不符"是记账错不是物理错。
probe = {(t["joint"], t["force"], tuple(t["gravity"]), t.get("test_armature_override")): t
         for t in ref["trials"]}
I_lid = 0.001 / probe[("lid", 0.001, (0.0, 0.0, 0.0), None)]["initial_acceleration"][0]
I_rotor = 0.001 / probe[("rotor", 0.001, (0.0, 0.0, 0.0), None)]["initial_acceleration"][1]
grav_trial = [t for t in ref["trials"] if tuple(t["gravity"]) == (0.0, 0.0, -9.81)][0]
A_LID_GRAV = grav_trial["initial_acceleration"][0]
TAU_GRAV_LID = I_lid * A_LID_GRAV


def ref_end(joint, force, steps, name, grav=(0.0, 0.0, 0.0)):
    """取同协议参考轨迹末态（有就取，没有返回 None，不外推）。"""
    t = probe.get((joint, force, grav, None))
    if t is None or t["steps"] != steps:
        return None
    idx = 0 if name == "lid" else 1
    return {"q": t["trajectory"][-1]["q"][idx], "v": t["trajectory"][-1]["v"][idx]}


# 实测的 τ_g(q) 曲线（本仓库自己用 2 步短采数出来的，见 mini_lid_gravity_torque_curve.json）
CURVE_FILE = OUTDIR / "mini_lid_gravity_torque_curve.json"
curve = json.loads(CURVE_FILE.read_text(encoding="utf-8")) if CURVE_FILE.is_file() else None
TAU_G = ({r["angle"]: r["implied_net_torque_Nm"] for r in curve["rows"] if r["gravity"] == "real"}
         if curve else None)

if TAU_G:
    _pts = sorted((float(k[1:]), v) for k, v in TAU_G.items())
    GRAV_BALANCE_Q = next(a1 for (a1, _), (a2, b2) in zip(_pts, _pts[1:]) if _ * b2 < 0)   # 首个过零
    GRAV_MAX_OPEN_TORQUE = max(v for _, v in _pts)
    GRAV_MAX_CLOSE_TORQUE = min(v for _, v in _pts)
else:
    GRAV_BALANCE_Q = GRAV_MAX_OPEN_TORQUE = GRAV_MAX_CLOSE_TORQUE = None
RECHECK_FILE = OUTDIR / "mini_lid_gravity_torque_recheck.json"
recheck = json.loads(RECHECK_FILE.read_text(encoding="utf-8")) if RECHECK_FILE.is_file() else None
# 只允许用自由行程点拟合出来的定轴摆模型 τ = A sin q + B cos q + C。
# 限位处的短采读数是被约束钳住的净响应（q=0 处读到 0），拿它当前馈会在起步时少给 0.0457 N·m。
if recheck and "fit_coefficients" in recheck:
    FIT_ABC = (recheck["fit_coefficients"]["A"], recheck["fit_coefficients"]["B"], recheck["fit_coefficients"]["C"])
    FIT_RESID = recheck["fit_max_residual_Nm"]
else:
    FIT_ABC = FIT_RESID = None
CURVE_POINTS = sorted((float(k[1:]), v) for k, v in (TAU_G or {}).items())


def tau_g(q):
    """盖子重力矩 τ_g(q)（N·m，正=朝开）。

    用定轴摆拟合式而不是插值表：表里贴限位的那几个点不是重力矩。拟合系数来自
    mini_lid_gravity_torque_recheck.json（自由行程点最小二乘，残差 3.8e-6 N·m，
    与原 3.3.0 两条独立方法的端点值差 3.4e-5 N·m）。缺文件就直接失败，不假装有模型。
    """
    if FIT_ABC is None:
        raise RuntimeError("missing mini_lid_gravity_torque_recheck.json -> no feedforward model")
    A, B, C = FIT_ABC
    return A * math.sin(q) + B * math.cos(q) + C


PRED = {
    "I_lid_kg_m2": I_lid, "I_rotor_kg_m2": I_rotor,
    "lid_gravity_torque_at_q0.7_Nm": TAU_GRAV_LID,
    "measured_gravity_curve": {"file": str(CURVE_FILE.name) if curve else "missing",
                               "balance_q_rad": GRAV_BALANCE_Q,
                               "max_open_torque_Nm": GRAV_MAX_OPEN_TORQUE,
                               "max_close_torque_Nm": GRAV_MAX_CLOSE_TORQUE,
                               "note": "实测（Isaac 2 步短采），与 3.3.0 在 q=0.7 的 9.53e-3 N·m 相差 0.1%"},
    "lid_open_60": ref_end("lid", 0.001, 60, "lid"),
    "lid_rev_60": ref_end("lid", -0.001, 60, "lid"),
    "rotor_spin_60": ref_end("rotor", 0.001, 60, "rotor"),
}

# ---------------- 低速轨迹控制器的设计值（测试控制器，不是源设备执行器行为） ----------------
# 惯量用 3.3.0 参考反推的实测值；增益按声明的闭环带宽/阻尼比算出来，不是凑出来的。
SLOW = {"I_used_kg_m2": I_lid, "bandwidth_rad_s": 30.0, "damping_ratio": 0.7,
        "Kp_Nm_per_rad": I_lid * 30.0 ** 2,
        "Kd_Nm_s_per_rad": 2.0 * I_lid * 0.7 * 30.0,
        "v_design_rad_s": 1.0, "a_design_rad_s2": 20.0, "tau_sat_Nm": 0.25,
        "v_hard_cap_rad_s": 2.0,
        "pos_tol_rad": 0.01, "hold_pos_tol_rad": 0.005, "hold_v_tol_rad_s": 0.02,
        "stop_margin_rad": 0.05, "tau_hold_tol_Nm": 0.01,
        "dv_jump_max_rad_s_per_step": 1.0,
        "dv_design_expectation_rad_s_per_step": 20.0 * (1.0 / 240.0),
        "basis": "Kp=I*omega^2, Kd=2*I*zeta*omega；omega/zeta/I 全部写在报告里，可复算"}


PLAN_DEADBAND_RAD = 1e-6      # 规划死区：小于这个的距离是 float32 舍入噪声，不是行程


def trapezoid(q_from, q_to, steps, v_max, a_max, dt):
    """有限时间到位、且**提前规划减速**的参考轨迹（不靠撞限位停）。

    已修缺陷（修订 2）：张量读回来的 0.1 是 float32 的 0.10000000149，与目标 0.1 差
    约 1.49e-9 rad。老版本把这 1.5 纳弧度当成真实行程，规划出 t_ramp=√(d/a)=8.63 微秒
    的加速段（不足一个 4.17 毫秒仿真步），于是"保持段"第一步仍带着 a_ref=−20 rad/s² 的
    惯性前馈，力矩从 +0.03856 掉到 +0.01746 N·m，一步砸出 −0.08323 rad/s 的速度。
    现在：距离进入死区、或整条计划短不到一步，一律按"已在目标"出常值参考，
    位置/速度反馈照旧保留（那是压住噪声的东西，不能一起关掉）。
    """
    d = q_to - q_from
    if abs(d) <= PLAN_DEADBAND_RAD or steps * dt < dt:
        return [(q_to, 0.0, 0.0)] * steps, 0.0
    sgn = 1.0 if d >= 0 else -1.0
    ad = abs(d)
    t_ramp = v_max / a_max
    d_ramp = 0.5 * a_max * t_ramp ** 2
    if 2 * d_ramp <= ad:
        t_cruise = (ad - 2 * d_ramp) / v_max
    else:                                   # 行程太短，只能三角波
        t_ramp = math.sqrt(ad / a_max); t_cruise = 0.0; v_max = a_max * t_ramp
    if t_ramp < dt:                         # 加速段不足一步 → 同属噪声级行程，别把它规划成运动
        return [(q_to, 0.0, 0.0)] * steps, 0.0
    t_total = 2 * t_ramp + t_cruise
    out = []
    for i in range(steps):
        t = min(i * dt, t_total)
        if t < t_ramp:
            q = q_from + sgn * 0.5 * a_max * t * t
            v = sgn * a_max * t; a_ = sgn * a_max
        elif t < t_ramp + t_cruise:
            q = q_from + sgn * (0.5 * a_max * t_ramp ** 2 + v_max * (t - t_ramp))
            v = sgn * v_max; a_ = 0.0
        else:
            tt = t_total - t
            q = q_to - sgn * 0.5 * a_max * tt * tt
            v = sgn * a_max * tt; a_ = -sgn * a_max
        if t >= t_total:
            q, v, a_ = q_to, 0.0, 0.0
        out.append((q, v, a_))
    return out, t_total


THRESH = {
    "declared_before_run": True,
    "source": "upstream/physics_reference/centrifuge_tiangen_tgear_mini.json (MuJoCo 3.3.0)",
    "sign_eps_rad": math.radians(0.05),
    "lid_brake_nulls_velocity": {"end_abs_v_lt_rad_s": 0.01,
                                "predicted_v_after_brake": 0.0,
                                "basis": "同一力矩等时长反向：Δv = 2·0.236958 → 末速应回到 0"},
    "lid_hold_static_zero_g": {"max_abs_dq_deg": 0.05, "max_abs_v_rad_s": 0.01,
                              "requires": "进入 hold 前速度必须已经归零（见 lid_brake_nulls_velocity）"},
    "lid_hold_under_gravity_expected_to_open": {"min_dq_deg": 5.0,
                                                "basis": "参考实测重力角加速度 +9.034952 rad/s²，60 步 ≈ +16.2°"},
    "lid_open_zero_g": {"min_dq_deg": 1.0, "ref_dq_deg": math.degrees(PRED["lid_open_60"]["q"] - 0.7),
                        "ref_v": probe[("lid", 0.001, (0.0, 0.0, 0.0), None)]["trajectory"][-1]["v"][0],
                        "tol_rel": 0.25},
    "lid_close": {"must_reach_lower_stop_deg_lt": 1.0, "end_abs_v_lt_rad_s": 0.05,
                  "torque_must_exceed_gravity": TAU_GRAV_LID, "applies_when": "gravity == real"},
    "rotor_spin": {"min_dq_deg": 100.0, "ref_dq_deg": math.degrees(PRED["rotor_spin_60"]["q"]),
                   "ref_v": PRED["rotor_spin_60"]["v"], "tol_rel": 0.25},
    "rotor_coast_no_hidden_damping": {"max_v_decay_rel": 0.05},
    "rotor_brake_stops": {"end_abs_v_lt_rad_s": 0.5, "predicted_brake_steps": 60},
    "segment_duration_basis": "时长按实测 τ_g(q) 与 I_lid 的运动学估算，让触发释放落在行程中段："
                              "open 0→0.65 rad ≈ 22 步（a≈160 rad/s²），close 1.712→0.5 rad ≈ 34 步（a≈123 rad/s²）；"
                              "触发未发生就记 reached=false，不偷偷延长",
    "lid_task_basis": {"gravity_balance_q_rad": GRAV_BALANCE_Q,
                       "press_in_torque_at_open_stop_Nm": GRAV_MAX_OPEN_TORQUE,
                       "hold_close_torque_at_lower_side_Nm": GRAV_MAX_CLOSE_TORQUE,
                       "close_torque_used_Nm": -0.200,
                       "open_torque_used_Nm": +0.200,
                       "margin": "0.200 / 0.0743 ≈ 2.7 倍，取自实测曲线的最大压入力矩",
                       "artifact": "outputs/runtime_validation/mini_lid_gravity_torque_curve.json"},
}
# 第一次跑（段表 open→hold→close，无 brake）时按"零输入=静止"判 hold，实测得到 +3.397°。
# 那是判据定义错，不是资产错：源模型 damping=frictionloss=0，带 0.237 rad/s 余速进入零输入
# 段必然匀速滑行（0.236958·0.25 s = 3.397°，与实测到小数第 3 位一致）。修订只是给"保持"
# 补一个前置换制速段，历史数字与原声明保留在 *.first_run_as_declared.json。
THRESH["revision_log"] = [{
    "was": "hold 段紧跟 open 段，判 |Δq|<0.05° 为保持成功",
    "now": "open → brake(等值反向、等时长) → hold，先证明余速归零再判保持",
    "reason": "零输入不等于零速度；源关节无阻尼，物理上就该滑行",
    "measured_evidence": "hold Δq=+3.39652°，|v| 全程 0.23696 未衰减",
    "original_artifact": "outputs/runtime_validation/mini_lid_loop_none_cpu.first_run_as_declared.json"}]

# 段定义：连续仿真内的分段力矩计划（N·m），一次复位都不做
SEGMENTS = {
    "lid_loop": [
        {"name": "open", "steps": 60, "torque": {"lid": +0.001, "rotor": 0.0}},
        {"name": "brake", "steps": 60, "torque": {"lid": -0.001, "rotor": 0.0}},
        {"name": "hold", "steps": 60, "torque": {"lid": 0.0, "rotor": 0.0}},
        {"name": "close", "steps": 180, "torque": {"lid": -0.050, "rotor": 0.0}},   # 覆盖见 --close-torque
    ],
    "lid_task": [
        # 起始 q=0（重力把它压在下限位，实测 τ_g(0.1)=-0.0386 N·m 朝关）。
        # 开盖：给 +0.2 N·m，推到刚越过实测重力平衡角 q≈0.585 就撤掉力矩；
        #       剩下的行程必须由重力自己完成，才算"这台设备的开盖"，不是"我把它瞬移到开位"。
        {"name": "open", "steps": 25, "torque": {"lid": +0.200, "rotor": 0.0},
         "release_above": 0.650},
        {"name": "release_after_open", "steps": 180, "torque": {"lid": 0.0, "rotor": 0.0}},
        {"name": "hold_open", "steps": 60, "torque": {"lid": 0.0, "rotor": 0.0}},
        # 关盖：上限位处重力压入 0.0743 N·m，给 −0.2 N·m（2.7 倍余量）把它拉开，
        #       退到平衡角以下立刻撤力，让重力把盖子压回下限位。
        {"name": "close", "steps": 45, "torque": {"lid": -0.200, "rotor": 0.0},
         "release_below": 0.500},
        {"name": "release_after_close", "steps": 180, "torque": {"lid": 0.0, "rotor": 0.0}},
    ],
    "lid_qstatic": [
        # 重力前馈 + 3 mN·m 偏置：慢速开盖（对比 lid_task 的 0.2 N·m 甩开，峰值角速要小一个量级）
        # 偏置不是随便取：开盖要 > |τ_g(0.1)|=0.0386（下限位侧实测压入），关盖要 > 0.0743（上限位侧）
        {"name": "open", "steps": 400, "torque": {"lid": 0.0, "rotor": 0.0}, "control": "gravity_comp",
         "dir": +1, "bias": 0.050, "release_above": 1.711},
        {"name": "hold_open", "steps": 90, "torque": {"lid": 0.0, "rotor": 0.0}},
        {"name": "close", "steps": 600, "torque": {"lid": 0.0, "rotor": 0.0}, "control": "gravity_comp",
         "dir": -1, "bias": 0.100, "release_below": 0.001},
        {"name": "settled", "steps": 90, "torque": {"lid": 0.0, "rotor": 0.0}},
    ],
    "lid_slow": [
        # 起/终点都留在行程内部：0.10 与 1.60，两端各距限位 0.10/0.112 rad，
        # 所以"停在目标上"必须是控制器主动保持，不能靠撞限位——也才谈得上无冲击。
        {"name": "hold_at_start", "steps": 60, "control": "trajectory", "target_rad": 0.10},
        {"name": "open_slow", "steps": 420, "control": "trajectory", "target_rad": 1.60},
        {"name": "hold_open", "steps": 120, "control": "trajectory", "target_rad": 1.60},
        {"name": "close_slow", "steps": 420, "control": "trajectory", "target_rad": 0.10},
        {"name": "hold_closed", "steps": 120, "control": "trajectory", "target_rad": 0.10},
    ],
    "rotor_loop": [
        {"name": "spin", "steps": 60, "torque": {"lid": 0.0, "rotor": +0.001}},
        {"name": "coast", "steps": 60, "torque": {"lid": 0.0, "rotor": 0.0}},
        {"name": "brake", "steps": 60, "torque": {"lid": 0.0, "rotor": -0.001}},
        {"name": "after_brake", "steps": 30, "torque": {"lid": 0.0, "rotor": 0.0}},
    ],
}
IC_BY_SEQUENCE = {
    "lid_loop": {"lid": 0.700, "rotor": 0.0},     # 与 3.3.0 试段初始条件一致（rad）
    "lid_task": {"lid": 0.000, "rotor": 0.0},     # 设备真实起始：盖子关到位、压在下限位上
    "lid_qstatic": {"lid": 0.000, "rotor": 0.0},  # 同上，只是控制器换成重力前馈慢速
    "lid_slow": {"lid": 0.100, "rotor": 0.0},     # 低速轨迹：起点就定在行程内部 0.10 rad
    "rotor_loop": {"lid": 0.000, "rotor": 0.0},   # 真实工况：关盖后转转子
}
IC_Q = IC_BY_SEQUENCE[args.sequence]

if args.close_torque is not None:                 # 只有用户显式传了才覆盖，否则用段表里的定值
    for _seg in SEGMENTS[args.sequence]:
        if _seg["name"] == "close":
            _seg["torque"]["lid"] = args.close_torque

SCHEDULED = [s["name"] for s in SEGMENTS[args.sequence]]
if args.only:
    wanted = [x for x in args.only.split(",") if x]
    bad = [x for x in wanted if x not in SCHEDULED]
    if bad:
        print("未知段名 %s；可选 %s" % (bad, SCHEDULED))
        sys.exit(2)
    RUN = [s for s in SEGMENTS[args.sequence] if s["name"] in wanted]
else:
    RUN = list(SEGMENTS[args.sequence])

report = {
    "scope": "continuous closed-loop joint torque motion test of the unchanged mini pilot asset; "
             "NOT whole-scene, NOT task acceptance, NOT robot interaction",
    "supersedes": {"path": "scripts/run_mini_lid_task.py", "sha256_16": sha16(ROOT / "scripts/run_mini_lid_task.py"),
                   "reason": "阶段未调度/视图次序/位置目标冒充转速/人为刚度/复位抹掉被检状态"},
    "sequence": args.sequence, "gravity": args.gravity, "mode": args.mode,
    "scheduled_segments": SCHEDULED, "run_segments": [s["name"] for s in RUN],
    "control": {"kind": "joint scalar torque via omni.physics.tensors set_dof_actuation_forces",
                "unit": "N*m", "matches_reference_protocol": "scalar_joint_protocol",
                "drive_injected": False, "solver_iterations_overridden": False,
                "position_or_velocity_drive_used": False},
    "dt": DT, "time_steps_per_second": round(1 / DT),
    "asset_sha256_16": sha16(PILOT), "scene_sha256_16": None,
    "reference": {"file": str(REF_FILE.relative_to(ROOT)), "mujoco_version": ref["mujoco_version"],
                  "original_version_matched": ref["original_version_matched"],
                  "source_xml_sha256": ref["source_xml_sha256"],
                  "task_script_migrated": ref["task_script_migrated"]},
    "wake_mode": args.wake, "close_torque_Nm": args.close_torque,
    "slow_controller_design": None if args.sequence != "lid_slow" else dict(
        SLOW, gain_formulas="Kp=I*omega^2, Kd=2*I*zeta*omega",
        disclosure="测试控制器：增益按实测惯量与声明带宽设计，不是源设备执行器行为（源 XML 无 actuator）"),
    "gravity_feedforward_model": {
        "form": "tau_g(q) = A*sin(q) + B*cos(q) + C",
        "coefficients": None if FIT_ABC is None else {"A": FIT_ABC[0], "B": FIT_ABC[1], "C": FIT_ABC[2]},
        "fit_max_residual_Nm": FIT_RESID,
        "source": RECHECK_FILE.name,
        "endpoint_values_Nm": None if FIT_ABC is None else {
            "q_0": tau_g(0.0), "q_1.712": tau_g(1.712),
            "reference_q_0": None if not recheck else recheck["endpoints"]["q_0"]["reference_Nm"],
            "reference_q_1.712": None if not recheck else recheck["endpoints"]["q_1.712"]["reference_Nm"]},
        "note": "限位处的 2 步短采读数是含约束净响应，不是孤立重力矩；端点用拟合外推"},
    "initial_condition_q_rad": IC_Q,
    "reference_predictions": PRED, "thresholds": THRESH,
    "segments": {}, "boundaries": [], "status": None, "first_failure_layer": None,
    "chain_completed": False, "criteria_total": None, "criteria_passed": None,
    "criteria_all_passed": None, "zero_response_suspected": None,
    "errors": [], "trajectory": [],
}

# ---------------------------------------------------------------- 建场景
def build_scene():
    os.environ["CARB_VULKARS_DISABLE"] = "1"
    os.environ["TEMP"] = os.environ["TMP"] = str(OUTDIR / "_v2_tmp")
    os.makedirs(os.environ["TEMP"], exist_ok=True)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True,
                         "extra_args": ["--portable-root", str(Path.home() / "CellCultureLabRuntime"),
                                        "--/app/settings/persistent=false",
                                        "--/renderer/multiGpu/enabled=false"]})
    import omni.usd
    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
    mark("build_open_pilot")
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    sc = UsdPhysics.Scene.Define(stage, "/physicsScene")          # 与已跑通场景同名，路径不改
    sc.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr(9.81)
    api = PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim())
    api.CreateEnableGPUDynamicsAttr(False)
    api.CreateBroadphaseTypeAttr("MBP")
    api.CreateTimeStepsPerSecondAttr(round(1 / DT))               # 正式 API + UInt，不裸写 Float

    mini = stage.DefinePrim("/Mini", "Xform")
    mini.GetReferences().AddReference(str(PILOT).replace("\\", "/"))   # 绝对路径，建的时候就一定能解析
    stage.SetDefaultPrim(mini.GetPrim())      # mini 摆在世界原点，不写零平移（零平移是无意义 op）

    # 地面：先量 mini 的实际世界包围盒，再把顶面放到最低点下方 clearance 处
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy], False)
    lo, hi = None, None
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith("/Mini") and str(prim.GetTypeName()) in \
                ("Mesh", "Cube", "Cylinder", "Sphere", "Capsule", "Cone"):
            rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            a, b = rng.GetMin(), rng.GetMax()
            lo = list(a) if lo is None else [min(x, y) for x, y in zip(lo, a)]
            hi = list(b) if hi is None else [max(x, y) for x, y in zip(hi, b)]
    g_top = lo[2] - args.ground_clearance
    THICK = 0.1
    g = UsdGeom.Cube.Define(stage, "/Ground")
    g.CreateSizeAttr(2.0)
    g.AddTransformOp().Set(Gf.Matrix4d(20, 0, 0, 0, 0, 20, 0, 0, 0, 0, THICK / 2, 0,
                                       0, 0, g_top - THICK / 2, 1))
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())

    stage.GetRootLayer().Export(str(SCENE))
    mark("build_exported", mini_world_min=lo, ground_top_z=g_top)

    # 导出之后重新打开核对，不看脚本看产物
    ctx.open_stage(str(SCENE))
    st2 = ctx.get_stage()
    rng_g = cache.ComputeWorldBound(st2.GetPrimAtPath("/Ground")).ComputeAlignedRange()
    lo2 = None
    for prim in st2.Traverse():
        if str(prim.GetPath()).startswith("/Mini") and str(prim.GetTypeName()) in \
                ("Mesh", "Cube", "Cylinder", "Sphere", "Capsule", "Cone"):
            r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            lo2 = list(r.GetMin()) if lo2 is None else [min(x, y) for x, y in zip(lo2, r.GetMin())]
    scene_prim = st2.GetPrimAtPath("/physicsScene")
    ps = PhysxSchema.PhysxSceneAPI(scene_prim)
    verify = {"scene": str(SCENE), "scene_sha256_16": sha16(SCENE),
              "unresolved_paths": ("missing_api_TraverseUnresolvedReferencePaths"
                                   if not hasattr(Usd, "TraverseUnresolvedReferencePaths")
                                   else [str(pp.GetPath()) for pp in Usd.TraverseUnresolvedReferencePaths(st2)]),
              "composition_errors": composition_errors(st2),
              "rigid_bodies": sum(1 for p in st2.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)),
              "revolute_joints": sum(1 for p in st2.Traverse()
                                     if p.IsA(UsdPhysics.RevoluteJoint)),
              "fixed_joints": sum(1 for p in st2.Traverse() if p.IsA(UsdPhysics.FixedJoint)),
              "articulation_roots": [str(p.GetPath()) for p in st2.Traverse()
                                     if p.HasAPI(UsdPhysics.ArticulationRootAPI)],
              "physics_scene": {"path": str(scene_prim.GetPath()),
                                "applied_schemas": [str(x) for x in scene_prim.GetAppliedSchemas()],
                                "gravity_magnitude": UsdPhysics.Scene(scene_prim).GetGravityMagnitudeAttr().Get(),
                                "timeStepsPerSecond": ps.GetTimeStepsPerSecondAttr().Get(),
                                "timeStepsPerSecond_type": "UInt",
                                "enableGPUDynamics": ps.GetEnableGPUDynamicsAttr().Get(),
                                "broadphaseType": ps.GetBroadphaseTypeAttr().Get()},
              "ground": {"world_min": list(rng_g.GetMin()), "world_max": list(rng_g.GetMax()),
                         "mini_world_min": lo2,
                         "clearance_m": round(lo2[2] - rng_g.GetMax()[2], 6),
                         "initial_overlap": bool(rng_g.GetMax()[2] - lo2[2] > 0)}}
    (OUTDIR / "mini_closed_loop_v2_build.json").write_text(
        json.dumps(verify, ensure_ascii=False, indent=1), encoding="utf-8")
    mark("build_verified", overlap=verify["ground"]["initial_overlap"],
         composition_errors=verify["composition_errors"])
    print("BUILD_VERIFY " + json.dumps(verify, ensure_ascii=False), flush=True)
    app.close()
    return verify


if args.build_only:
    # 实测教训：build 里一条 Python 异常没接住，收尾阶段就会变成 rtx access violation，
    # 退出码不再能定位失败层。这里显式捕获并落盘，再由干净的 app.close() 退出。
    try:
        verify = build_scene()
    except Exception as exc:
        mark("build_failed", error="%s: %s" % (type(exc).__name__, exc))
        (OUTDIR / "mini_closed_loop_v2_build.json").write_text(
            json.dumps({"error": "%s: %s" % (type(exc).__name__, exc),
                        "traceback": traceback.format_exc().splitlines()[-12:],
                        "marks": MARKS}, ensure_ascii=False, indent=1), encoding="utf-8")
        print("BUILD_FAILED " + repr(exc), flush=True)
        sys.exit(3)
    sys.exit(0)

# ---------------------------------------------------------------- 跑连续闭环
os.environ["TEMP"] = os.environ["TMP"] = str(OUTDIR / "_v2_tmp")
os.makedirs(os.environ["TEMP"], exist_ok=True)
if not SCENE.is_file():
    print("先跑 --build-only 生成场景")
    sys.exit(2)
report["scene"] = str(SCENE)
report["scene_sha256_16"] = sha16(SCENE)

from isaacsim import SimulationApp
app = SimulationApp({"headless": True,
                     "extra_args": ["--portable-root", str(Path.home() / "CellCultureLabRuntime"),
                                    "--/app/settings/persistent=false",
                                    "--/renderer/multiGpu/enabled=false"]})
import numpy as np
import omni.usd, omni.physx
import omni.physics.tensors as tensors
from pxr import UsdPhysics, PhysxSchema, Gf
sys.path.insert(0, str(ROOT / "scripts"))
from runtime_joint_math import joint_constraint_error

view = None
iface = None
resets_mid_run = 0


def _json_default(o):
    """numpy 标量/数组、Gf 类型一律转成 Python 原生；转不了就报错，不静默丢字段。"""
    try:
        import numpy as _np
        if isinstance(o, (_np.floating, _np.integer)):
            return float(o)
        if isinstance(o, _np.ndarray):
            return o.tolist()
    except Exception:
        pass
    for attr in ("tolist",):
        if hasattr(o, attr):
            return getattr(o, attr)()
    return str(o)


def flush():
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=1, allow_nan=False,
                                      default=_json_default), encoding="utf-8")


try:
    ctx = omni.usd.get_context()
    iface = omni.physx.get_physx_interface()
    mark("layer_open_stage")
    iface.reset_simulation()
    tensors.reset()
    if not ctx.open_stage(str(SCENE)):
        raise RuntimeError("open_stage_failed")
    stage = ctx.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    scene_prim = stage.GetPrimAtPath("/physicsScene")
    if not scene_prim:
        raise RuntimeError("no_physicsScene")
    sc = UsdPhysics.Scene(scene_prim)
    mag = 9.81 if args.gravity == "real" else 0.0
    sc.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr(mag)
    ps = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    ps.CreateEnableGPUDynamicsAttr(args.mode == "gpu")
    ps.CreateBroadphaseTypeAttr("MBP")
    ps.CreateTimeStepsPerSecondAttr(round(1 / DT))
    report["configured"] = {"gravity_magnitude": mag,
                            "gpu_dynamics_readback": ps.GetEnableGPUDynamicsAttr().Get(),
                            "broadphase_readback": ps.GetBroadphaseTypeAttr().Get(),
                            "timeStepsPerSecond_readback": ps.GetTimeStepsPerSecondAttr().Get()}
    # 模式标签必须与引擎读回一致，否则 --mode gpu 不能当 GPU 运行证据
    if bool(report["configured"]["gpu_dynamics_readback"]) != (args.mode == "gpu"):
        raise RuntimeError("gpu_mode_label_mismatch")

    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    report["revolute_joint_count"] = len(joints)
    # 接触状态：本脚本自始至终没有关过任何碰撞；把"有没有关"写成可读回的事实而不是注释。
    colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    report["contact_config"] = {
        "colliders_composed": len(colliders),
        "colliders_with_contact_enabled_false": [str(p.GetPath()) for p in colliders
                                                 if p.GetAttribute("physxCollision:contactEnabled") and
                                                 p.GetAttribute("physxCollision:contactEnabled").Get() is False],
        "collision_groups": [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.CollisionGroup)],
        "kinematic_rigid_bodies": [str(p.GetPath()) for p in stage.Traverse()
                                   if p.HasAPI(UsdPhysics.RigidBodyAPI) and
                                   p.GetAttribute("physics:kinematicEnabled").Get()],
        "contacts_disabled_by_this_script": False}
    report["authored_joint_drives"] = {}
    for j in joints:
        report["authored_joint_drives"][str(j.GetPath())] = {
            "stiffness": j.GetAttribute("drive:angular:physics:stiffness").Get(),
            "damping": j.GetAttribute("drive:angular:physics:damping").Get(),
            "limits": [j.GetAttribute("physics:lowerLimit").Get(),
                       j.GetAttribute("physics:upperLimit").Get()]}
    mark("layer_start_simulation")
    iface.start_simulation()
    iface.update_simulation(DT, 0)
    mark("layer_create_simulation_view")
    view = tensors.create_simulation_view("numpy")
    if view is None:
        raise RuntimeError("simulation_view_None")
    art = view.create_articulation_view("/Mini/Joints/world_fixed")
    if art is None:
        raise RuntimeError("articulation_view_None")
    names = list(art.shared_metatype.dof_names)
    report["invariants"] = {"count": art.count, "fixed_base": bool(art.shared_metatype.fixed_base),
                            "max_dofs": art.max_dofs, "dof_names": names}
    if art.count != 1 or not art.shared_metatype.fixed_base or art.max_dofs != 2:
        raise RuntimeError("articulation_invariants_failed")
    IDX = {}
    for want in ("lid", "rotor"):
        hit = [i for i, n in enumerate(names) if n.endswith("_" + want)]
        if not hit:
            raise RuntimeError("no_dof_for_" + want)
        IDX[want] = hit[0]
    report["lid_rotor_mapping"] = IDX
    bodies = view.create_rigid_body_view("/Mini/Bodies/*")
    idx_arr = np.array([0], dtype=np.uint32)
    qbuf = np.zeros((1, art.max_dofs), dtype=np.float32)
    vbuf = np.zeros((1, art.max_dofs), dtype=np.float32)
    fbuf = np.zeros((1, art.max_dofs), dtype=np.float32)
    IC_VEC = [0.0] * art.max_dofs
    for k, i in IDX.items():
        qbuf[0, i] = IC_VEC[i] = IC_Q[k]
    art.set_dof_positions(qbuf, idx_arr)
    art.set_dof_velocities(vbuf, idx_arr)
    q = art.get_dof_positions().copy()[0].tolist()
    if any(not math.isfinite(x) for x in q):
        raise RuntimeError("ic_readback_not_finite")
    report["ic_readback_q_rad"] = [nz(x) for x in q]
    mark("loop_begin", segments=[s["name"] for s in RUN],
         ic_q_deg=[nz(math.degrees(x), 4) for x in q])

    if args.record_steps == "full":
        steps_path = step_record({"kind": "header",
                                  "dof_names": names, "dof_mapping": IDX, "dt": DT,
                                  "time_steps_per_second": round(1 / DT),
                                  "gravity_magnitude": mag, "mode": args.mode,
                                  "scene_sha256_16": report["scene_sha256_16"],
                                  "asset_sha256_16": report["asset_sha256_16"],
                                  "reference": report["reference"],
                                  "control_design": SLOW,
                                  "numeric_precision": "float32 底层读数与写入缓冲区原样转记，不做额外四舍五入；"
                                                        "转 JSON 不增加精度",
                                  "field_semantics": "q_pre/v_pre 是本步推进前读数；torque_Nm 是真正写进 "
                                                     "set_dof_actuation_forces 的向量（float32 缓冲区值）；"
                                                     "q_post/v_post 是推进后读数"})
        report["step_record_file"] = str(steps_path.name)

    WAKE = None
    if args.wake == "each_step":
        all_idx = np.arange(bodies.count, dtype=np.uint32)
        for form, fn in (("noarg", lambda: bodies.wake_up()),
                         ("indices", lambda: bodies.wake_up(all_idx))):
            try:
                fn(); WAKE = form; break
            except Exception as exc:
                report.setdefault("wake_api_attempts", []).append("%s: %s" % (form, str(exc)[:120]))
        report["wake_api"] = WAKE
        if WAKE is None:
            raise RuntimeError("no usable RigidBodyView.wake_up signature")
    step_global = 0
    prev_boundary_q = None
    prev_boundary_name = None
    for seg in RUN:
        sname = seg["name"]
        seg_torque = seg.get("torque", {"lid": 0.0, "rotor": 0.0})   # 轨迹段没有常值力矩
        for i in range(art.max_dofs):
            fbuf[0, i] = 0.0
        for k, i in IDX.items():
            fbuf[0, i] = seg_torque[k]
        rec = {"segment": sname, "steps": seg["steps"], "torque_Nm": dict(seg_torque),
               "q_start_rad": None, "q_end_rad": None, "v_start": None, "v_end": None,
               "min_abs_q_lid": None, "max_abs_q_lid": None, "samples": [], "read_failed": False}
        # 角度触发释放：不再靠"我猜要推多少步"，而是推到指定角度就把力矩撤掉（闭环按状态切换）。
        trig_key = next((k for k in ("release_above", "release_below") if k in seg), None)
        trig_done = False
        rec["release_rule"] = {"kind": trig_key, "target_rad": seg.get(trig_key) if trig_key else None,
                               "triggered_at_step": None, "q_at_trigger_rad": None}
        # 重力前馈补偿控制器：τ_applied = -τ_g(q) ± bias。τ_g 来自实测曲线（前馈自测量，不是调出来的增益），
        # 目的是"慢慢开/慢慢关"，而不是用 0.2 N·m 把盖子甩到限位上。
        gcomp = seg.get("control") == "gravity_comp"
        traj = seg.get("control") == "trajectory"
        refs = None
        if traj:
            rec["control"] = ("test_controller: 参考梯形轨迹 + 重力/惯性前馈 + 位置/速度反馈（增益见 design）"
                              " | target=%.3f rad" % seg["target_rad"])
            rec["max_tracking_error_rad"] = 0.0
            rec["torque_saturation_steps"] = 0
            rec["min_distance_to_stops_rad"] = [9.9, 9.9]
        else:
            rec["control"] = ("gravity_feedforward(bias=%g N*m)" % seg.get("bias", 0.0)) if gcomp else "constant_torque"
        rec["peak_abs_v_rad_s"] = 0.0
        rec["applied_torque_max_abs"] = 0.0
        for s_i in range(seg["steps"] + 1):
            qn = art.get_dof_positions().copy()[0].tolist()
            vn = art.get_dof_velocities().copy()[0].tolist()
            if any(not math.isfinite(x) for x in qn + vn):
                rec["read_failed"] = True
                raise RuntimeError("non_finite_read_%s_%d" % (sname, s_i))
            rec["peak_abs_v_rad_s"] = max(rec["peak_abs_v_rad_s"], max(abs(x) for x in vn))
            if gcomp and not trig_done:
                fbuf[0, IDX["lid"]] = -tau_g(qn[IDX["lid"]]) + seg["dir"] * seg["bias"]
                rec["applied_torque_max_abs"] = max(rec["applied_torque_max_abs"], float(abs(fbuf[0, IDX["lid"]])))
            if traj:
                if refs is None:      # 起点用**当前实际角度**，不是我以为的初值，也不是复位出来的
                    refs, t_total = trapezoid(qn[IDX["lid"]], seg["target_rad"], seg["steps"],
                                              SLOW["v_design_rad_s"], SLOW["a_design_rad_s2"], DT)
                    rec["control"] += ", t_total=%.3f s" % t_total
                qr, vr, ar = refs[min(s_i, len(refs) - 1)]
                li = IDX["lid"]
                tau = SLOW["I_used_kg_m2"] * ar - tau_g(qr)                     + SLOW["Kp_Nm_per_rad"] * (qr - qn[li]) + SLOW["Kd_Nm_s_per_rad"] * (vr - vn[li])
                if abs(tau) > SLOW["tau_sat_Nm"]:
                    rec["torque_saturation_steps"] += 1
                    tau = math.copysign(SLOW["tau_sat_Nm"], tau)
                fbuf[0, li] = tau
                rec["applied_torque_max_abs"] = max(rec["applied_torque_max_abs"], abs(tau))
                rec["max_tracking_error_rad"] = max(rec["max_tracking_error_rad"], abs(qr - qn[li]))
                rec["min_distance_to_stops_rad"] = [
                    min(rec["min_distance_to_stops_rad"][0], abs(qn[li] - 0.0)),
                    min(rec["min_distance_to_stops_rad"][1], abs(1.712 - qn[li]))]
                rec.setdefault("last_cmd", {})
                rec["last_cmd"] = {"q_ref_rad": nz(qr), "v_ref_rad_s": nz(vr), "tau_cmd_Nm": nz(tau)}
            if rec["q_start_rad"] is None:
                rec["q_start_rad"], rec["v_start"] = qn, vn
                if prev_boundary_q is not None:
                    delta = max(abs(a - b) for a, b in zip(qn, prev_boundary_q))
                    acc = max(abs(x - y) for x, y in zip(qn, IC_VEC))
                    report["boundaries"].append({"from": prev_boundary_name, "to": sname,
                                                 "max_abs_dq_at_boundary_rad": delta,
                                                 "state_carried_over": delta < 1e-9,
                                                 "accumulated_vs_initial_condition_rad": acc})
            rec["q_end_rad"], rec["v_end"] = qn, vn
            if trig_key and not trig_done and s_i >= 1:
                ql = qn[IDX["lid"]]
                crossed = ql > seg[trig_key] if trig_key == "release_above" else ql < seg[trig_key]
                if crossed:
                    trig_done = True
                    rec["release_rule"]["triggered_at_step"] = s_i
                    rec["release_rule"]["q_at_trigger_rad"] = nz(ql)
                    for k, i in IDX.items():
                        fbuf[0, i] = 0.0            # 之后每一步都是零输入，靠重力/接触自己走
            if s_i % 10 == 0 or s_i == seg["steps"]:
                rec["samples"].append({"step": s_i, "q": [nz(x) for x in qn], "v": [nz(x) for x in vn]})
            if step_global % 20 == 0:
                poses = dict(zip(bodies.prim_paths, bodies.get_transforms().copy().tolist()))
                xyzw = lambda x: [*x.GetImaginary(), x.GetReal()]
                worst = 0.0
                for j in joints:
                    b0 = str(j.GetRelationship("physics:body0").GetTargets()[0])
                    b1 = str(j.GetRelationship("physics:body1").GetTargets()[0])
                    if b0 not in poses or b1 not in poses:
                        raise RuntimeError("body_missing_from_view")
                    e = joint_constraint_error("RevoluteJoint", poses[b0], poses[b1],
                                               list(j.GetAttribute("physics:localPos0").Get()),
                                               list(j.GetAttribute("physics:localPos1").Get()),
                                               xyzw(j.GetAttribute("physics:localRot0").Get()),
                                               xyzw(j.GetAttribute("physics:localRot1").Get()), "X")
                    worst = max(worst, e["position_m"])
                report["trajectory"].append({"step": step_global, "seg": sname, "q": [nz(x) for x in qn],
                                             "v": [nz(x) for x in vn], "anchor_err_m": nz(worst, 9)})
            if s_i < seg["steps"]:
                if WAKE is not None:
                    (bodies.wake_up() if WAKE == "noarg" else bodies.wake_up(all_idx))
                tau_applied = [float(x) for x in fbuf[0]]
                art.set_dof_actuation_forces(fbuf, idx_arr)
                t_before = (step_global + 1) * DT
                iface.update_simulation(DT, t_before)
                if args.record_steps == "full":
                    qp = art.get_dof_positions().copy()[0].tolist()
                    vp = art.get_dof_velocities().copy()[0].tolist()
                    dv = [abs(b - a2) for a2, b in zip(vn, vp)]
                    rec["max_abs_dv_per_step_rad_s"] = max(rec.get("max_abs_dv_per_step_rad_s", 0.0),
                                                           max(dv))
                    STEP_COUNT += 1
                    step_write({"step": step_global, "segment": sname, "t_sim_s": t_before,
                                "q_pre": [float(x) for x in qn], "v_pre": [float(x) for x in vn],
                                "torque_Nm": tau_applied,
                                "q_post": [float(x) for x in qp], "v_post": [float(x) for x in vp]})
                step_global += 1
        lid0 = rec["q_start_rad"][IDX["lid"]]
        lid1 = rec["q_end_rad"][IDX["lid"]]
        rec["dq_lid_deg"] = round(math.degrees(lid1 - lid0), 5)
        rec["dq_rotor_deg"] = round(math.degrees(rec["q_end_rad"][IDX["rotor"]] -
                                                 rec["q_start_rad"][IDX["rotor"]]), 5)
        rec["dq_lid_from_ic_deg"] = round(math.degrees(lid1 - IC_Q["lid"]), 5)
        net = max([abs(seg_torque[k]) for k in IDX] + [rec["applied_torque_max_abs"]])
        still = max(abs(rec["q_end_rad"][i] - rec["q_start_rad"][i]) for i in IDX.values())
        rec["commanded_torque_max_abs"] = net
        if rec["release_rule"]["triggered_at_step"] is not None:
            rec["release_rule"]["dq_after_release_deg"] = round(math.degrees(
                rec["q_end_rad"][IDX["lid"]] - rec["release_rule"]["q_at_trigger_rad"]), 5)
            rec["release_rule"]["steps_after_release"] = seg["steps"] - rec["release_rule"]["triggered_at_step"]
        rec["release_rule"]["reached"] = (trig_key is None) or bool(trig_done)
        rec["zero_response_suspected"] = bool(net > 1e-6 and still < 1e-9 and seg["steps"] >= 30)
        if rec["zero_response_suspected"]:
            report["suspicious_no_response"] = (report.get("suspicious_no_response") or []) + [sname]
        report["segments"][sname] = rec
        prev_boundary_q, prev_boundary_name = rec["q_end_rad"], sname
        flush()
        mark("segment_done", segment=sname, dq_lid_deg=rec["dq_lid_deg"],
             dq_rotor_deg=rec["dq_rotor_deg"], v_end=[nz(x, 5) for x in rec["v_end"]])

    if STEP_FH[0] is not None:
        STEP_FH[0].close()
        STEP_FH[0] = None
    report["step_records_written"] = STEP_COUNT
    report["status"] = "chain_complete"   # 只看链路；判据汇总在算完 V 之后补
    report["steps_executed"] = step_global
    report["resets_mid_run"] = resets_mid_run
    # ------------------------------------------------------------- 判据（预先声明）
    V = {}
    seg = report["segments"]
    V["one_continuous_run_no_reset"] = (resets_mid_run == 0 and step_global > 0)
    V["state_carried_across_boundaries"] = bool(report["boundaries"]) and \
        all(b["state_carried_over"] for b in report["boundaries"])
    if args.sequence == "lid_loop":
        LI = IDX["lid"]
        if "open" in seg:
            dq = seg["open"]["dq_lid_deg"]
            refdq = THRESH["lid_open_zero_g"]["ref_dq_deg"]
            V["open_moves_in_commanded_direction"] = dq > THRESH["lid_open_zero_g"]["min_dq_deg"]
            if args.gravity == "none":
                V["open_matches_330_within_tol"] = abs(dq - refdq) <= abs(refdq) * THRESH["lid_open_zero_g"]["tol_rel"]
        if "brake" in seg:
            b = seg["brake"]
            V["brake_nulls_residual_velocity"] = abs(b["v_end"][LI]) < THRESH["lid_brake_nulls_velocity"]["end_abs_v_lt_rad_s"]
            # 制动段物理上就是"还在朝开方向走、但速度往 0 收"，所以查速度不查位移符号。
            # 旧名 brake_reverses_dq_sign_vs_open 名字与含义不符，已弃用。
            V["brake_reduces_speed_toward_zero"] = (abs(b["v_end"][LI]) < abs(b["v_start"][LI]) and
                                                    b["v_start"][LI] > 0.05 and b["dq_lid_deg"] > 0)
        if "hold" in seg:
            h = seg["hold"]
            held_still = abs(h["dq_lid_deg"]) < THRESH["lid_hold_static_zero_g"]["max_abs_dq_deg"] and \
                abs(h["v_end"][LI]) < THRESH["lid_hold_static_zero_g"]["max_abs_v_rad_s"]
            if args.gravity == "none":
                V["hold_is_static_after_brake"] = held_still
            else:
                # 勘误：这条判据放错了位置。brake 结束时盖子已经贴在上限位 1.712，
                # 那里"零输入继续朝开"没有行程可走，所以原 hold_falls_open_without_latch...
                # 在这种几何下必然 false，它测的不是"缺插销"。真正考同一件事的是
                # lid_slow 的悬空保持（离两端限位都 >0.098 rad）。
                V["hold_rests_at_open_stop_gravity_limited"] = (abs(h["dq_lid_deg"]) < 0.05
                                                                and h["q_end_rad"][LI] >= math.radians(98.09) - 1e-4)
                V["hold_is_static_after_brake"] = held_still      # 真实重力下预期 False：靠限位而非插销

        if "close" in seg:
            c = seg["close"]
            V["close_reverses_sign"] = c["dq_lid_deg"] < -THRESH["lid_hold_static_zero_g"]["max_abs_dq_deg"]
            V["close_reaches_authored_lower_stop"] = \
                math.degrees(c["q_end_rad"][LI]) < THRESH["lid_close"]["must_reach_lower_stop_deg_lt"]
            V["close_ends_at_rest"] = abs(c["v_end"][LI]) < THRESH["lid_close"]["end_abs_v_lt_rad_s"]
            if args.gravity == "real":
                _ct = abs(report["segments"]["close"]["torque_Nm"]["lid"])
                V["close_torque_exceeds_gravity_torque"] = _ct > THRESH["lid_close"]["torque_must_exceed_gravity"]
        if "open" in seg and "close" in seg:
            V["net_cycle_ends_below_where_it_started"] = (seg["close"]["q_end_rad"][LI] <
                                                          seg["open"]["q_start_rad"][LI] + THRESH["sign_eps_rad"])
    elif args.sequence == "lid_qstatic":
        LI = IDX["lid"]
        # 判据来自同一次实测曲线 + 与 lid_task(0.2 N·m 甩开) 的峰值角速对照，跑前定死
        PEAK_V = 2.0
        o, h, c, st = (seg.get("open"), seg.get("hold_open"), seg.get("close"), seg.get("settled"))
        if o:
            V["qstatic_open_reaches_upper_stop"] = o["q_end_rad"][LI] >= math.radians(98.09) - 1e-4
            V["qstatic_open_trigger_reached"] = bool(o["release_rule"]["reached"])
            V["qstatic_open_peak_v_under_2rad_s"] = o["peak_abs_v_rad_s"] < PEAK_V
        if h:
            V["qstatic_held_open_static_zero_input"] = (abs(h["dq_lid_deg"]) < 0.05
                                                        and abs(h["v_end"][LI]) < 0.01)
        if c:
            V["qstatic_close_reaches_lower_stop"] = abs(c["q_end_rad"][LI]) < 1e-3
            V["qstatic_close_trigger_reached"] = bool(c["release_rule"]["reached"])
            V["qstatic_close_peak_v_under_2rad_s"] = c["peak_abs_v_rad_s"] < PEAK_V
        if st:
            V["qstatic_ends_closed_at_rest"] = (abs(st["q_end_rad"][LI]) < 1e-3
                                                and abs(st["v_end"][LI]) < 0.01)
        if o and c:
            V["qstatic_full_cycle_state_returns"] = abs(c["q_end_rad"][LI] - o["q_start_rad"][LI]) < 1e-3
            report["peak_v_qstatic_vs_slam_rad_s"] = {
                "qstatic_open": o["peak_abs_v_rad_s"], "qstatic_close": c["peak_abs_v_rad_s"]}
    elif args.sequence == "lid_slow":
        LI = IDX["lid"]
        allseg = list(report["segments"].values())
        V["slow_endpoints_inside_travel_by_design"] = all(
            SLOW["stop_margin_rad"] <= sg["target_rad"] <= 1.712 - SLOW["stop_margin_rad"]
            for sg in SEGMENTS["lid_slow"] if "target_rad" in sg)
        peak_v = max(sg["peak_abs_v_rad_s"] for sg in allseg)
        V["slow_peak_v_le_1p2_rad_s"] = peak_v <= 1.2
        V["slow_peak_v_below_hard_cap_2rad_s"] = peak_v <= SLOW["v_hard_cap_rad_s"]
        V["slow_never_touches_stops"] = min(min(sg["min_distance_to_stops_rad"]) for sg in allseg
                                            if "min_distance_to_stops_rad" in sg) >= 1e-4
        V["slow_tracking_error_le_10mrad"] = max(sg["max_tracking_error_rad"] for sg in allseg
                                                 if "max_tracking_error_rad" in sg) <= SLOW["pos_tol_rad"]
        V["slow_no_torque_saturation"] = sum(sg["torque_saturation_steps"] for sg in allseg
                                             if "torque_saturation_steps" in sg) == 0
        for tag, tgt in (("hold_at_start", 0.10), ("hold_open", 1.60), ("hold_closed", 0.10)):
            h = seg.get(tag)
            if h:
                e = abs(h["q_end_rad"][LI] - tgt)
                V["%s_position_within_5mrad" % tag] = e <= SLOW["hold_pos_tol_rad"]
                V["%s_velocity_within_20mrad_s" % tag] = abs(h["v_end"][LI]) <= SLOW["hold_v_tol_rad_s"]
                # 保持段必须有非零保持力矩：证明"停住"是控制器做的，不是撞在限位上
                # 保持所需的指令力矩 = −g(q)（与重力大小相等、方向相反）
                V["%s_holding_torque_counters_gravity" % tag] = abs(
                    h["last_cmd"]["tau_cmd_Nm"] + tau_g(tgt)) <= SLOW["tau_hold_tol_Nm"]
                V["%s_holding_torque_is_nonzero" % tag] = abs(h["last_cmd"]["tau_cmd_Nm"]) > 5e-3
        o = seg.get("open_slow"); c = seg.get("close_slow")
        if o and c:
            V["slow_open_travel_matches_plan"] = abs(o["dq_lid_deg"] - math.degrees(1.50)) < 2.0
            V["slow_close_returns_to_start"] = abs(c["q_end_rad"][LI] - o["q_start_rad"][LI]) < SLOW["pos_tol_rad"]
            dvs = [sg.get("max_abs_dv_per_step_rad_s", 0.0) for sg in allseg]
            max_dv = max(dvs) if dvs else None
            report["max_abs_dv_per_step_rad_s"] = max_dv
            # 实算相邻步速度差（含阶段边界，因为整段是同一连续循环，边界处不推进物理步所以不会产生假跳变）。
            # 设计预期：a_design=20 rad/s² × dt=1/240 → 0.0833 rad/s/步；门槛 1 rad/s/步 等价 240 rad/s²，
            # 也就是"不许出现冲击级跳变"。以前这条是硬写 True，等于没测，已撤回。
            V["slow_no_velocity_jump_beyond_1rad_s_per_step"] = (
                max_dv is not None and max_dv <= SLOW["dv_jump_max_rad_s_per_step"])
            V["slow_dv_within_design_expectation"] = (
                max_dv is not None and max_dv <= SLOW["a_design_rad_s2"] * DT * 3.0)
        report["slow_summary"] = {"peak_v_rad_s": peak_v,
                                  "max_tracking_error_rad": max(sg.get("max_tracking_error_rad", 0.0) for sg in allseg),
                                  "min_stop_distance_rad": min(min(sg["min_distance_to_stops_rad"])
                                                               for sg in allseg
                                                               if "min_distance_to_stops_rad" in sg)}
    elif args.sequence == "lid_task":
        LI = IDX["lid"]
        bal, tau_press = GRAV_BALANCE_Q, GRAV_MAX_OPEN_TORQUE
        if bal is None or tau_press is None:
            report["thresholds_missing"] = "缺少 mini_lid_gravity_torque_curve.json，任务判据无法成立"
        q_close_start = seg.get("close", {}).get("q_start_rad", [None])[0]
        if "open" in seg:
            o = seg["open"]
            V["open_lifts_lid_off_lower_stop"] = o["dq_lid_deg"] > 1.0
            V["open_reaches_release_angle_under_torque"] = bool(o["release_rule"]["reached"])
        _o_trig = seg.get("open", {}).get("release_rule", {})
        _dq_after_open = ((_o_trig.get("dq_after_release_deg") or 0.0)
                          + (seg.get("release_after_open", {}).get("dq_lid_deg") or 0.0))

        if "release_after_open" in seg or _o_trig:
            r = seg.get("release_after_open", seg.get("open"))
            V["lid_reaches_open_stop"] = r["q_end_rad"][LI] >= math.radians(98.09) - 1e-4
            V["gravity_moves_lid_the_rest_of_the_way_with_zero_input"] = (
                bool(_o_trig.get("reached")) and _dq_after_open > 1.0)
        if "hold_open" in seg:
            h = seg["hold_open"]
            V["held_open_at_stop_without_torque"] = (abs(h["dq_lid_deg"]) < 0.05 and
                                                     abs(h["v_end"][LI]) < 0.01)
        if "close" in seg:
            c = seg["close"]
            ct = abs(c["torque_Nm"]["lid"])
            V["close_torque_exceeds_measured_press_in"] = (tau_press is not None and ct > tau_press)
            V["close_breaks_away_from_open_stop"] = c["dq_lid_deg"] < -1.0
            V["close_reaches_release_angle_under_torque"] = bool(c["release_rule"]["reached"])
            V["close_gets_below_balance_point"] = (bal is not None and c["q_end_rad"][LI] < bal)
        if "release_after_close" in seg:
            r = seg["release_after_close"]
            V["gravity_finishes_closing_with_zero_input"] = (abs(r["q_end_rad"][LI]) < 1e-3 and
                                                             abs(r["v_end"][LI]) < 0.05)
            V["ends_fully_closed"] = math.degrees(r["q_end_rad"][LI]) < 0.05
        if "open" in seg and "release_after_close" in seg:
            V["full_cycle_start_equals_end_state"] = (abs(seg["release_after_close"]["q_end_rad"][LI] -
                                                          seg["open"]["q_start_rad"][LI]) < 1e-3)
    else:
        if "spin" in seg:
            sp = seg["spin"]
            V["spin_sign_follows_torque"] = sp["dq_rotor_deg"] > THRESH["rotor_spin"]["min_dq_deg"]
            refq = math.degrees(PRED["rotor_spin_60"]["q"])
            V["spin_matches_330_within_tol"] = abs(sp["dq_rotor_deg"] - refq) <= abs(refq) * THRESH["rotor_spin"]["tol_rel"]
        if "coast" in seg and "spin" in seg:
            v_sp = abs(seg["spin"]["v_end"][IDX["rotor"]])
            v_co = abs(seg["coast"]["v_end"][IDX["rotor"]])
            V["coast_keeps_speed_no_hidden_damping"] = (v_sp > 0 and
                                                        abs(v_co - v_sp) / v_sp <= THRESH["rotor_coast_no_hidden_damping"]["max_v_decay_rel"])
        if "brake" in seg:
            V["brake_stops_rotor"] = abs(seg["brake"]["v_end"][IDX["rotor"]]) < \
                THRESH["rotor_brake_stops"]["end_abs_v_lt_rad_s"]
        if "after_brake" in seg and "brake" in seg:
            V["stopped_state_persists"] = abs(seg["after_brake"]["v_end"][IDX["rotor"]]) < \
                THRESH["rotor_brake_stops"]["end_abs_v_lt_rad_s"] * 3
    for name in SCHEDULED:
        if name not in V:
            V[name] = "not_scheduled" if name not in seg else "measured_no_threshold"
    if any(not report["segments"].get(s["name"], {}).get("samples") for s in RUN):
        V["read_integrity"] = "read_failed"
    report["verdict"] = V
    if report.get("suspicious_no_response"):
        V["zero_response_segments"] = report["suspicious_no_response"]
        V["do_not_read_as_held_or_closed"] = ("有非零指令力矩但位移精确为 0，先判为可疑无响应，"
                                             "不能当作「已保持/已关到位」；需区分休眠/限位粘连/指令未送达")
    booleans = [v for v in V.values() if isinstance(v, bool)]
    report["chain_completed"] = True
    report["criteria_total"] = len(booleans)
    report["criteria_passed"] = sum(1 for v in booleans if v)
    report["criteria_all_passed"] = (report["criteria_passed"] == len(booleans))
    report["zero_response_suspected"] = bool(report.get("suspicious_no_response"))
    # status 只说明链路是否跑完；动作是否合格一律看 criteria_*
    report["status"] = "chain_complete"
    report["verdict_note"] = "只有链路与边界连续性为真时，动作判定才有意义；未跑段一律 not_scheduled，不写 0"
    flush()
except Exception as exc:
    report["chain_completed"] = False
    report["criteria_all_passed"] = None
    report["status"] = ("init_failed" if not report.get("segments")
                        else "chain_incomplete_partial_motion")
    report["first_failure_layer"] = MARKS[-1]["layer"] if MARKS else "before_any_mark"
    report["errors"].append("%s: %s" % (type(exc).__name__, str(exc)[:300]))
    report["traceback_tail"] = traceback.format_exc().splitlines()[-8:]
    flush()
finally:
    mark("layer_cleanup_begin")
    if STEP_FH[0] is not None:
        STEP_FH[0].close()
        STEP_FH[0] = None
        report["step_records_written"] = STEP_COUNT
        report["step_records_flushed_early"] = True
    for fn in (lambda: view.invalidate(), lambda: iface.reset_simulation()):
        try:
            fn()
        except Exception as exc:
            report["cleanup_error"] = str(exc)[:160]
    try:
        flush()
    except Exception as exc:
        print("FLUSH_FAILED " + repr(exc), flush=True)
    mark("layer_before_app_close")
    app.close()

print(json.dumps({"status": report["status"], "first_failure_layer": report["first_failure_layer"],
                  "verdict": report.get("verdict"), "errors": report["errors"],
                  "boundaries": report["boundaries"],
                  "configured": report.get("configured"),
                  "invariants": report.get("invariants")},
                 ensure_ascii=False, indent=1), flush=True)
print("报告：" + str(args.report), flush=True)
