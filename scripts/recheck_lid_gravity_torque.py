"""用我自己那份 Isaac 短采曲线独立复核端点重力矩，并与 Codex 的原 3.3.0 审计对照。

要点：限位处的 I·Δv/Δt 是**含约束净响应**，不是孤立重力矩——盖子压在限位上时约束反力
可以把重力矩完全抵消，所以 q=0 那行的 0 不能当成"该处重力矩为 0"。自由行程上的点才可用。

做法（只用自由行程点，最小二乘）：τ_g(q) = A·sin(q - φ) + B
  物理上盖子是定轴摆，重力矩本就是这个形式，所以能外推到端点而不是硬取测值。
"""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MINE = ROOT / "outputs/runtime_validation/mini_lid_gravity_torque_curve.json"
THEIRS = ROOT / "outputs/runtime_validation/mini_lid_gravity_reference_audit.json"
OUT = ROOT / "outputs/runtime_validation/mini_lid_gravity_torque_recheck.json"

mine = json.loads(MINE.read_text(encoding="utf-8"))
theirs = json.loads(THEIRS.read_text(encoding="utf-8"))

real = [(float(r["angle"][1:]), r["implied_net_torque_Nm"], r["v_end_rad_s"]) for r in mine["rows"]
        if r["gravity"] == "real"]
# at_limit 由 Codex 的行标出；同时自己按"2 步内末速为 0 却有位移"判定被投影钳住
limit_qs = {row["q_rad"] for row in theirs["comparison_to_isaac_samples"] if row.get("at_limit")}
free = [(q, t) for q, t, v in real if q not in limit_qs and abs(v) > 1e-9]
clamped = [(q, t) for q, t, v in real if q in limit_qs or abs(v) <= 1e-9]

# 展开 A sin(q-φ)+B = a·sin q + b·cos q + B，线性最小二乘
n = len(free)
S = lambda f: sum(f(q) for q, _ in free)
import itertools


def solve(M, y):
    # 3x3 高斯消元，避免为这一步引入 numpy 依赖差异
    M = [row[:] + [y[i]] for i, row in enumerate(M)]
    for i in range(3):
        piv = max(range(i, 3), key=lambda r: abs(M[r][i]))
        M[i], M[piv] = M[piv], M[i]
        for r in range(3):
            if r != i:
                f = M[r][i] / M[i][i]
                M[r] = [c - f * d for c, d in zip(M[r], M[i])]
    return [M[i][3] / M[i][i] for i in range(3)]


rows = [[math.sin(q), math.cos(q), 1.0] for q, _ in free]
A1, B1, C1 = solve([list(map(sum, zip(*[[r[j] * r[k] for k in range(3)] for r in rows])))[i]
                    if False else [sum(r[i] * r[k] for r in rows) for k in range(3)] for i in range(3)],
                   [sum(r[i] * t for r, (_, t) in zip(rows, free)) for i in range(3)])
amp = math.hypot(A1, B1)
phi = math.atan2(B1, A1)                    # τ = amp·sin(q + phi)
fit = lambda q: A1 * math.sin(q) + B1 * math.cos(q) + C1
resid = max(abs(fit(q) - t) for q, t in free)

ref_at = {}
trows = theirs["rows"]
for target in (0.0, 0.5845205177021824, 1.712):
    best = min(trows, key=lambda r: abs(r["q_rad"] - target))
    ref_at[round(best["q_rad"], 6)] = best["gravity_torque_nm"]

cross = []
for q, t in free:
    best = min(trows, key=lambda r: abs(r["q_rad"] - q))
    cross.append({"q_rad": q, "isaac_measured_Nm": t, "isaac_fit_Nm": round(fit(q), 8),
                  "reference_Nm": best["gravity_torque_nm"],
                  "isaac_vs_reference_delta_Nm": round(t - best["gravity_torque_nm"], 8)})

report = {
    "scope": "recheck of lid gravity-torque endpoints using my own Isaac free-travel samples",
    "source_of_samples": MINE.name, "reference_audit": THEIRS.name,
    "reference_mujoco_version": theirs["mujoco_version"],
    "reference_methods": theirs["methods"], "reference_max_method_difference": theirs["max_method_difference"],
    "clamped_rows_excluded": [{"q_rad": q, "net_torque_Nm": t} for q, t in clamped],
    "reason_clamped_excluded": "限位处读数是被约束钳住的净响应，约束反力可抵消重力矩，不能当孤立重力矩用",
    "free_points_used": len(free),
    "fit_model": "tau_g(q) = A*sin(q) + B*cos(q) + C  (等价 amp*sin(q+phi))",
    "fit_coefficients": {"A": A1, "B": B1, "C": C1, "amplitude": amp, "phase_rad": phi},
    "fit_max_residual_Nm": resid,
    "equilibrium_zero_of_fit_rad": (phi + math.pi) % math.pi - phi - 0.0 if False else None,
    "endpoints": {
        "q_0": {"fit_Nm": fit(0.0), "reference_Nm": theirs["low_endpoint"]["gravity_torque_nm"],
                "isaac_naive_value_Nm": 0.0,
                "note": "我原先写 0 是限位钳位读数，非重力矩"},
        "q_1.712": {"fit_Nm": fit(1.712), "reference_Nm": theirs["high_endpoint"]["gravity_torque_nm"],
                    "isaac_naive_value_Nm": None}},
    "balance_point": {"fit_rad": None, "reference_rad": theirs["unstable_interior_equilibrium_rad"],
                      "reference_deg": theirs["unstable_interior_equilibrium_deg"]},
    "breakaway_thresholds_corrected": {
        "open_from_closed_Nm": abs(theirs["low_endpoint"]["gravity_torque_nm"]),
        "close_from_full_open_Nm": abs(theirs["high_endpoint"]["gravity_torque_nm"]),
        "superseded_wrong_values_I_published": {"open_from_closed_Nm": 0.038557,
                                                "why_wrong": "取了 q=0.1 的局部值当端点界限"},
        "validity": theirs["threshold_scope"]},
    "cross_check_free_points": cross,
    "max_free_point_isaac_vs_reference_delta_Nm": max(abs(c["isaac_vs_reference_delta_Nm"]) for c in cross),
    "not_claimed": ["不是真实设备标定值（reference 里 real_device_calibrated=false）",
                    "不是任意动态轨迹的力矩下界",
                    "端点值来自正弦拟合外推 + 原模型两条独立方法，不是 Isaac 在限位处的直接读数"]}

# 平衡角 = 拟合曲线的过零处，并且必须是 τ 由负变正的那个根（重力不稳定平衡）。
# A sin q + B cos q = r0·sin(q + psi)，psi = atan2(B, A)；前一版把 psi 的符号写反，
# 于是取到 2.5576 rad 这个假根，被参考值一对比就露馅了。
r0 = math.hypot(A1, B1)
psi = math.atan2(B1, A1)
# 完整求根：A sin q + B cos q + C = r0 sin(q+psi) + C = 0 → sin(q+psi) = -C/r0。
# 前一版令 C=0 直接取 q=-psi，漏掉了常数项，端点偏 5.2e-4 rad；C 虽小，
# 但平衡角正是 τ=0 的那个点，C 会整体平移它，不能省。
arg = max(-1.0, min(1.0, -C1 / r0))
base = math.asin(arg)
cands = sorted({(base - psi) % (2 * math.pi), (math.pi - base - psi) % (2 * math.pi)})
inside = [c for c in cands if 0.0 < c < 1.712 and (fit(c + 1e-4) - fit(c - 1e-4)) > 0]
report["balance_point"]["fit_rad"] = inside[0] if inside else None
report["balance_point"]["fit_deg"] = math.degrees(inside[0]) if inside else None
report["balance_point"]["candidate_roots_rad"] = [round(c, 6) for c in cands]
report["balance_point"]["fit_residual_at_root_Nm"] = (
    None if report["balance_point"]["fit_rad"] is None else fit(report["balance_point"]["fit_rad"]))
report["balance_point"]["root_requires_C_term"] = {
    "with_C_rad": report["balance_point"]["fit_rad"],
    "without_C_rad": (-psi) % (2 * math.pi),
    "C_Nm": C1, "shift_rad": None if report["balance_point"]["fit_rad"] is None
    else report["balance_point"]["fit_rad"] - ((-psi) % (2 * math.pi))}
report["balance_point"]["fit_vs_reference_delta_rad"] = (
    None if report["balance_point"]["fit_rad"] is None
    else round(report["balance_point"]["fit_rad"] - theirs["unstable_interior_equilibrium_rad"], 6))
report["fit_endpoints_agree_with_reference_within_Nm"] = max(
    abs(fit(0.0) - theirs["low_endpoint"]["gravity_torque_nm"]),
    abs(fit(1.712) - theirs["high_endpoint"]["gravity_torque_nm"]))

OUT.write_text(json.dumps(report, indent=1, ensure_ascii=False), encoding="utf-8")
print(json.dumps({k: report[k] for k in ("clamped_rows_excluded", "fit_coefficients", "fit_max_residual_Nm",
                                         "endpoints", "balance_point", "breakaway_thresholds_corrected",
                                         "max_free_point_isaac_vs_reference_delta_Nm",
                                         "fit_endpoints_agree_with_reference_within_Nm")},
                 indent=1, ensure_ascii=False), flush=True)
print("WRITTEN", OUT)
