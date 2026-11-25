"""
求解器配置扫描：把"关 CCD"和"降迭代"两个变量拆开，同时量帧时与物理塌陷。

为什么要拆开：上一轮一次性改了两样，结果是盖子下沉 42.6 mm —— 高迭代其实是用来压住
"盖↔管身"这个病态约束的，不该跟着 CCD 一起砍。

选法（写死，避免事后找理由）：在所有【最大沉降 ≤ 2 mm 且无刚体位移 > 10 mm】的配置里，
取帧时中位最小的那个。

用法：
    D:/IsaacSim5/env/Scripts/python.exe scripts/tune_solver_scan.py
"""
import os, sys, math, statistics, json

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd, omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(PROJ, "assets", "scenes", "biolab_main_v04.usda")
if "--single" in sys.argv:
    idx = sys.argv.index("--single")
    SCENE = os.path.abspath(sys.argv[idx + 1])
    print("单轮稳定性复核：", SCENE)
ctx = omni.usd.get_context()
ctx.open_stage(SCENE)
st = ctx.get_stage()

# 记下所有被我们可能改动的原始值，便于在配置之间精确复原
orig = {}
for prim in st.TraverseAll():
    for an in ("physxRigidBody:enableCCD",
               "physxRigidBody:solverPositionIterationCount",
               "physxRigidBody:solverVelocityIterationCount"):
        a = prim.GetAttribute(an)
        if a and a.HasAuthoredValue():
            orig[(str(prim.GetPath()), an)] = a.Get()
print("记录了 %d 处原始求解器设置" % len(orig))
print("判据：以刚 play 的原始姿态为基准，跑 600 帧（约 10 秒仿真）量绝对位移；")
print("      最大沉降 > 2 mm 或 漂移 > 10 mm 即判为塌陷。")
BODIES = [p for p in st.TraverseAll() if p.HasAPI(UsdPhysics.RigidBodyAPI)]


def apply_cfg(ccd_off, iter_down):
    with Usd.EditContext(st, st.GetSessionLayer()):
        for (path, an), v in orig.items():
            prim = st.GetPrimAtPath(path)
            a = prim.GetAttribute(an)
            if not a:
                continue
            if an == "physxRigidBody:enableCCD":
                a.Set(False if ccd_off else v)
            elif an == "physxRigidBody:solverPositionIterationCount":
                a.Set(16 if (iter_down and int(v or 0) > 16) else v)
            elif an == "physxRigidBody:solverVelocityIterationCount":
                a.Set(4 if (iter_down and int(v or 0) > 4) else v)


def snapshot():
    xfc = UsdGeom.XformCache()
    out = {}
    for p in BODIES:
        m = xfc.GetLocalToWorldTransform(p)
        t = m.ExtractTranslation()
        out[str(p.GetPath())] = (t[0], t[1], t[2])
    return out


def run(tag, ccd_off, iter_down, total=600, perf_last=200):
    apply_cfg(ccd_off, iter_down)
    tl = omni.timeline.get_timeline_interface()
    tl.stop(); tl.play()
    import time as _t
    # 基准 = 刚 play 完、一帧都没跑时的原始姿态；这样任何阶段的塌陷都跑不掉
    a = snapshot()
    ts = []
    for i in range(total):
        t0 = _t.perf_counter()
        app.update()
        ms = (_t.perf_counter() - t0) * 1000.0
        if i >= total - perf_last:
            ts.append(ms)
    b = snapshot()
    tl.stop()

    worst_sink, worst_drift, n_move = 0.0, 0.0, 0
    who = ""
    for k, v in b.items():
        if k not in a:
            continue
        dz = v[2] - a[k][2]
        dh = math.dist(v[:2], a[k][:2])
        if dz < worst_sink:
            worst_sink, who = dz, k
        if dh > worst_drift:
            worst_drift = dh
        if abs(dz) > 0.001 or dh > 0.001:
            n_move += 1
    ts.sort()
    p50 = statistics.median(ts) if ts else 0.0
    ok = (worst_sink >= -0.002) and (worst_drift <= 0.010)
    print("%-22s 帧时中位 %6.2f ms   相对原始姿态最大沉降 %+8.4f m   最大漂移 %.4f m   动过的刚体 %3d   %s"
          % (tag, p50, worst_sink, worst_drift, n_move, "合格" if ok else "★塌陷"))
    if not ok and who:
        print("       塌陷对象：", who)
    return {"tag": tag, "p50": p50, "sink": worst_sink, "drift": worst_drift, "ok": ok,
            "moved": n_move}


if "--single" in sys.argv:
    rows = [run("文件自身配置", False, False)]
else:
    rows = [run("基线（原配置）", False, False),
            run("只关 CCD", True, False),
            run("只降迭代→16/4", False, True),
            run("CCD+迭代都改", True, True)]

good = [r for r in rows if r["ok"]]
print("=" * 96)
if good:
    best = min(good, key=lambda r: r["p50"])
    print("合格且最快：%s（%.2f ms，基线 %.2f ms，省 %.1f%%）"
          % (best["tag"], best["p50"], rows[0]["p50"],
             100.0 * (rows[0]["p50"] - best["p50"]) / rows[0]["p50"]))
    json.dump({"best": best["tag"], "rows": rows},
              open(os.path.join(PROJ, "docs", "求解器扫描.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print("已写 docs/求解器扫描.json")
else:
    print("没有一个配置合格，说明测量本身有问题")
app.close()
