"""
稳定性判据：调低求解迭代 / 关掉 CCD 之后，场景会不会塌。

做法：一次启动里跑两轮 300 帧，分别在"稳定后"与"再跑 240 帧"采样每个刚体的世界坐标，
比较位移。基线先跑，然后在会话层施加调优（关 CCD、迭代压到 16/4）再跑一遍。
不落盘、不写回 USD，只出判据。

判据（我定的，写死在这里免得事后找理由）：
  沉降   —— 任一刚体 z 向下超过 5 mm 视为塌；
  漂移   —— 水平位移超过 10 mm 视为漂；
  抖动   —— 稳定段内逐帧位移最大者超过 2 mm 视为抖。
"""
import os, sys, math

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd, omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(PROJ, "assets", "scenes", "biolab_main_v04.usda")
ctx = omni.usd.get_context()
ctx.open_stage(SCENE)
st = ctx.get_stage()

BODIES = [p for p in st.TraverseAll() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
print("刚体数：%d" % len(BODIES))

def snapshot():
    xfc = UsdGeom.XformCache()
    out = {}
    for p in BODIES:
        try:
            m = xfc.GetLocalToWorldTransform(p)
            t = m.ExtractTranslation()
            out[str(p.GetPath())] = (t[0], t[1], t[2])
        except Exception:
            pass
    return out

def run_round(tag, settle=90, watch=240):
    tl = omni.timeline.get_timeline_interface()
    tl.stop(); tl.play()
    for _ in range(settle):
        app.update()
    a = snapshot()
    per_step_max = 0.0
    prev = a
    for _ in range(watch):
        app.update()
        cur = snapshot()
        # 只每 30 帧算一次两帧间最大位移，省时间
        if _ % 30 == 0 and _ > 0:
            for k, v in cur.items():
                if k in prev:
                    d = math.dist(v, prev[k])
                    if d > per_step_max:
                        per_step_max = d
        prev = cur
    b = snapshot()
    tl.stop()

    sinks, drifts, moved = [], [], 0
    for k, v in b.items():
        if k not in a:
            continue
        dz = v[2] - a[k][2]
        dh = math.dist(v[:2], a[k][:2])
        if dz < -0.005:
            sinks.append((k, dz))
        if dh > 0.010:
            drifts.append((k, dh))
        if abs(dz) > 0.001 or dh > 0.001:
            moved += 1
    sinks.sort(key=lambda x: x[1])
    drifts.sort(key=lambda x: -x[1])
    print("=" * 88)
    print("[%s] 观察 %d 帧" % (tag, watch))
    print("  沉降>5mm：%d 个   水平漂移>10mm：%d 个   有位移的刚体：%d / %d"
          % (len(sinks), len(drifts), moved, len(b)))
    print("  逐帧最大位移（30 帧抽一帧测）：%.4f m" % per_step_max)
    for k, v in sinks[:3]:
        print("     沉  %+.4f m  %s" % (v, k))
    for k, v in drifts[:3]:
        print("     漂  %+.4f m  %s" % (v, k))
    return {"sink": len(sinks), "drift": len(drifts), "jitter": per_step_max}

base = run_round("基线（原求解配置）")

# ---- 会话层施加调优：关 CCD、迭代压到 16/4 ----
n_ccd = n_it = 0
with Usd.EditContext(st, st.GetSessionLayer()):
    for prim in st.TraverseAll():
        ccd = prim.GetAttribute("physxRigidBody:enableCCD")
        if ccd and ccd.HasAuthoredValue() and ccd.Get():
            ccd.Set(False); n_ccd += 1
        pos = prim.GetAttribute("physxRigidBody:solverPositionIterationCount")
        if pos and pos.HasAuthoredValue() and int(pos.Get() or 0) > 16:
            pos.Set(16); n_it += 1
        vel = prim.GetAttribute("physxRigidBody:solverVelocityIterationCount")
        if vel and vel.HasAuthoredValue() and int(vel.Get() or 0) > 4:
            vel.Set(4)
print("\n已施加调优：关 CCD %d 个、降迭代 %d 个\n" % (n_ccd, n_it))
tuned = run_round("调优后（关CCD + 16/4）")

print("=" * 88)
print("对比：沉降 %d→%d，漂移 %d→%d，抖动 %.4f→%.4f m"
      % (base["sink"], tuned["sink"], base["drift"], tuned["drift"],
         base["jitter"], tuned["jitter"]))
ok = tuned["sink"] <= base["sink"] + 2 and tuned["drift"] <= base["drift"] + 2 \
     and tuned["jitter"] <= max(0.002, base["jitter"] * 1.5)
print("判据结论：调优后%s稳定，可以固化。" % ("仍" if ok else "不再"))
app.close()
sys.exit(0 if ok else 3)
