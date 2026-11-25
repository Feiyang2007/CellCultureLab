"""
量 Play 状态下的帧时间：决定 96 支枪头 × 37 片凸包（共 4438 个碰撞体）这个代价要不要砍。

headless 跑物理，不开渲染窗口；用 timeline.play() 之后统计每次 app.update() 的墙钟耗时，
给中位数 / p95 / 最差值，并对照未播放（idle）时的耗时，把物理开销单独拆出来。

用法：
    D:/IsaacSim5/env/Scripts/python.exe scripts/measure_play_perf.py [场景] [--steps 240]
"""
import os, sys, time, statistics

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
args = [a for a in sys.argv[1:] if not a.startswith("--")]
steps = 240
if "--steps" in sys.argv:
    steps = int(sys.argv[sys.argv.index("--steps") + 1])
scene = os.path.abspath(args[0]) if args else os.path.join(
    PROJ, "assets", "scenes", "biolab_main_v04.usda")

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd
import omni.timeline
from pxr import Usd, UsdPhysics

ctx = omni.usd.get_context()
if not ctx.open_stage(scene):
    print("打不开场景：", scene)
    app.close(); sys.exit(1)
st = ctx.get_stage()

# 可选：在【会话层】把每支枪头除第一个之外的碰撞体禁用掉，用来量"37 片凸包"到底贵多少。
# 只改内存里的 session layer，不写回任何 USD 文件。
if "--one-collider-tips" in sys.argv:
    import collections, re
    from pxr import Sdf
    groups = collections.defaultdict(list)
    for prim in st.TraverseAll():
        path = str(prim.GetPath())
        m = re.search(r"(/World/TaskArea/Objects/Tips/.*?Tip_[A-H]\d\d)/", path)
        if not m:
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            groups[m.group(1)].append(prim)
    keep = 0 if "--zero-tip-collision" in sys.argv else 1
    n_off = 0
    with Usd.EditContext(st, st.GetSessionLayer()):
        for root, prims in groups.items():
            for extra in prims[keep:]:
                a = extra.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool)
                a.Set(False)
                n_off += 1
    print("会话层禁用枪头碰撞体：%d 个（涉及 %d 支枪头，每支保留 %d 个）" % (n_off, len(groups), keep))

# 另一组对照：把枪头本体设为运动学（不参与动力学求解，但碰撞保留），用来验证
# "贵的是 96 个自由/醒着的刚体，而不是碰撞体个数"这个解释。
if "--tips-kinematic" in sys.argv:
    import re as _re
    from pxr import Sdf
    n_kin = 0
    with Usd.EditContext(st, st.GetSessionLayer()):
        for prim in st.TraverseAll():
            path = str(prim.GetPath())
            if not _re.search(r"/Tips/[^/]+/Tip_[A-H]\d\d$", path):
                continue
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            a = prim.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool)
            a.Set(True)
            n_kin += 1
    print("会话层把 %d 支枪头设成运动学" % n_kin)

# 嫌疑最重的两项：CCD（101 个刚体开着）与超高的求解迭代（盖子 128/32、管身 64/8）。
# 会话层临时改，用来验证 27 ms 的净开销是不是花在这里，不写回文件。
if "--tune-solver" in sys.argv:
    from pxr import Sdf
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
    print("会话层：关掉 CCD %d 个刚体；降低求解迭代 %d 个刚体" % (n_ccd, n_it))

rb = sum(1 for p in st.TraverseAll() if p.HasAPI(UsdPhysics.RigidBodyAPI))
col = sum(1 for p in st.TraverseAll() if p.HasAPI(UsdPhysics.CollisionAPI)
          and p.GetAttribute("physics:collisionEnabled").Get() is not False)
meshes = sum(1 for p in st.TraverseAll() if str(p.GetTypeName()) == "Mesh")
print("=" * 88)
print("场景：", scene)
print("刚体 %d 碰撞体 %d 网格 %d" % (rb, col, meshes))

def sample(n, tag):
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        app.update()
        ts.append((time.perf_counter() - t0) * 1000.0)
    ts.sort()
    p50 = ts[len(ts) // 2]
    p95 = ts[int(len(ts) * 0.95) - 1]
    print("  %-16s 中位 %6.2f ms   p95 %6.2f ms   最差 %6.2f ms   折算 %5.1f fps(按中位)"
          % (tag, p50, p95, ts[-1], 1000.0 / p50))
    return p50, p95

# 先量不播放时的空转开销
idle50, idle95 = sample(60, "idle(未播放)")

tl = omni.timeline.get_timeline_interface()
tl.play()
warm = sample(60, "play 预热段")
play50, play95 = sample(steps, "play(稳定段)")
tl.stop()

print("\n物理净开销 ≈ play 中位 - idle 中位 = %.2f ms/帧" % (play50 - idle50))
print("判据：单帧 < 30 ms 可接受；30~60 ms 勉强能采但慢；> 60 ms 就该砍碰撞体")
print("      （砍法：把每支枪头 37 片凸包换成 1 片整体凸包，或用 collision approximation=boundingBox/单片 convexHull）")
app.close()
