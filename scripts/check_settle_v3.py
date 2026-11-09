"""
稳定性检查第三版：位姿一律向 PhysX 本体要，不再读 USD 变换。

前两版都是拿 UsdGeom.XformCache 的世界变换做差，结果第二版里所有"飞掉"的物体终点都是
(0,0,0) —— 那是取不到变换时回落到恒等矩阵的假象，不是物理事实。所以"场景会散"这个结论
到目前为止并没有被可信地证实过。

这一版只用 omni.physx 的 get_rigidbody_transformation()，并且在解析不出返回结构时
直接把原始返回打出来（宁可报错也不假装读懂）。

用法：
    D:/IsaacSim5/env/Scripts/python.exe scripts/check_settle_v3.py [场景] [--frames 600]
"""
import os, sys, math

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd, omni.timeline, omni.physx, carb
from pxr import Usd, UsdPhysics, Gf

# USD 层设了 gpuTotalAggregatePairsCapacity 引擎不采纳（实测仍喊要提到 1025），
# 得走 Isaac 自己用的 carb 设置。这里在开仿之前一次设好。
if "--gpu-cap" in sys.argv:
    cs = carb.settings.get_settings()
    for k, v in (("gpuTotalAggregatePairsCapacity", 16384),
                 ("gpuFoundLostAggregatePairsCapacity", 16384),
                 ("gpuMaxRigidContactCountEstimate", 262144),
                 ("gpuMaxPatchCountEstimate", 4194304),
                 ("gpuTensor2GlobalSize", 8388608),
                 ("gpuTensorContactCount", 8388608)):
        cs.set("/persistent/physics/" + k, v)
    print("已通过 carb 设置 GPU 接触容量")

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
args = [a for a in sys.argv[1:] if not a.startswith("--")]
scene = os.path.abspath(args[0] if args else os.path.join(
    PROJ, "assets", "scenes", "biolab_main_v04.usda"))
frames = 600
if "--frames" in sys.argv:
    frames = int(sys.argv[sys.argv.index("--frames") + 1])

ctx = omni.usd.get_context()
if not ctx.open_stage(scene):
    print("打不开：", scene); app.close(); sys.exit(1)
st = ctx.get_stage()
iface = omni.physx.get_physx_interface()

bodies = []
for prim in st.Traverse():
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        continue
    a = prim.GetParent()
    nested = False
    while a:
        if a.HasAPI(UsdPhysics.RigidBodyAPI):
            nested = True
            break
        a = a.GetParent()
    if not nested:
        bodies.append(str(prim.GetPath()))

print("=" * 92)
print("场景：", os.path.basename(scene), " 独立刚体根：", len(bodies))


def norm(ret, raw_first=False):
    """PhysX 官方测试里的用法是 transform["position"]；返回 {'ret_val': False} 表示
    这个 prim 在仿真里压根不是被仿真的刚体 —— 这是重要信息，单列计数，别当成位移。"""
    if ret is None:
        return None
    if isinstance(ret, dict):
        if ret.get("ret_val") is False or "position" not in ret:
            return None
        p = ret["position"]
        try:
            return (float(p[0]), float(p[1]), float(p[2]))
        except Exception:
            return None
    # 情况 A： (pos, quat) 两元组
    if isinstance(ret, (tuple, list)) and len(ret) == 2:
        p = ret[0]
    else:
        p = ret
    # 矩阵：取最后一列平移
    try:
        if hasattr(p, "ExtractTranslation"):
            v = p.ExtractTranslation()
            return (v[0], v[1], v[2])
    except Exception:
        pass
    try:
        seq = list(p)
    except Exception:
        return None
    if len(seq) and isinstance(seq[0], (list, tuple)):        # 4x4
        if len(seq) >= 3 and len(seq[0]) >= 4:
            return (float(seq[0][3]), float(seq[1][3]), float(seq[2][3]))
        return None
    nums = [float(x) for x in seq if isinstance(x, (int, float))]
    if len(nums) >= 3:
        return (nums[0], nums[1], nums[2])
    return None


fails = set()


def sample(tag_paths):
    out = {}
    for path in tag_paths:
        try:
            ret = iface.get_rigidbody_transformation(path)
        except Exception as e:
            out[path] = None
            continue
        v = norm(ret)
        out[path] = v
        if v is None:
            fails.add(path)
    return out



tl = omni.timeline.get_timeline_interface()
tl.stop(); tl.play()
for _ in range(3):
    app.update()
a = sample(bodies)
n_ok = sum(1 for v in a.values() if v)
print("PhysX 里真正被仿真的刚体：%d / %d   （%d 个读不到 = 不是仿真体）"
      % (n_ok, len(bodies), len(bodies) - n_ok))
grp = {}
for f in sorted(fails):
    key = "/".join(f.split("/")[:5])
    grp[key] = grp.get(key, 0) + 1
if grp:
    print("读不到的是这些位置（按前 5 段归并）：")
    for k, v in sorted(grp.items(), key=lambda x: -x[1])[:12]:
        print("   %-70s %d" % (k, v))
for _ in range(frames):
    app.update()
b = sample(bodies)
tl.stop()

moved = []
for k in b:
    if not (a.get(k) and b.get(k)):
        continue
    p0, p1 = a[k], b[k]
    d = math.dist(p0, p1)
    if d > 0.001:
        moved.append((d, p1[2] - p0[2], math.dist(p0[:2], p1[:2]), k, p0, p1))
moved.sort(reverse=True)
sinks = [m for m in moved if m[1] < -0.005]
drift = [m for m in moved if m[2] > 0.010]
absurd = [m for m in moved if m[0] > 3.0]

print()
print("跑了 %d 帧后：有位移 %d 个（其中沉降>5mm %d 个、水平漂移>10mm %d 个、位移>3m 的可疑 %d 个）"
      % (frames, len(moved), len(sinks), len(drift), len(absurd)))
print("位移最大的 8 个：")
for d, dz, dh, k, p0, p1 in moved[:8]:
    print("   %8.4f m (Δz %+8.4f 水平 %7.4f)  %s" % (d, dz, dh, k.replace("/World/TaskArea/", "")))
    print("              起 (%+.4f,%+.4f,%+.4f) → 止 (%+.4f,%+.4f,%+.4f)" % (p0 + p1))
print()
measured = sum(1 for k in b if a.get(k) and b.get(k))
if measured == 0:
    print("判读：【无效】一个刚体的位姿都没测到，没有东西被仿真，"
          "不能据此说场景稳（这正是嵌套刚体把变换栈弄坏时的表现）")
    app.close(); sys.exit(4)
verdict = "稳" if (len(sinks) == 0 and len(drift) == 0 and len(absurd) == 0) else "不稳"
print("判读（PhysX 视角，实测 %d/%d 个刚体）：场景在 %d 帧内【%s】"
      % (measured, len(bodies), frames, verdict))
app.close()
