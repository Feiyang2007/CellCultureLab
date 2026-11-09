"""
把 GPU 接触对容量写进场景，消掉 PhysX 那句"会漏接触"的报错。产出 v07。

依据（不是猜的）：跑仿真的日志里反复出现
    PhysX error: The application needs to increase
    PxGpuDynamicsMemoryConfig::totalAggregatePairsCapacity to 1025,
    otherwise, the simulation will miss interactions
漏接触对"抓取/插拔"这类任务是致命的——明明碰到了，引擎不算接触。

 Isaac 自己用的属性名来自 PhysxSceneAPI 的 gpuTotalAggregatePairsCapacity
（见 exts/isaacsim.core.api/.../physics_context.py 里的 CreateGpuTotalAggregatePairsCapacityAttr）。
这里优先用 schema API 建属性，取不到再按同名的原始属性兜底。
"""
import os, sys

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, Sdf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(PROJ, "assets", "scenes", "biolab_main_v06.usda")
DST = os.path.join(PROJ, "assets", "scenes", "biolab_main_v07.usda")

# 引擎建议 1025，留足余量（枪头 96 支 + 管 70 支 + 仪器活动件，接触对会随操作增长）
VALS = {
    "gpuTotalAggregatePairsCapacity": 8192,
    "gpuFoundLostAggregatePairsCapacity": 8192,
    "gpuMaxRigidContactCountEstimate": 262144,
    "gpuMaxPatchCountEstimate": 4194304,
}

st = Usd.Stage.Open(SRC)
sc = st.GetPrimAtPath("/World/physicsScene")
if not sc:
    for p in st.Traverse():
        if str(p.GetTypeName()) == "PhysicsScene":
            sc = p
            break
if not sc:
    print("场景里没有 PhysicsScene")
    sys.exit(1)
print("physicsScene：", sc.GetPath())

try:
    from pxr import PhysxSchema
    api = PhysxSchema.PhysxSceneAPI.Apply(sc)
    for k, v in VALS.items():
        fn = getattr(api, "Create" + k[0].upper() + k[1:] + "Attr", None)
        if fn is None:
            print("  schema 没有这个属性，改用原始属性：", k)
            a = sc.GetAttribute("physxScene:" + k) or sc.CreateAttribute("physxScene:" + k, Sdf.ValueTypeNames.Int)
            a.Set(int(v))
        else:
            fn().Set(int(v))
        print("  %-40s = %d" % (k, v))
except Exception as e:
    print("schema 路径不可用（%s），改用原始属性" % type(e).__name__)
    for k, v in VALS.items():
        a = sc.GetAttribute("physxScene:" + k) or sc.CreateAttribute("physxScene:" + k, Sdf.ValueTypeNames.Int)
        a.Set(int(v))
        print("  %-40s = %d" % (k, v))

print()
print("写完后场景上实际的 physxScene:* 属性：")
for a in sc.GetAttributes():
    if a.GetName().startswith("physxScene:"):
        print("   ", a.GetName(), "=", a.Get())

ok = st.GetRootLayer().Export(DST)
print("\n导出 v07：", "成功" if ok else "失败", os.path.basename(DST))
