"""
给导入器产出的仪器补"底座固定"：把每台仪器根 body 设为运动学。

根因（实测）：官方 MJCF 导入器写出的 rootJoint_body 是 FixedJoint，但 body0 为空 ——
PhysX 不会因此把底座钉住，于是整台仪器作为自由落体从台面上掉到地板：
C1000 的 body 起始 z=0.9704 m，实测最大沉降 −0.9704 m，数字完全对上；
连带撞飞 142 个刚体、最大漂移 3.02 m。这也解释了为什么改 CCD / 改迭代 / 挪位置都无效。

正确做法在本项目里已有先例：师兄那台离心机的 /World/TaskArea/Equipment/Centrifuge/Body
上就写着 physics:kinematicEnabled = True。照同样方式补齐即可，盖子等活动件仍是动力学刚体。

产出 v07（Export，不覆盖 v06）。
"""
import os, sys

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdPhysics, Sdf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(PROJ, "assets", "scenes", "biolab_main_v06.usda")
DST = os.path.join(PROJ, "assets", "scenes", "biolab_main_v07.usda")

st = Usd.Stage.Open(SRC)

# 规则（不靠猜名字）：任何"固定关节的 body0 为空、body1 是刚体"的情况，
# 都说明这个刚体本该被钉在世界里却没钉住 —— 官方 MJCF 导入器的 rootJoint_body 就是这样。
# 处理方式与本项目里师兄那台离心机一致：把该刚体设为运动学。
anchored = []
for prim in st.TraverseAll():
    if str(prim.GetTypeName()) != "PhysicsFixedJoint":
        continue
    r0 = prim.GetRelationship("physics:body0")
    r1 = prim.GetRelationship("physics:body1")
    t0 = r0.GetTargets() if r0 else []
    t1 = r1.GetTargets() if r1 else []
    if t0 or not t1:
        continue
    b = st.GetPrimAtPath(t1[0])
    if b and b.HasAPI(UsdPhysics.RigidBodyAPI):
        anchored.append((str(prim.GetPath()), str(t1[0])))

print("发现 %d 处【无 body0 的固定关节】，其单体刚体本该被钉住：" % len(anchored))
done = []
for jp, bp in anchored:
    b = st.GetPrimAtPath(bp)
    a = b.GetAttribute("physics:kinematicEnabled") or         b.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool)
    was = a.Get()
    a.Set(True)
    done.append(bp)
    print("   钉住 %-58s（关节 %s）原值=%s" % (bp.replace("/World/TaskArea/", ""),
                                               jp.split("/")[-1], was))
print("共钉住 %d 个底座刚体" % len(done))

# 顺手删掉这些畸形关节本身：两端最终都指向同一个（现在的运动学）底座，
# 留着会让 PhysX 生成退化约束并数值发散（实测沉降 −5964 m 就是这么来的）。
for jp, bp in anchored:
    st.RemovePrim(jp)
print("删除畸形根关节 %d 条" % len(anchored))
ok = st.GetRootLayer().Export(DST)
print("导出 v07：", "成功" if ok else "失败", os.path.getsize(DST), "字节")
