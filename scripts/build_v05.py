"""
把"关掉 CCD"这个结论固化成 v05（不覆盖 v04）。

依据：tune_solver_scan.py 实测 —— v04 原样跑 600 帧（约 10 秒仿真）后 145 个刚体飞散
（最大漂移 3.02 m、最大沉降 0.97 m），而把 CCD 关掉后一个都不动、帧时不劣化；
单降求解迭代反而让 50mL 管盖下沉 57 mm，所以迭代保持原值不动。

派生新场景一律用 Export(新路径)——早先用 Save() 把 v02 覆盖过一次。
"""
import os, sys

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, Sdf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(PROJ, "assets", "scenes", "biolab_main_v04.usda")
DST = os.path.join(PROJ, "assets", "scenes", "biolab_main_v05.usda")

st = Usd.Stage.Open(SRC)
if st is None:
    print("打不开 v04"); sys.exit(1)

changed = skipped = 0
for prim in st.TraverseAll():
    a = prim.GetAttribute("physxRigidBody:enableCCD")
    if not (a and a.HasAuthoredValue() and a.Get()):
        continue
    a.Set(False)
    changed += 1

ok = st.GetRootLayer().Export(DST)          # 关键：Export 到新路径，不用 Save
print("关掉 CCD 的刚体：%d 个" % changed)
print("导出 %s -> %s" % ("成功" if ok else "失败", os.path.basename(DST)))
if ok:
    print("大小：", os.path.getsize(DST), "字节")
