"""一枪 tip 的落台实验：oracle 是"插到位"还是"从上面掉下去"，哪个不炸。

2026-11-23 reachable 全量跑出来：10 条落点里凡是 tip_200ul 的体积类判据全部
炸飞（pos 落到 z=-19.5205，甚至有 [57110, -61991, -9242] 这种），而 tube、
cryovial、板子落同一张台面都稳。到底是"插入时初始互穿"还是"tip 自己站不住"，
坐着猜不出来，直接把同一个 tip 用不同的初始间隙放下去，逐帧抄 z。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/_probe_tip_drop.py
"""
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 320, "height": 240})

import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(PROJ, "assets", "scenes", "biolab_main_v07.usda")
TIP = "/World/TaskArea/Objects/Tips/RackPool/Tip_G10"
TUBE = "/World/TaskArea/Objects/CentrifugeTubes/RackPool/Tube_R04_C11"

# 台面（实测顶面 z=0.8717）上一块空着的落点，直接抄 tasks_v01_seed0.json 里
# spatial_03/spatial_01 的 oracle xy，不另发明第二个坐标。
SPOTS = {"tip@spatial_03": (TIP, [-1.336, -1.154, 0.8987]),
         "tip@+20mm":      (TIP, [-1.336, -1.154, 0.9187]),
         "tip@+60mm":      (TIP, [-1.336, -1.154, 0.9587]),
         "tube@spatial_01": (TUBE, [-1.7131, -1.551, 0.8737])}
N = 120

ctx = omni.usd.get_context()
ctx.open_stage(SCENE)
stage = ctx.get_stage()
tl = omni.timeline.get_timeline_interface()
px = omni.physx.get_physx_interface()


def world_z(path):
    """PhysX 里的实时位置；读不到就报 None，不拿 USD 变换冒充。

    和判据脚本走同一个 API（get_rigidbody_transformation 返回 dict，
    位置在 "position"），别再发明第二条读取路径。
    """
    ret = px.get_rigidbody_transformation(path)
    if not isinstance(ret, dict) or ret.get("ret_val") is False:
        return None
    p = ret.get("position")
    return None if p is None else [float(p[0]), float(p[1]), float(p[2])]


def place(path, xyz):
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    cur = xf.GetLocalToWorldTransform(prim).ExtractTranslation()
    op = prim.GetAttribute("xformOp:translate")
    if not op:
        return "没有 xformOp:translate"
    op.Set(Gf.Vec3d(*op.Get()) + (Gf.Vec3d(*xyz) - Gf.Vec3d(*cur)))
    return None


for name, (obj, xyz) in SPOTS.items():
    tl.stop()
    for _ in range(6):
        app.update()
    e = place(obj, xyz)
    if e:
        print(name, "摆放失败", e)
        continue
    tl.play()
    zs = []
    for i in range(N):
        app.update()
        if i % 10 == 0 or i == N - 1:
            p = world_z(obj)
            zs.append("%d:%s" % (i, "None" if not p else "%.4f" % p[2]))
    print("%-16s 起点 z=%.4f  逐帧 z: %s" % (name, xyz[2], "  ".join(zs)))

tl.stop()
app.close()
