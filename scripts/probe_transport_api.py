"""#24 的前置探测：仿真播放中到底用哪条 API 能把刚体按帧搬起来。

判据脚本的 Placer 写的是 USD xform，而"边播边挪等于没挪"是之前记进文档的账
（PhysX 把自己的位姿盖回来）。所以搬运老师要么走 kinematic 刚体（位姿由 xform
驱动），要么走 RigidBodyView/RigidPrim 直接写 PhysX。这条脚本把三条写法各推 20 帧
对照读数，renderer=None、不跑长仿真，20 s 出结论。

跑法（台式机）：
    C:\\Users\\73\\IsaacLab\\.venv\\Scripts\\python.exe -u scripts\\probe_transport_api.py
"""
import inspect
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "renderer": "None", "width": 320, "height": 240})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda")
TARGET = "/World/TaskArea/Stations/Dish100_1"   # spatial_00 的主角：100 mm 皿
STEP = 0.004                                     # 每帧抬 4 mm，20 帧 = 8 cm


def say(tag, exc):
    print("   %s：%s %s" % (tag, type(exc).__name__,
                            str(exc).encode("ascii", "backslashreplace").decode("ascii")[:220]))


def set_z(placer_prim, dz):
    """改自己的 local translate（和 Placer.place_world 同一套写法）。"""
    a = placer_prim.GetAttribute("xformOp:translate")
    if not a or not a.IsValid() or a.Get() is None:
        UsdGeom.Xformable(placer_prim).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        a = placer_prim.GetAttribute("xformOp:translate")
    base = np.array(a.Get())
    a.Set(Gf.Vec3d(*(base + np.array([0.0, 0.0, dz]))))


def main():
    ctx = omni.usd.get_context()
    if not ctx.open_stage(SCENE):
        print("打不开场景", SCENE)
        return 2
    stage = ctx.get_stage()
    tl = omni.timeline.get_timeline_interface()
    physx_iface = __import__("omni.physx", fromlist=["get_physx_interface"]).get_physx_interface()
    from pxr import UsdGeom as _UG
    cache = _UG.XformCache()

    def read(tag):
        r = physx_iface.get_rigidbody_transformation(TARGET)
        p = None if not isinstance(r, dict) or r.get("ret_val") is False else np.round(r["position"], 4).tolist()
        u = np.round(np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(TARGET))
                              .ExtractTranslation()), 4).tolist()
        print("   %-16s physx=%s  usd=%s" % (tag, p, u))
        return p

    prim = stage.GetPrimAtPath(TARGET)
    kin = prim.GetAttribute("physics:kinematicEnabled")
    print("目标 %s  存在=%s  RigidBodyAPI=%s  kinematicEnabled 属性=%s(值 %s)"
          % (TARGET.rsplit("/", 1)[-1], bool(prim), prim.HasAPI(UsdPhysics.RigidBodyAPI),
             bool(kin), kin.Get() if kin else None))

    tl.play()
    for _ in range(30):
        app.update()
    read("起点")

    print("== A) 播放中只写 USD xform（Placer 那条路）")
    for _ in range(20):
        set_z(prim, STEP)
        app.update()
    za = read("A 结果")

    print("== B) 先 physics:kinematicEnabled=True 再写 xform")
    if kin:
        kin.Set(True)
        for _ in range(20):
            set_z(prim, STEP)
            app.update()
        zb = read("B 结果")
        kin.Set(False)
    else:
        zb = None

    print("== C) RigidBodyView 直接写 PhysX")
    zc = None
    try:
        from omni.physx.tensors import create_rigid_body_view
        rbv = create_rigid_body_view(TARGET)
        print("   view 建成，count =", getattr(rbv, "count", "n/a"))
        if rbv.count:
            p0 = rbv.get_positions().copy()
            meth = [n for n in dir(rbv) if n.startswith("set_") and
                    any(k in n for k in ("position", "transform", "pose"))]
            print("   候选写法：", meth)
            for i in range(20):
                p = p0 + np.array([[0.0, 0.0, STEP]], dtype=np.float32)
                done = False
                for n in ("set_transforms", "set_positions_and_orientations", "set_positions"):
                    f = getattr(rbv, n, None)
                    if not f:
                        continue
                    try:
                        if n == "set_positions":
                            f(p)
                        elif n == "set_transforms":
                            f(positions=p)
                        else:
                            f(p, np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32))
                        done = True
                        break
                    except Exception as exc:
                        say("     " + n, exc)
                if not done:
                    break
                p0 = p
                app.update()
            zc = read("C 结果")
    except Exception as exc:
        say("RigidBodyView", exc)

    print("== D) isaacsim.core.prims.RigidPrim(kinematic_only=True)")
    zd = None
    try:
        from isaacsim.core.prims import RigidPrim
        print("   set_world_poses 形参：", str(inspect.signature(RigidPrim.set_world_poses))[:200])
        rp = RigidPrim(prim_paths=[TARGET], name="probe_rp")
        rp.initialize()
        for _ in range(20):
            p, q = rp.get_world_poses()
            rp.set_world_poses(positions=np.array(p) + np.array([[0.0, 0.0, STEP]]),
                               orientations=q, kinematic_only=True)
            app.update()
        zd = read("D 结果")
    except Exception as exc:
        say("RigidPrim", exc)

    tl.stop()
    z0 = None
    print()
    print("结论对照（起点 z 之后的增量，mm）：A_xform=%s B_kinematic=%s C_RigidBodyView=%s D_RigidPrim=%s"
          % (za, zb, zc, zd))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
