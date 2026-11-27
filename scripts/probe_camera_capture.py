"""v09 带 RTX 相机能不能取到帧——#21 数据闭环和 GUI 验收都卡在这一步。

为什么要单独试（2026-11-24 台式机）：
    纯物理判读一直是 renderer=None 跑的，因为一旦开 RTX，v09 在
    p_eval6 那次收尾刷了
        [Fatal] [omni.rtx] Out of resource descriptors!
        - [1] SYNC SCOPE: IRenderImpl (paused: no, last tick: 47840 ms ago)
    注意 last tick 47.8 s —— 主循环被 40 个任务的物理吃满，渲染线程整整没被
    tick 过，descriptor 池就是这么涨爆的。取帧必须"每帧都 tick 渲染"，
    所以这里只开一个相机、只跑几十帧，把变量隔离干净。

相机：v03 里已经有 5 个（global_pipette/tube/centrifuge + robot1/2_wrist）。
wrist 挂在 UR5e 的 payload 上，那个 payload 还是远端 omniverse-content-production
链接，离线 pxr 打不开，台式机能不能拉到也得看网络——先只测不依赖机器人的
global_tube_camera。

跑法（台式机）：
    C:\\Users\\73\\IsaacLab\\.venv\\Scripts\\python.exe -u scripts\\probe_camera_capture.py
"""
import inspect
import os
import sys
import time

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda")
N_FRAMES = 30


def find_cam(stage):
    """相机在场景里的真实路径（v03 垫了多层，别把路径写死猜错）。"""
    from pxr import UsdGeom
    hits = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdGeom.Camera)]
    print("场景里的相机 prim %d 个：" % len(hits))
    for h in hits:
        print("   ", h)
    for h in hits:
        if "global_tube" in h:
            return h
    return hits[0] if hits else None


def main():
    ctx = omni.usd.get_context()
    if not ctx.open_stage(SCENE):
        print("打不开场景", SCENE)
        app.close()
        return 2
    stage = ctx.get_stage()
    st = carb.settings.get_settings()
    print("渲染器 active=%s" % st.get("/renderer active"),)
    path = find_cam(stage)
    if not path:
        print("场景里没有相机，先别谈取帧")
        app.close()
        return 2

    import isaacsim.sensors.camera as cammod
    print("isaacsim.sensors.camera 导出：", [n for n in dir(cammod) if not n.startswith("_")])
    from isaacsim.sensors.camera import Camera
    try:
        print("Camera.__init__ 签名：", inspect.signature(Camera.__init__))
    except (TypeError, ValueError) as exc:
        print("签名读不出来：", exc)

    # 6.1 的形参叫 prim_path（5.x 文档里写的 camera_prim_path 会 TypeError）。
    cam = Camera(prim_path=path, name="probe_cam", resolution=(320, 240))
    try:
        cam.initialize()
    except Exception as exc:
        print("initialize 失败：", type(exc).__name__, exc)
        print("可用方法：", [n for n in dir(cam) if not n.startswith("_")])
        app.close()
        return 3

    tl = omni.timeline.get_timeline_interface()
    tl.play()
    got = 0
    t0 = time.time()
    for i in range(1, N_FRAMES + 1):
        app.update()
        frame = None
        for name in ("get_rgba", "get_current_frame", "get_rgb", "get_rgba_async"):
            fn = getattr(cam, name, None)
            if fn is None:
                continue
            try:
                frame = fn()
            except Exception as exc:
                print("  %s() 抛：%s" % (name, type(exc).__name__))
                continue
            if frame is not None:
                got += 1
                if i in (1, 2, 5, 10, N_FRAMES):
                    print("帧 %2d 用 %s() 拿到 %s dtype=%s 用时 %.2fs" % (
                        i, name, getattr(frame, "shape", type(frame)),
                        getattr(frame, "dtype", "-"), time.time() - t0))
                break
        else:
            if i <= 3:
                print("帧 %2d 一个取帧接口都没返回数据，方法表：%s" % (
                    i, [n for n in dir(cam) if "get" in n or "update" in n]))
    print("汇总：%d/%d 帧取到，方法表 %s" % (
        got, N_FRAMES, [n for n in dir(cam) if not n.startswith("_")]))
    tl.stop()
    app.close()
    return 0 if got >= 5 else 4


if __name__ == "__main__":
    sys.exit(main())
