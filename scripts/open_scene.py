"""打开实验室场景；运行缓存与日志存放到当前用户的 LocalAppData。"""
import argparse
import os
import sys
from datetime import datetime
from pathlib import Path

project = Path(__file__).resolve().parent.parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('scene', nargs='?', default=str(project / 'assets/scenes/biolab_main_v06.usda'))
parser.add_argument('--capture', help='可选：保存当前视口截图，用于验证渲染')
args, kit_args = parser.parse_known_args()
scene = Path(args.scene).resolve()
if not scene.is_file():
    parser.error(f'找不到场景文件：{scene}')

runtime = Path(os.environ['LOCALAPPDATA']) / 'IsaacSim' / '5.1' / 'CellCultureLab'
logs = runtime / 'logs'
logs.mkdir(parents=True, exist_ok=True)
log_file = logs / ('scene_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.log')
# SimulationApp 默认启用 portable 模式，缓存会落到 D 盘安装目录。
# 显式指定可写的 portable root，覆盖 shader、KVDB、材质和 OGN 缓存路径。
sys.argv = [sys.argv[0], '--portable-root', runtime.as_posix(), *kit_args]
print(f'场景：{scene}\n缓存：{runtime}\n日志：{log_file}', flush=True)

from isaacsim import SimulationApp
app = SimulationApp({
    'headless': False,
    'create_new_stage': True,
    'multi_gpu': False,
    'sync_loads': False,
    'limit_cpu_threads': 4,
    'width': 960,
    'height': 540,
    'extra_args': [
        f'--/log/file={log_file.as_posix()}',
        '--/rtx/hydra/mdlMaterialWarmup=false',
    ],
})

try:
    import omni.usd
    import carb.settings
    from pxr import Usd
    settings = carb.settings.get_settings()
    settings.set_bool('/rtx/hydra/mdlMaterialWarmup', False)
    context = omni.usd.get_context()
    if not context.open_stage(scene.as_posix()):
        raise RuntimeError(f'无法打开场景：{scene}')
    stage = context.get_stage()
    # 场景保存的每视口设置会覆盖启动参数；仅在会话层关闭预热与同步等待。
    # 不写回原 USD，不改变材质或几何。
    render_root = stage.GetPrimAtPath('/Render')
    if render_root:
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for prim in Usd.PrimRange(render_root):
                for name in (
                    'omni:rtx:scene:hydra:mdlMaterialWarmup',
                    'omni:rtx:material:db:syncLoads',
                    'omni:rtx:scene:hydra:materialSyncLoads',
                ):
                    attr = prim.GetAttribute(name)
                    if attr:
                        attr.Set(False)
    print('场景已打开，材质与贴图在后台加载。', flush=True)
    capture_task = None
    if args.capture:
        import asyncio
        import omni.kit.app
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        async def save_capture():
            for _ in range(120):
                await omni.kit.app.get_app().next_update_async()
            await asyncio.sleep(35)
            viewport = get_active_viewport()
            print(f'视口：{viewport.camera_path}, 分辨率：{viewport.resolution}', flush=True)
            output = Path(args.capture).resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            capture = capture_viewport_to_file(viewport, str(output))
            await capture.wait_for_result()
            print(f'截图已保存：{output}', flush=True)
        capture_task = asyncio.ensure_future(save_capture())
    while app.is_running():
        app.update()
        if capture_task is not None and capture_task.done():
            capture_task.result()
            capture_task = None
finally:
    app.close()
