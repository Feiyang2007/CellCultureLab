"""Headless RTX smoke/soak test. Never saves changes into the source USD.

No scene means a local cube baseline. --neutral removes source materials only
in the session layer for a geometry/material comparison. A PNG is evidence of
one rendered view, not full scene correctness or indefinite stability.
"""
import argparse
import json
import os
import struct
import sys
import time
import zlib
from datetime import datetime
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('scene', nargs='?')
parser.add_argument('--neutral', action='store_true')
parser.add_argument('--seconds', type=float, default=300)
parser.add_argument('--output', required=True, help='Output directory, use a new directory for each test')
args = parser.parse_args()
if args.seconds < 1:
    parser.error('--seconds must be at least 1')
scene = Path(args.scene).resolve() if args.scene else None
if scene and not scene.is_file():
    parser.error('Scene does not exist')
out = Path(args.output).resolve()
out.mkdir(parents=True, exist_ok=True)
runtime = Path(os.environ['LOCALAPPDATA']) / 'IsaacSim/5.1/CellCultureLab-render-check'
runtime.mkdir(parents=True, exist_ok=True)
report = {'scene': str(scene), 'neutral': args.neutral, 'completed': False,
          'requested_seconds': args.seconds, 'captures': [], 'started': datetime.now().isoformat(),
          'note': 'Remote references may use the network; headless is not a sandbox or GPU bypass.'}
def save_report():
    (out / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
save_report()
sys.argv = [sys.argv[0], '--portable-root', runtime.as_posix()]
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'create_new_stage': True,
    'renderer': 'RaytracedLighting', 'multi_gpu': False, 'sync_loads': False,
    'limit_cpu_threads': 4, 'width': 960, 'height': 540,
    'extra_args': [f'--/log/file={(out / "kit.log").as_posix()}',
                   '--/rtx/hydra/mdlMaterialWarmup=false']})
try:
    import carb.settings
    import omni.usd
    import omni.replicator.core as rep
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade
    carb.settings.get_settings().set_bool('/rtx/hydra/mdlMaterialWarmup', False)
    context = omni.usd.get_context()
    if scene and not context.open_stage(scene.as_posix()):
        raise RuntimeError('open_stage failed')
    stage = context.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    render = stage.GetPrimAtPath('/Render')
    if render:
        for prim in Usd.PrimRange(render):
            for name in ('omni:rtx:scene:hydra:mdlMaterialWarmup',
                         'omni:rtx:material:db:syncLoads',
                         'omni:rtx:scene:hydra:materialSyncLoads'):
                attr = prim.GetAttribute(name)
                if attr:
                    attr.Set(False)
    if args.neutral:
        # Expose instance children before changing their bindings in this session.
        while True:
            instances = [p for p in stage.Traverse() if p.IsInstance()]
            if not instances:
                break
            for prim in instances:
                prim.SetInstanceable(False)
        prims = list(stage.Traverse())
        material_paths = [p.GetPath() for p in prims if p.IsA(UsdShade.Material)]
        for prim in prims:
            for rel in prim.GetRelationships():
                if rel.GetName().startswith('material:binding'):
                    rel.SetTargets([])
        for path in material_paths:
            stage.OverridePrim(path).SetActive(False)
        material = UsdShade.Material.Define(stage, '/RenderCheck/Neutral')
        shader = UsdShade.Shader.Define(stage, '/RenderCheck/Neutral/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.5))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Gprim):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    if not scene:
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        cube = UsdGeom.Cube.Define(stage, '/World/TestCube')
        cube.CreateSizeAttr(1)
        UsdLux.DomeLight.Define(stage, '/World/TestLight').CreateIntensityAttr(1000)
    config = stage.GetRootLayer().customLayerData.get('cameraSettings', {}).get('Perspective', {})
    eye = Gf.Vec3d(*config.get('position', (3, -4, 3)))
    target = Gf.Vec3d(*config.get('target', (0, 0, 0)))
    camera = UsdGeom.Camera.Define(stage, '/RenderCheck/Camera')
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse())
    camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 1000))
    product = rep.create.render_product(str(camera.GetPath()), (960, 540))
    rgb = rep.AnnotatorRegistry.get_annotator('rgb')
    rgb.attach([product])
    def capture(name):
        data = rgb.get_data()
        if data is None or getattr(data, 'size', 0) == 0:
            raise RuntimeError('RGB annotator returned no pixels')
        height, width, channels = data.shape
        if channels not in (3, 4):
            raise RuntimeError(f'Unexpected RGB shape: {data.shape}')
        pixels = data[:, :, :3].copy()
        def chunk(kind, payload):
            return struct.pack('>I', len(payload)) + kind + payload + struct.pack('>I', zlib.crc32(kind + payload) & 0xffffffff)
        raw = b''.join(b'\x00' + row.tobytes() for row in pixels)
        png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
        png += chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b'')
        (out / name).write_bytes(png)
        report['captures'].append({'file': name, 'width': width, 'height': height,
                                   'minimum': int(pixels.min()), 'maximum': int(pixels.max()),
                                   'mean': float(pixels.mean())})
        save_report()
        print('CAPTURE', name, report['captures'][-1], flush=True)
    started = time.monotonic()
    next_status = 0
    first = False
    while time.monotonic() - started < args.seconds:
        rep.orchestrator.step(rt_subframes=1, pause_timeline=True, delta_time=0.0)
        elapsed = time.monotonic() - started
        if elapsed >= min(60, args.seconds / 2) and not first:
            capture('first.png')
            first = True
        if elapsed >= next_status:
            print(f'RENDER_CHECK {elapsed:.1f}/{args.seconds}s', flush=True)
            next_status = elapsed + 30
    capture('last.png')
    report['render_seconds'] = time.monotonic() - started
    rgb.detach([product])
    product.destroy()
except BaseException as exc:
    report['error'] = repr(exc)
    save_report()
    raise
finally:
    app.close()
report['completed'] = True
save_report()
print('RENDER_CHECK_COMPLETED', flush=True)
