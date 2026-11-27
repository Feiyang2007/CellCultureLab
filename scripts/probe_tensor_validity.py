"""Which launch config gives valid tensor readbacks, a responsive arm, and speed?

For each config this prints: the boot joint angles (garbage = a race between the
GPU physics step and the tensor buffer fill), app.update throughput, whether the
arm actually obeys a commanded joint step, whether the tube free-falls, and
whether a tube pose is still readable with the viewport frozen.
"""
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "outputs" / "gui_review" / "biolab_v06_original_local.usda"
TUBE = "/World/TaskArea/Objects/Tubes/RackPoolLarge/Tube_A01/TubeWithCap/Tube/Tube"
ARM = "/World/Robot/Robot1Mount/ur5e/root_joint"
WRIST = "/World/Robot/Robot1Mount/ur5e/wrist_3_link"
W, H = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) > 2 else (640, 360)
HEADLESS = sys.argv[3] == "1" if len(sys.argv) > 3 else False
AUTHOR = (sys.argv[4] != "noauth") if len(sys.argv) > 4 else True
CFG = "%s%s" % ("headless" if HEADLESS else "%dx%d" % (W, H), "" if AUTHOR else "-noauth")

runtime = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "IsaacSim" / "5.1" / "CellCultureLab"
runtime.mkdir(parents=True, exist_ok=True)
os.environ["TEMP"] = os.environ["TMP"] = str(runtime / "tmp")

from isaacsim import SimulationApp

app = SimulationApp({"headless": HEADLESS, "width": W, "height": H,
                     "window_width": W, "window_height": H,
                     "multi_gpu": False, "limit_cpu_threads": 4,
                     "extra_args": ["--portable-root", str(runtime).replace("\\", "/"),
                                    "--/rtx/hydra/mdlMaterialWarmup=false"]})

import numpy as np
import omni.timeline
import omni.usd
from omni.physics.tensors import create_simulation_view
from pxr import PhysxSchema, Usd


def P(msg):
    print("[CFG %s] %s" % (CFG, msg), flush=True)


ctx = omni.usd.get_context()
ctx.open_stage(str(SCENE))
stage = ctx.get_stage()
if AUTHOR:
    for q in stage.Traverse():
        if q.GetTypeName() == "PhysicsScene":
            api = PhysxSchema.PhysxSceneAPI.Apply(q)
            api.CreateGpuTotalAggregatePairsCapacityAttr(16384)
            api.CreateGpuFoundLostAggregatePairsCapacityAttr(16384)
omni.timeline.get_timeline_interface().play()
for _ in range(60):
    app.update()

view = create_simulation_view("numpy")
art = view.create_articulation_view(ARM)
tb = view.create_rigid_body_view([TUBE])
wr = view.create_rigid_body_view([WRIST])
idx = np.array([0], dtype=np.uint32)
HOME_DEG = [0.0, -100.0, 100.0, -90.0, -90.0, 180.0]


def qraw():
    return np.asarray(art.get_dof_positions(), float)


P("views art=%s wrist=%s tube=%s" % (art.count, wr.count, tb.count))
raw0 = qraw()
tgt = np.zeros(raw0.shape, dtype=np.float32)
for i in range(6):
    tgt[0, i] = np.float32(math.radians(HOME_DEG[i]))


def qdeg():
    return [round(math.degrees(float(x)), 2) for x in qraw()[0][:6]]


def wrist():
    if int(wr.count) < 1:
        raise RuntimeError("wrist rigid-body view matched nothing")
    return np.asarray(wr.get_transforms(), float)[0][0:3]


def tubez():
    if int(tb.count) < 1:
        raise RuntimeError("tube rigid-body view matched nothing")
    return float(np.asarray(tb.get_transforms(), float)[0][2])


P("boot dof_shape=%s q_deg=%s wrist=%s" % (raw0.shape, qdeg(), np.round(wrist(), 4).tolist()))
sane = bool(np.all(np.abs(np.asarray(qdeg())) < 720.0))
P("readback_sane=%s" % sane)

# hold the arm at home, then command pan +8 deg and measure the response
for _ in range(120):
    art.set_dof_position_targets(tgt, idx)
    app.update()
before = wrist().copy()
tgt[0, 0] = np.float32(math.radians(HOME_DEG[0] + 8.0))
for _ in range(90):
    art.set_dof_position_targets(tgt, idx)
    app.update()
moved = float(np.linalg.norm(wrist() - before))
P("actuation: wrist moved %.4f m, q now=%s" % (moved, qdeg()))
tgt[0, 0] = np.float32(math.radians(HOME_DEG[0]))
for _ in range(90):
    art.set_dof_position_targets(tgt, idx)
    app.update()
P("actuation: returned %.4f m from start" % float(np.linalg.norm(wrist() - before)))

t = time.time()
for _ in range(40):
    art.set_dof_position_targets(tgt, idx)
    app.update()
rate = 40.0 / (time.time() - t)
P("throughput %.1f updates/s (render on)" % rate)

z0 = tubez()
tr = np.asarray(tb.get_transforms(), float)
tr[0][2] = z0 + 0.25
tb.set_transforms(tr, idx)
t = time.time()
for _ in range(40):
    art.set_dof_position_targets(tgt, idx)
    app.update()
P("freefall: tube %.4f -> %.4f in 40 steps (%.1f updates/s)" % (
    z0, tubez(), 40.0 / (time.time() - t)))

try:
    from omni.kit.viewport.utility import get_active_viewport
    vp = get_active_viewport()
    if vp is not None:
        vp.updates_enabled = False
        z_before = tubez()
        t = time.time()
        for _ in range(40):
            art.set_dof_position_targets(tgt, idx)
            app.update()
        P("viewport frozen: tube %.4f -> %.4f, q=%s, %.1f updates/s" % (
            z_before, tubez(), qdeg(), 40.0 / (time.time() - t)))
except Exception as e:
    P("viewport-off leg skipped %r" % e)

P("RESULT sane_readback=%s actuated=%s rate=%.1f" % (sane, moved > 0.005, rate))
print("VALIDITY_DONE", flush=True)
app.close()
