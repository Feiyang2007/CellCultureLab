"""Is physics still stepping when the viewport is switched off, and what does
rendering cost per update? Decides how the IK search loop must be run."""
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
RES = (int(sys.argv[1]) if len(sys.argv) > 1 else 320, int(sys.argv[2]) if len(sys.argv) > 2 else 180)

runtime = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "IsaacSim" / "5.1" / "CellCultureLab"
runtime.mkdir(parents=True, exist_ok=True)
os.environ["TEMP"] = os.environ["TMP"] = str(runtime / "tmp")

from isaacsim import SimulationApp

app = SimulationApp({"headless": False, "width": RES[0], "height": RES[1],
                     "window_width": RES[0], "window_height": RES[1],
                     "multi_gpu": False, "limit_cpu_threads": 4,
                     "extra_args": ["--portable-root", str(runtime).replace("\\", "/"),
                                    "--/rtx/hydra/mdlMaterialWarmup=false"]})

import numpy as np
import omni.timeline
import omni.usd
from omni.kit.viewport.utility import get_active_viewport
from omni.physics.tensors import create_simulation_view

viewport = get_active_viewport()
try:
    print("[PERF] viewport=%s res=%s" % (viewport.viewport_name, viewport.resolution), flush=True)
except Exception as e:
    print("[PERF] viewport attrs failed %r" % e, flush=True)
ctx = omni.usd.get_context()
ctx.open_stage(str(SCENE))
stage = ctx.get_stage()
# the PhysX aggregate pair capacity has to be raised or contacts are dropped
from pxr import PhysxSchema, Sdf, UsdGeom, Gf
for q in stage.Traverse():
    if q.GetTypeName() == "PhysicsScene":
        api = PhysxSchema.PhysxSceneAPI.Apply(q)
        api.CreateGpuTotalAggregatePairsCapacityAttr(16384)
        api.CreateGpuFoundLostAggregatePairsCapacityAttr(16384)
        print("[PERF] capacity authored on", q.GetPath(), flush=True)
omni.timeline.get_timeline_interface().play()
for _ in range(60):
    app.update()

view = create_simulation_view("numpy")
art = view.create_articulation_view(ARM)
tb = view.create_rigid_body_view([TUBE])
wr = view.create_rigid_body_view([WRIST])
idx = np.array([0], dtype=np.uint32)
names = list(art.shared_metatype.dof_names)


def bench(tag, n):
    t = time.time()
    for _ in range(n):
        app.update()
    dt = time.time() - t
    print("[PERF] %s: %d updates in %.1f s -> %.1f/s" % (tag, n, dt, n / dt), flush=True)
    return dt / n


def tube_z():
    return float(np.asarray(tb.get_transforms(), float)[0][2])


def wrist_pos():
    return np.asarray(wr.get_transforms(), float)[0][0:3]


print("[PERF] boot tube_z=%.4f wrist=%s q=%s" % (
    tube_z(), np.round(wrist_pos(), 4).tolist(),
    [round(math.degrees(float(x)), 1) for x in np.asarray(art.get_dof_positions(), float)[0][:6]]), flush=True)

per_on = bench("render ON ", 120)

# free fall while the viewport is disabled
z_rest = tube_z()
tr = np.asarray(tb.get_transforms(), float)
tr[0][2] = z_rest + 0.25
tb.set_transforms(tr, idx)
viewport.updates_enabled = False
per_off = bench("render OFF", 120)
z_after_off = tube_z()
print("[PERF] teleported tube +0.25 then 120 updates with render OFF: z %.4f -> %.4f (fell %.4f m)" % (
    z_rest + 0.25, z_after_off, z_rest + 0.25 - z_after_off), flush=True)

viewport.updates_enabled = True
per_on2 = bench("render ON again", 60)
print("[PERF] SUMMARY res=%s on=%.4f s/update off=%.4f s/update speedup=%.1fx" % (
    RES, per_on, per_off, per_on / max(per_off, 1e-6)), flush=True)
print("PERF_DONE", flush=True)
app.close()
