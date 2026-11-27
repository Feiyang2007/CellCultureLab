"""Actuation diagnostics for the v06 lab scene: does the UR5e respond at all?

One boot, several independent channels compared:
  * articulation tensor view (positions / velocities / targets)
  * rigid body tensor view of wrist_3_link
  * Fabric world pose through isaacsim.core.prims.XFormPrim
  * a free-fall test on the centrifuge tube, to prove the timeline is stepping
"""
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "outputs" / "gui_review" / "biolab_v06_original_local.usda"
ARM = "/World/Robot/Robot1Mount/ur5e/root_joint"
WRIST = "/World/Robot/Robot1Mount/ur5e/wrist_3_link"
TUBE = "/World/TaskArea/Objects/Tubes/RackPoolLarge/Tube_A01/TubeWithCap/Tube/Tube"
HOME_DEG = [0.0, -100.0, 100.0, -90.0, -90.0, 180.0]

runtime = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "IsaacSim" / "5.1" / "CellCultureLab"
runtime.mkdir(parents=True, exist_ok=True)
os.environ["TEMP"] = os.environ["TMP"] = str(runtime / "tmp")

from isaacsim import SimulationApp

app = SimulationApp({"headless": False, "width": 1280, "height": 720, "multi_gpu": False,
                     "limit_cpu_threads": 4,
                     "extra_args": ["--portable-root", str(runtime).replace("\\", "/"),
                                    "--/rtx/hydra/mdlMaterialWarmup=false"]})

import numpy as np
import omni.timeline
import omni.usd
from pxr import Usd, UsdGeom


def P(msg):
    print("[ACT] " + str(msg), flush=True)


ctx = omni.usd.get_context()
ctx.open_stage(str(SCENE))
stage = ctx.get_stage()
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(60):
    app.update()

from omni.physics.tensors import create_simulation_view

view = create_simulation_view("numpy")
art = view.create_articulation_view(ARM)
names = list(art.shared_metatype.dof_names)
idx = np.array([0], dtype=np.uint32)
P("dofs=%s" % names)
P("playing=%s" % tl.is_playing())
try:
    import omni.physx
    pi = omni.physx.get_physics_interface()
    P("physx step_count=%s enable_ccd=%s" % (getattr(pi, "step_count", "?"), None))
except Exception as e:
    P("physx interface failed %r" % e)

wr = view.create_rigid_body_view([WRIST])
tb = view.create_rigid_body_view([TUBE])
P("views: wrist=%s tube=%s" % (wr.count, tb.count))


def snapshot(tag):
    q = np.asarray(art.get_dof_positions(), float)[0]
    v = np.asarray(art.get_dof_velocities(), float)[0]
    w = np.asarray(wr.get_transforms(), float)[0]
    t = np.asarray(tb.get_transforms(), float)[0]
    P("%s: q_deg=%s" % (tag, [round(math.degrees(float(x)), 2) for x in q[:6]]))
    P("%s: |v|max=%.4f  wrist_xyz=(%.4f,%.4f,%.4f)  tube_xyz=(%.4f,%.4f,%.4f)" % (
        tag, float(np.abs(v).max()), w[0], w[1], w[2], t[0], t[1], t[2]))
    return w.copy()


w0 = snapshot("boot")

# ---- free fall check: lift the tube and see whether it comes back down
z0 = float(np.asarray(tb.get_transforms(), float)[0][2])
tr = np.asarray(tb.get_transforms(), float)
tr[0][2] = z0 + 0.25
tb.set_transforms(tr, np.array([0], dtype=np.uint32))
for _ in range(40):
    app.update()
z1 = float(np.asarray(tb.get_transforms(), float)[0][2])
P("FREEFALL tube z %.4f -> teleported %.4f -> after 40 steps %.4f (fell=%.3f m)" % (
    z0, z0 + 0.25, z1, (z0 + 0.25) - z1))

# ---- command the arm through the tensor targets
tgt = np.zeros((1, len(names)), dtype=np.float32)
for i in range(6):
    tgt[0, i] = np.float32(math.radians(HOME_DEG[i]))
tgt[0, 0] = np.float32(math.radians(HOME_DEG[0] + 40.0))
for _ in range(150):
    art.set_dof_position_targets(tgt, idx)
    app.update()
w1 = snapshot("after_pan+40deg_tensor")

# ---- same through the USD drive attribute (degrees)
prim = stage.GetPrimAtPath("/World/Robot/Robot1Mount/ur5e/joints/shoulder_pan_joint")
attr = prim.GetAttribute("drive:angular:physics:targetPosition")
P("pan targetPosition attr exists=%s value=%s" % (bool(attr), attr.Get() if attr else None))
if attr:
    attr.Set(40.0)
    for _ in range(150):
        app.update()
    w2 = snapshot("after_pan+40deg_usd_drive")

# ---- Fabric route
try:
    from isaacsim.core.prims import XFormPrim
    xp = XFormPrim(prim_paths_expr=[WRIST], name="wrist_probe")
    pos, ori = xp.get_world_pose()
    P("XFormPrim wrist pos=%s ori=%s" % (np.round(pos[0], 4).tolist(), np.round(ori[0], 4).tolist()))
except Exception as e:
    P("XFormPrim route failed %r" % e)

P("DISPLACEMENTS wrist boot->tensor=%.4f m, tensor->usd=%.4f m" % (
    float(np.linalg.norm(w1[0:3] - w0[0:3])),
    float(np.linalg.norm(w2[0:3] - w1[0:3])) if "w2" in dir() else -1))
P("ACTUATION_PROBE_DONE")
app.close()
