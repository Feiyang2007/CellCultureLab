"""Manual GUI control for the lab robot: 6 arm joint sliders + gripper slider.

Runs the local-robot GUI scene, plays physics, and writes joint position targets
every frame. Intended for hand-driven interaction trials (grasp, lid opening),
not for automated acceptance. Use --arm 1|2 to choose the robot.
"""
import argparse
import os
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--scene", default=None)
parser.add_argument("--arm", type=int, default=1, choices=(1, 2))
args = parser.parse_args()

ROOT = Path(__file__).resolve().parents[1]
SCENE = Path(args.scene) if args.scene else ROOT / "outputs/gui_review/biolab_v06_original_local.usda"
ARM_ROOT = f"/World/Robot/Robot{args.arm}Mount/ur5e/root_joint"

runtime = Path(os.environ["LOCALAPPDATA"]) / "IsaacSim" / "5.1" / "CellCultureLab"
runtime.mkdir(parents=True, exist_ok=True)
os.environ["TEMP"] = os.environ["TMP"] = str(runtime / "tmp")
Path(os.environ["TEMP"]).mkdir(parents=True, exist_ok=True)

from isaacsim import SimulationApp

app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
    "window_width": 1600,
    "window_height": 950,
    "multi_gpu": False,
    "sync_loads": False,
    "limit_cpu_threads": 4,
    "extra_args": [
        "--portable-root", str(runtime).replace("\\", "/"),
        "--/rtx/hydra/mdlMaterialWarmup=false",
    ],
})

import numpy as np
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.physics.tensors import create_simulation_view
from pxr import UsdLux

ctx = omni.usd.get_context()
if not ctx.open_stage(str(SCENE)):
    raise RuntimeError(f"open_stage failed: {SCENE}")
stage = ctx.get_stage()
stage.SetEditTarget(stage.GetSessionLayer())
UsdLux.DomeLight.Define(stage, "/ManualReviewLight").CreateIntensityAttr(1000)

from isaacsim.core.utils.viewports import set_camera_view

set_camera_view(eye=[-3.2, -3.2, 2.2], target=[-1.5, -1.5, 0.95])

tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(90):
    app.update()

view = create_simulation_view("numpy")
art = view.create_articulation_view(ARM_ROOT)
if art is None:
    raise RuntimeError(f"no articulation at {ARM_ROOT}")
names = list(art.shared_metatype.dof_names)
idx = np.array([0], dtype=np.uint32)
arm_idx = [i for i, n in enumerate(names) if i < 6]
finger_idx = [i for i, n in enumerate(names) if n == "finger_joint"]
if len(arm_idx) != 6 or not finger_idx:
    raise RuntimeError(f"unexpected dofs: {names}")
finger_idx = finger_idx[0]

deg_state = [0.0] * 6
gripper_deg = 0.0
paused = False
ui_labels = []
sliders = []


def current_degrees():
    q = art.get_dof_positions().copy()[0]
    return [float(np.degrees(q[i])) for i in range(6)]


def toggle_pause():
    global paused
    paused = not paused
    if paused:
        tl.pause()
    else:
        tl.play()


def sync_sliders():
    for i, (s, d) in enumerate(zip(sliders, current_degrees())):
        s.model.set_value(d)
    q = art.get_dof_positions().copy()[0]
    gripper_slider.model.set_value(float(np.degrees(q[finger_idx])))


window = ui.Window(f"Manual Robot Control - Robot{args.arm}", width=380, height=430)
with window.frame:
    with ui.VStack(spacing=6, height=0):
        ui.Label(f"Articulation: {ARM_ROOT}", height=22)
        ui.Label("DOFs: " + ", ".join(n.replace("_joint", "") for n in names),
                 height=44, word_wrap=True)
        ui.Button("Play / Pause (physics)", height=32, clicked_fn=toggle_pause)
        ui.Button("Sync sliders to current pose", height=28, clicked_fn=sync_sliders)
        ui.Spacer(height=6)
        for i in range(6):
            ui.Label(f"J{i + 1}  {names[i]}", height=18)
            s = ui.FloatSlider(min=-180.0, max=180.0, step=0.5, height=22)
            s.model.set_value(float(np.degrees(art.get_dof_positions().copy()[0][i])))
            sliders.append(s)
        ui.Spacer(height=6)
        ui.Label("Gripper (0=open, 47=closed)", height=18)
        gripper_slider = ui.FloatSlider(min=0.0, max=47.0, step=0.5, height=22)
        gripper_slider.model.set_value(float(np.degrees(art.get_dof_positions().copy()[0][finger_idx])))
        ui.Spacer(height=4)
        status = ui.Label("", height=40)
        ui_labels.append(status)

print(f"MANUAL_GUI_READY arm={ARM_ROOT} dofs={names}", flush=True)

while app.is_running():
    app.update()
    if not paused:
        target = art.get_dof_positions().copy()
        for i, s in enumerate(sliders):
            deg = float(s.model.get_value_as_float())
            target[0, arm_idx[i]] = np.radians(deg)
        target[0, finger_idx] = np.radians(float(gripper_slider.model.get_value_as_float()))
        art.set_dof_position_targets(target.astype(np.float32), idx)
        q = art.get_dof_positions().copy()[0]
        status.text = "q(deg): " + " ".join(f"{np.degrees(q[i]):.1f}" for i in range(6)) + \
                      f" | finger {np.degrees(q[finger_idx]):.1f}"

app.close()
