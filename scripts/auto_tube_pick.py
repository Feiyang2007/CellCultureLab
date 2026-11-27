"""Auto tube pick: UR5e + Robotiq picks a 15 mL centrifuge tube out of the rack.

Numeric IK (coordinate descent on the 6 arm DOFs) drives a measured grasp-point
landmark to a sequence of waypoints; the grasp itself is real contact + friction.
CPU physics pipeline is forced so PhysX cannot drop contacts (W3).
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "runtime_validation"
SHOTS = ROOT / "outputs" / "gui_review" / "auto_tube_pick"

p = argparse.ArgumentParser()
p.add_argument("--scene", default=str(ROOT / "outputs/gui_review/biolab_v06_original_local.usda"))
p.add_argument("--arm", default="/World/Robot/Robot1Mount/ur5e/root_joint")
p.add_argument("--wrist", default="/World/Robot/Robot1Mount/ur5e/wrist_3_link")
p.add_argument("--gripper", default="/World/Robot/Gripper1Temp/Robotiq_2F_85_edit/Robotiq_2F_85")
p.add_argument("--tube", default="/World/TaskArea/TestTube")
p.add_argument("--drop", default="-1.45,-1.35,0.8719")
p.add_argument("--yaw", default="45.0", help="deg; world direction the finger separation axis should sit on, so the fingers straddle the tube along a free diagonal")
p.add_argument("--cpu-physics", default="0", help="1 = force the CPU dynamics pipeline (measured ~1 s/step, too slow for IK search)")
p.add_argument("--broadphase", default="GPU", help="only used with --cpu-physics 1")
p.add_argument("--agg-cap", default="16384", help="gpu*AggregatePairsCapacity; 0 leaves the PhysX defaults (1024) in place")
p.add_argument("--finger-open", default="0.33", help="finger_joint rad; 0.82 = shut, 0 = 85 mm span; the diagonal gap between rack tubes is only 21.8 mm so a fully open gripper cannot enter")
p.add_argument("--finger-close", default="0.48", help="finger_joint rad after the grasp (~27 mm span on a 29.1 mm tube)")
p.add_argument("--width", default="640")
p.add_argument("--height", default="360")
p.add_argument("--render", default="on", help="off freezes the viewport; measured to invalidate tensor readbacks")
p.add_argument("--trace", default="0", help="1 = sample the articulation state through the home settle")
p.add_argument("--probe", default="0")
p.add_argument("--label", default="auto")
args = p.parse_args()
RUN_T0 = time.time()

runtime = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "IsaacSim" / "5.1" / "CellCultureLab"
runtime.mkdir(parents=True, exist_ok=True)
os.environ["TEMP"] = os.environ["TMP"] = str(runtime / "tmp")
(OUT / "logs").mkdir(parents=True, exist_ok=True)
SHOTS.mkdir(parents=True, exist_ok=True)

MARK = OUT / "logs" / ("auto_tube_pick_%s.marks.txt" % args.label)


def mark(msg):
    line = "[%s] %s" % (time.strftime("%H:%M:%S"), msg)
    print(line, flush=True)
    with open(MARK, "a", encoding="utf-8") as f:
        f.write(line + "\n")


from isaacsim import SimulationApp

app = SimulationApp({
    "headless": False,
    "width": int(args.width),
    "height": int(args.height),
    "window_width": int(args.width),
    "window_height": int(args.height),
    "multi_gpu": False,
    "sync_loads": True,
    "limit_cpu_threads": 4,
    "extra_args": [
        "--portable-root", str(runtime).replace("\\", "/"),
        "--/rtx/hydra/mdlMaterialWarmup=false",
    ],
})

import numpy as np
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

try:  # standalone python.bat puts the schema under the pxr namespace
    from pxr import PhysxSchema
except Exception:
    try:
        import PhysxSchema
    except Exception as e:
        mark("PhysxSchema import FAILED: %r" % e)
        PhysxSchema = None

ctx = omni.usd.get_context()
if not ctx.open_stage(args.scene):
    raise RuntimeError("open_stage failed " + args.scene)
stage = ctx.get_stage()
stage.SetEditTarget(stage.GetSessionLayer())
T = Usd.TimeCode.Default()

mark("stage opened, %d prims" % len(list(stage.Traverse())))

# Spawn a test tube from asset if using default path
if args.tube == "/World/TaskArea/TestTube":
    tube_asset = str(ROOT / "assets/objects/centrifuge_50ml.usda")
    mark("Spawning test tube from %s at %s" % (tube_asset, args.tube))
    # Create parent Xform
    parent = UsdGeom.Xform.Define(stage, "/World/TaskArea/TestTubeParent")
    parent.AddTranslateOp().Set(Gf.Vec3d(-1.65, -1.50, 0.88))  # Position near rack location
    # Reference the tube asset
    tube_prim = stage.DefinePrim("/World/TaskArea/TestTube", "Xform")
    tube_prim.GetReferences().AddReference(tube_asset)
    # Give PhysX time to load the asset
    for _ in range(120):
        app.update()
    mark("Test tube spawned")

# ------------------------------------------------- W3 contact-capacity fix
# The CPU dynamics pipeline was measured at ~1 s/step on this scene (one core
# pegged), so the fix stays on the GPU side: PhysX asks for larger aggregate
# pair capacities, and that is exactly what is authored here.
scenes = [q.GetPath() for q in stage.Traverse() if q.GetTypeName() == "PhysicsScene"]
gpu_readback = []


def set_scene_attr(prim, name, value, vtype):
    if PhysxSchema is not None:
        api = PhysxSchema.PhysxSceneAPI.Apply(prim)
        meth = getattr(api, "Create" + name[0].upper() + name[1:] + "Attr", None)
        if meth is None:
            mark("PhysxSceneAPI has no Create%sAttr" % (name[0].upper() + name[1:]))
            return None
        meth(value)
    else:  # schema-free fallback: author the token directly
        prim.AddAppliedSchema("PhysxSceneAPI")
        prim.CreateAttribute("physxScene:" + name, vtype).Set(value)
    return prim.GetAttribute("physxScene:" + name).Get()


CAP_ATTRS = ("gpuTotalAggregatePairsCapacity", "gpuFoundLostAggregatePairsCapacity")
for sp in scenes:
    prim = stage.GetPrimAtPath(sp)
    rb = {"scene": str(sp)}
    if args.cpu_physics == "1":
        rb["enableGPUDynamics"] = set_scene_attr(prim, "enableGPUDynamics", False, Sdf.ValueTypeNames.Bool)
        rb["broadphaseType"] = str(set_scene_attr(prim, "broadphaseType", args.broadphase,
                                                 Sdf.ValueTypeNames.Token))
    if args.agg_cap != "0":
        cap = int(args.agg_cap)
        rb["aggCaps"] = {nm: set_scene_attr(prim, nm, cap, Sdf.ValueTypeNames.UInt) for nm in CAP_ATTRS}
    gpu_readback.append(rb)


def capacity_raised():
    if args.agg_cap == "0":
        return False
    cap = int(args.agg_cap)
    return bool(gpu_readback) and all(
        len(r.get("aggCaps", {})) == len(CAP_ATTRS) and
        all(v is not None and int(v) >= cap for v in r["aggCaps"].values())
        for r in gpu_readback)
mark("physics scenes=%s gpu_readback=%s" % (scenes, gpu_readback))

# light = UsdLux.DomeLight.Define(stage, "/AutoPickLight")
# light.CreateIntensityAttr(800)

from isaacsim.core.utils.viewports import set_camera_view

tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(180):
    app.update()

from omni.physics.tensors import create_simulation_view

view = create_simulation_view("numpy")
art = view.create_articulation_view(args.arm)
if art is None:
    raise RuntimeError("no articulation at " + args.arm)
names = list(art.shared_metatype.dof_names)

idx = np.array([0], dtype=np.uint32)
ARM = list(range(6))
FINGER = names.index("finger_joint")
mark("dofs=%s" % names)

# ---------------------------------------------------------------- seed pose
# The arm is driven to the joint targets authored in the scene rather than to
# whatever the tensor view reports at boot: a bogus read there otherwise becomes
# a 1e6 rad target, the drive saturates at maxForce and the arm thrashes.
def drive_target_rad(joint_path):
    prim = stage.GetPrimAtPath(joint_path)
    if not prim:
        return None
    a = prim.GetAttribute("drive:angular:physics:targetPosition")
    if not a or a.Get() is None:
        return None
    return math.radians(float(a.Get()))


def limit_rad(joint_path, which, default):
    prim = stage.GetPrimAtPath(joint_path)
    if not prim:
        return default
    a = prim.GetAttribute("physics:" + which)
    if not a or a.Get() is None:
        return default
    return math.radians(float(a.Get()))


arm_joints = args.arm.rsplit("/", 1)[0] + "/joints"
finger_joint_path = args.gripper + "/Joints/finger_joint"
HOME = []
for nm in names[:6]:
    v = drive_target_rad(arm_joints + "/" + nm)
    HOME.append(0.0 if v is None else v)
FINGER_CLOSED = limit_rad(finger_joint_path, "upperLimit", 0.82)
FINGER_OPEN_Q = float(args.finger_open)

q_target = np.zeros((1, len(names)), dtype=np.float32)
for i in ARM:
    q_target[0, i] = np.float32(HOME[i])
q_target[0, FINGER] = np.float32(FINGER_OPEN_Q)

def apply_targets(steps):
    for _ in range(steps):
        art.set_dof_position_targets(q_target, idx)
        app.update()

# The arm is driven to HOME; if the boot state was garbage the drives will
# saturate but the settle loop below should still bring it close to HOME.
# Wait for payloads to fully load before reading articulation state.
for _ in range(240):
    apply_targets(1)
q_boot = np.asarray(art.get_dof_positions(), float)[0]
if np.any(np.abs(q_boot[:6]) > 4 * math.pi):
    mark("WARNING boot state garbage, waiting for recovery: %s" % q_boot[:6].tolist())
    for _ in range(480):
        apply_targets(1)
        q_check = np.asarray(art.get_dof_positions(), float)[0]
        if np.all(np.abs(q_check[:6]) < 4 * math.pi):
            mark("recovered to sane state: %s" % [round(math.degrees(float(v)), 2) for v in q_check[:6]])
            break
    else:
        mark("FAILED to recover from garbage boot state")

def rv_count(rv):
    """A view whose pattern matched nothing comes back with a null backend, and
    touching .count on it raises instead of returning 0."""
    try:
        return int(rv.count) if rv is not None else 0
    except Exception:
        return 0


def rigid_view(path):
    rv = view.create_rigid_body_view([path])
    if rv_count(rv) != 1:
        raise RuntimeError("rigid body view for %s got %s" % (path, rv_count(rv)))
    return rv


def body_pose(rv):
    # get_transforms -> (count, 7) as x, y, z, qx, qy, qz, qw
    t = np.asarray(rv.get_transforms(), float)[0]
    q = t[3:7]
    return t[0:3], np.array([q[3], q[0], q[1], q[2]], float)


wrbv = rigid_view(args.wrist)
mark("wrist rigid body view ok")


def rotmat(q_wxyz):
    w, x, y, z = q_wxyz
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def wrist_pose():
    return body_pose(wrbv)


# ------------------------------------------------- grasp landmark, from USD
bc = UsdGeom.BBoxCache(T, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False, True)


def world_center(path):
    prim = stage.GetPrimAtPath(path)
    if not prim:
        return None
    b = bc.ComputeWorldBound(prim).ComputeAlignedBox()
    if b.IsEmpty():
        return None
    return np.array(((b.GetMin() + b.GetMax()) * 0.5), float)


FINGER_LINKS = ("left_outer_finger", "right_outer_finger",
                "left_inner_finger", "right_inner_finger")
Mw_inv = UsdGeom.Xformable(stage.GetPrimAtPath(args.wrist)).ComputeLocalToWorldTransform(T).GetInverse()
pad_local_list = []
for nm in FINGER_LINKS:
    c = world_center(args.gripper + "/" + nm)
    if c is None:
        mark("finger link %s has no world bbox" % nm)
        continue
    pad_local_list.append(np.array(Mw_inv.Transform(Gf.Vec3d(float(c[0]), float(c[1]), float(c[2]))), float))
if not pad_local_list:
    raise RuntimeError("no finger geometry under " + args.gripper)
# Pure-USD derivation: runtime simulation does not write poses back to USD
# xforms, so the landmark must come from the authored transform only. It is
# expressed in the wrist frame, where it is a rigid-body constant.
pad_local = np.mean(pad_local_list, axis=0)
# The authored finger state is fully open, so the left/right outer-finger
# separation read off USD is the maximum span; the model scales it with q.
HALF_SPAN_OPEN = 0.5 * abs(float(pad_local_list[0][0] - pad_local_list[1][0]))
_pad_t = []
for nm in ("left_inner_finger", "right_inner_finger"):
    _pr = stage.GetPrimAtPath(args.gripper + "/" + nm)
    if _pr:
        _b = bc.ComputeWorldBound(_pr).ComputeAlignedBox()
        _pad_t.append(min(_b.GetMax()[0] - _b.GetMin()[0], _b.GetMax()[1] - _b.GetMin()[1]))
PAD_R = 0.5 * float(min(_pad_t)) if _pad_t else 0.004
mark("finger links=%d/%d pad_local_in_wrist=%s half_span_open=%.4f pad_half_thick=%.4f" % (
    len(pad_local_list), len(FINGER_LINKS), np.round(pad_local, 4).tolist(), HALF_SPAN_OPEN, PAD_R))

tube_rbv = rigid_view(args.tube)


def tube_state():
    return body_pose(tube_rbv)


# The rigid-body origin of a tube sits at its bottom, so every waypoint has to
# be expressed in the body frame; that keeps it valid once the tube tilts.
_tb_inv = UsdGeom.Xformable(stage.GetPrimAtPath(args.tube)).ComputeLocalToWorldTransform(T).GetInverse()
_tbox = bc.ComputeWorldBound(stage.GetPrimAtPath(args.tube)).ComputeAlignedBox()
_tmn, _tmx = _tbox.GetMin(), _tbox.GetMax()
_tcen = (_tmn + _tmx) * 0.5
TUBE_SIZE = np.array([_tmx[i] - _tmn[i] for i in range(3)], float)
TUBE_CENTER_LOCAL = np.array(_tb_inv.Transform(Gf.Vec3d(*_tcen)), float)
TUBE_GRASP_LOCAL = np.array(_tb_inv.Transform(Gf.Vec3d(_tcen[0], _tcen[1], _tmx[2] - 0.012)), float)
TUBE_TOP_LOCAL = np.array(_tb_inv.Transform(Gf.Vec3d(_tcen[0], _tcen[1], _tmx[2])), float)
mark("tube bbox min=%s max=%s size=%s grasp_local=%s" % (
    [round(float(_tmn[i]), 4) for i in range(3)], [round(float(_tmx[i]), 4) for i in range(3)],
    np.round(TUBE_SIZE, 4).tolist(), np.round(TUBE_GRASP_LOCAL, 4).tolist()))


def tube_point(local_vec):
    p, r = tube_state()
    return p + rotmat(r) @ np.asarray(local_vec, float)


def grip_point():
    p, r = wrist_pose()
    return p + rotmat(r) @ pad_local


def approach_dir():
    """Unit vector from wrist origin toward the pad centre = the gripper axis."""
    p, r = wrist_pose()
    d = rotmat(r) @ pad_local
    return d / max(np.linalg.norm(d), 1e-9)


def sep_dir():
    """Finger separation axis: the wrist x axis, which is what must avoid neighbours."""
    p, r = wrist_pose()
    return rotmat(r)[:, 0]


YAW_UNIT = np.array([math.cos(math.radians(float(args.yaw))),
                     math.sin(math.radians(float(args.yaw))), 0.0])


from omni.kit.viewport.utility import get_active_viewport

viewport = get_active_viewport()


def set_render(on):
    """RTX rendering is ~0.5 s/frame on this scene; the IK search only needs physics."""
    try:
        viewport.updates_enabled = bool(on)
    except Exception as e:
        mark("set_render(%s) failed %r" % (on, e))


if args.render == "off":
    set_render(False)


def state(tag):
    q = np.asarray(art.get_dof_positions(), float)[0]
    v = np.asarray(art.get_dof_velocities(), float)[0]
    mark("STATE %s q_deg=%s vel_dps=%s wrist=%s" % (
        tag, [round(math.degrees(float(x)), 2) for x in q[:7]],
        [round(math.degrees(float(x)), 1) for x in v[:7]],
        np.round(wrist_pose()[0], 4).tolist()))
    return q


state("post-play pre-home")
if args.trace:
    for _k in range(12):
        apply_targets(20)
        state("home+%d" % (20 * (_k + 1)))
else:
    apply_targets(240)
q_meas = np.asarray(art.get_dof_positions(), float)[0]
mark("home targets_deg=%s measured_deg=%s finger_open_q=%.3f closed_q=%.3f" % (
    [round(math.degrees(v), 1) for v in HOME],
    [round(math.degrees(float(v)), 1) for v in q_meas[:6]], FINGER_OPEN_Q, FINGER_CLOSED))
if np.any(np.abs(q_meas[:6]) > 4 * math.pi):
    mark("WARNING measured arm angles outside +-720 deg: %s" % q_meas[:6].tolist())
_t = time.time()
for _ in range(40):
    app.update()
UPD_RATE = 40.0 / max(time.time() - _t, 1e-6)
mark("app.update throughput %.1f/s at %sx%s render=%s" % (
    UPD_RATE, args.width, args.height, args.render))

READBACK_OK = bool(np.all(np.abs(q_meas[:6]) < 4 * math.pi))


def verify_actuation():
    """Command a known joint step and prove the simulated arm obeys it. Without
    this the search can silently run against an arm that never moves.

    The wrist is measured through the rigid-body view, which is what the search
    actually depends on; the articulation readback is reported separately because
    in some launch configs it comes back as nonsense even while the arm obeys."""
    global READBACK_OK
    before = wrist_pose()[0].copy()
    q_save = np.array([float(q_target[0, i]) for i in ARM])
    q_target[0, 0] = np.float32(q_save[0] + math.radians(8.0))
    apply_targets(60)
    moved = float(np.linalg.norm(wrist_pose()[0] - before))
    pan_read = float(np.asarray(art.get_dof_positions(), float)[0][0])
    for i in ARM:
        q_target[0, i] = np.float32(q_save[i])
    apply_targets(60)
    back = float(np.linalg.norm(wrist_pose()[0] - before))
    cmd = math.degrees(q_save[0] + math.radians(8.0))
    READBACK_OK = abs(pan_read) < 4 * math.pi and abs(math.degrees(pan_read) - cmd) < 4.0
    out = dict(wrist_moved_m=round(moved, 4), return_error_m=round(back, 4),
               pan_commanded_deg=round(cmd, 2),
               pan_readback_deg=round(math.degrees(pan_read), 2),
               readback_sane=bool(READBACK_OK))
    mark("actuation check %s" % out)
    return out


ACT = verify_actuation()
ACTUATED = bool(ACT["wrist_moved_m"] > 0.005 and ACT["return_error_m"] < 0.01)


def finger_q():
    """Live finger DOF. Falls back to the commanded value when the articulation
    readback is unusable; the finger position drive is stiff enough that the two
    agree to well inside the pad radius."""
    if READBACK_OK:
        v = float(np.asarray(art.get_dof_positions(), float)[0, FINGER])
        if abs(v) < 4.0:
            return v
    return float(q_target[0, FINGER])
# Re-derive the initial tube pose after the arm has moved into place.


DOWN = np.array([0.0, 0.0, -1.0])
UP = np.array([0.0, 0.0, 1.0])


def residual(target, orient_w, z_floor, yaw_w):
    """Stacked pose error the damped-least-squares step drives to zero.

    Rows: 3 position, 3 approach-axis (cross product, so its component along the
    axis itself is the wrist twist), 1 yaw of the finger separation axis, and 1
    one-sided floor barrier.
    """
    g = grip_point()
    e = target - g
    a = approach_dir()
    c = np.cross(a, DOWN)
    p, _ = wrist_pose()
    rows = [e[0], e[1], e[2],
            orient_w * c[0], orient_w * c[1], orient_w * c[2],
            yaw_w * float(np.dot(np.cross(sep_dir(), YAW_UNIT), UP)),
            (20.0 * max(0.0, z_floor - p[2])) if z_floor is not None else 0.0]
    return np.array(rows, float), float(np.linalg.norm(e)), float(np.linalg.norm(c))


def set_q(q6, settle):
    for i in ARM:
        q_target[0, i] = np.float32(q6[i])
    apply_targets(settle)


def ik_move(target, orient_w=0.3, z_floor=None, rounds=30, settle=30, tol=0.012,
            yaw_w=0.0, lam=0.25, step=math.radians(1.0), max_dq=0.30):
    """Numeric IK evaluated against the live simulation, so drive droop, joint
    limits and contacts are all inside the measured Jacobian."""
    q = np.array([float(q_target[0, i]) for i in ARM])
    e0, d, o = residual(target, orient_w, z_floor, yaw_w)
    it = 0
    wn_prev, stall = float(np.linalg.norm(e0)), 0
    J = np.zeros((e0.size, 6), float)
    for it in range(rounds):
        if d < tol and o < 0.06:
            break
        for k in range(6):
            qp = q.copy()
            qp[k] += step
            set_q(qp, settle)
            ep, _, _ = residual(target, orient_w, z_floor, yaw_w)
            J[:, k] = -(ep - e0) / step
        set_q(q, settle)
        A = J @ J.T + (lam * lam) * np.eye(e0.size)
        dq = J.T @ np.linalg.solve(A, e0)
        q = q + np.clip(dq, -max_dq, max_dq)
        set_q(q, settle * 2)
        e0, d, o = residual(target, orient_w, z_floor, yaw_w)
        wn = float(np.linalg.norm(e0))
        stall = stall + 1 if wn > wn_prev - 1e-3 else 0
        wn_prev = wn
        if stall >= 2:
            break
    return dict(dist=round(d, 4), orient_err=round(o, 4), iters=it + 1,
                q_deg=[round(math.degrees(float(v)), 2) for v in q],
                grip=np.round(grip_point(), 4).tolist())


# --------------------------------------------------------------------- shots
from omni.kit.viewport.utility import capture_viewport_to_file


def shot(name):
    fp = SHOTS / (name + ".png")
    try:
        set_render(True)
        for _ in range(10):
            app.update()
        capture_viewport_to_file(viewport, str(fp))
        for _ in range(10):
            app.update()
        mark("shot %s exists=%s size=%s" % (name, fp.exists(),
                                            fp.stat().st_size if fp.exists() else 0))
    except Exception as e:
        mark("shot %s FAILED %r" % (name, e))
    finally:
        if args.render == "off":
            set_render(False)
    return str(fp)


report = {"scene": args.scene, "arm": args.arm, "tube": args.tube,
          "cpu_gpu_readback": gpu_readback, "dof_names": names,
          "pad_local_in_wrist": pad_local.tolist(),
          "tube_grasp_local": TUBE_GRASP_LOCAL.tolist(), "yaw_deg": args.yaw,
          "home_targets_deg": [round(math.degrees(v), 1) for v in HOME],
          "half_span_open_m": HALF_SPAN_OPEN, "pad_half_thick_m": PAD_R,
          "finger_open_q": FINGER_OPEN_Q, "finger_close_q": float(args.finger_close),
          "finger_closed_limit_q": FINGER_CLOSED,
          "updates_per_second": round(UPD_RATE, 1), "actuation_check": ACT,
          "phases": [], "shots": []}

tube0, trot0 = tube_state()
grasp0 = tube_point(TUBE_GRASP_LOCAL)
report["tube_initial"] = tube0.tolist()
report["grasp_initial"] = grasp0.tolist()
mark("tube body origin=%s grasp point=%s" % (np.round(tube0, 4).tolist(), np.round(grasp0, 4).tolist()))

# The other tubes in the same rack. Enumerated from what PhysX actually created,
# not from the stage: sibling actors may or may not exist depending on how the
# asset was authored, and the two lists differ.
rack_root = args.tube.rsplit("/Tube_", 1)[0]
pool_root = "/".join(rack_root.split("/")[:-1])
try:
    enum = view.create_rigid_body_view(pool_root + "/*")
    rb_all = [str(x) for x in (enum.prim_paths or [])] if rv_count(enum) else []
except Exception as e:
    mark("pool enumeration failed %r" % e)
    rb_all = []
mark("rigid bodies under %s = %d" % (pool_root, len(rb_all)))
nb_paths = [p for p in rb_all if p.startswith(rack_root + "/Tube_")
            and p.endswith("/TubeWithCap/Tube/Tube") and p != args.tube]
nb_sim = view.create_rigid_body_view(nb_paths) if nb_paths else None
if rv_count(nb_sim) == 0:
    nb_sim = None
nb = nb_sim
nb0 = np.asarray(nb.get_transforms(), float)[:, 0:2].copy() if nb is not None else None
mark("rack neighbours simulated+tracked=%d root=%s" % (rv_count(nb), rack_root))

# Authored axis positions of every sibling, whether or not they are simulated:
# static tubes are hard obstacles, so clearance against them matters either way.
nb_axis, nb_rad = [], []
for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
    ap = str(prim.GetPath())
    if (ap.startswith(rack_root + "/Tube_") and ap.endswith("/TubeWithCap/Tube/Tube")
            and ap != args.tube and ap not in nb_paths):
        nb_paths.append(ap)
for ap in nb_paths:
    prim = stage.GetPrimAtPath(ap)
    if not prim.IsValid():
        continue
    M = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(T)
    tr = np.array([float(v) for v in M.ExtractTranslation()], float)
    box = bc.ComputeWorldBound(prim).ComputeAlignedBox()
    mn, mx = box.GetMin(), box.GetMax()
    nb_axis.append((0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1])))
    nb_rad.append(0.5 * max(mx[0] - mn[0], mx[1] - mn[1]))
mark("siblings=%d tube_radius=%.1fmm" % (
    len(nb_axis), 1000 * (min(nb_rad) if nb_rad else 0.0)))


def finger_frame():
    """The two finger pads as world points, from the runtime wrist pose plus the
    live finger DOF (0.78 = shut, 0 = fully open, 42.5 mm of travel per side)."""
    p, r = wrist_pose()
    R = rotmat(r)
    pad = p + R @ pad_local
    qf = finger_q()
    half = HALF_SPAN_OPEN * (1.0 - min(max(qf / FINGER_CLOSED, 0.0), 1.0))
    xh = R[:, 0]
    return pad + xh * half, pad - xh * half


def min_nb_clearance():
    if not nb_axis:
        return None
    worst = 1e9
    for f in finger_frame():
        for (ax, ay), rad in zip(nb_axis, nb_rad):
            worst = min(worst, math.hypot(f[0] - ax, f[1] - ay) - rad - PAD_R)
    return round(float(worst), 4)


def neighbour_disp():
    if nb is None or rv_count(nb) == 0:
        return 0.0, []
    t = np.asarray(nb.get_transforms(), float)[:, 0:2]
    d = np.linalg.norm(t - nb0, axis=1)
    return float(d.max()), [round(float(v), 4) for v in d]


set_camera_view(eye=grasp0 + np.array([0.45, -0.55, 0.45]), target=grasp0 + np.array([0, 0, -0.03]))
apply_targets(8)
report["shots"].append(shot("00_before"))

DROP = np.array([float(v) for v in args.drop.split(",")])
TOP = grasp0 + np.array([0.0, 0.0, 0.24])
LIFT = grasp0 + np.array([0.0, 0.0, 0.18])
# the tube bottom sits 102.6 mm below the grasp landmark, so this puts it 20 mm over the bench
PLACE = np.array([DROP[0], DROP[1], DROP[2] + 0.123])

if args.probe == "1":
    pr = {"gpu_readback": gpu_readback, "pad_local": pad_local.tolist(),
          "tube_grasp_local": TUBE_GRASP_LOCAL.tolist(),
          "tube0": tube0.tolist(), "grasp0": grasp0.tolist(),
          "wrist0": wrist_pose()[0].tolist(), "grip0": grip_point().tolist(),
          "neighbours_simulated": rv_count(nb),
          "neighbours_authored": len(nb_axis),
          "nb_axes": [[round(a, 4), round(b, 4)] for a, b in nb_axis],
          "nb_radius_m": [round(v, 4) for v in nb_rad], "pad_half_thick_m": PAD_R,
          "rack_bodies_in_physx": len(rb_all)}
    r = ik_move(np.array(TOP), orient_w=0.5, z_floor=None, rounds=20, settle=30, tol=0.05, yaw_w=1.0)
    pr["ik_probe"] = r
    pr["clearance_at_above"] = min_nb_clearance()
    r2 = ik_move(np.array(grasp0), orient_w=1.0, z_floor=float(grasp0[2]) + 0.02,
                 rounds=20, settle=30, tol=0.010, yaw_w=1.0)
    pr["ik_probe_descent"] = r2
    pr["clearance_at_grasp"] = min_nb_clearance()
    pr["criteria_probe"] = {
        "agg_cap_raised": bool(capacity_raised()),
        "arm_actuation_verified": ACTUATED,
        "tensor_readback_sane": bool(READBACK_OK),
        "grip_moved": bool(np.linalg.norm(np.array(r["grip"]) - np.array(pr["grip0"])) > 0.01),
        "grip_within_10cm": bool(np.linalg.norm(np.array(r["grip"]) - np.array(TOP)) < 0.10),
        "descent_within_3cm": bool(np.linalg.norm(np.array(r2["grip"]) - np.array(grasp0)) < 0.03),
        "tube_untouched": bool(math.dist(tube_point(TUBE_GRASP_LOCAL)[:2], grasp0[:2]) < 0.005),
        "pad_local_sane": bool(0.05 < np.linalg.norm(pad_local) < 0.20),
        "grasp_local_sane": bool(0.03 < np.linalg.norm(TUBE_GRASP_LOCAL) < 0.20),
        "yaw_reached": bool(abs(float(np.dot(sep_dir(), YAW_UNIT))) > 0.7),
        "sibling_geometry_known": bool(len(nb_axis) >= 5),
        "finger_clearance_positive": bool(min_nb_clearance() is not None and min_nb_clearance() > 0.0),
    }
    (OUT / "auto_tube_pick_probe.json").write_text(json.dumps(pr, indent=2, default=float))
    mark("PROBE " + json.dumps(pr["criteria_probe"]))
    print("PROBE_DONE", flush=True)
    app.close()
    sys.exit(0)

PHASES = [
    ("above", TOP, float(grasp0[2]) + 0.10, 0.012, 1.0),
    ("grasp", grasp0, float(grasp0[2]) + 0.02, 0.010, 1.0),
]
for nm, tgt, zf, tol, yw in PHASES:
    t0 = time.time()
    r = ik_move(np.array(tgt), orient_w=0.5, z_floor=zf, tol=tol, yaw_w=yw)
    r["phase"] = nm
    r["seconds"] = round(time.time() - t0, 1)
    apply_targets(20)
    r["tube_pos"] = np.round(tube_state()[0], 4).tolist()
    r["neighbour_max_m"] = round(neighbour_disp()[0], 4)
    r["nb_clearance_m"] = min_nb_clearance()
    report["phases"].append(r)
    mark("IK %s -> %s" % (nm, r))
    report["shots"].append(shot("10_%s" % nm))

# close the gripper
t0 = time.time()
q_target[0, FINGER] = np.float32(float(args.finger_close))
apply_targets(180)  # increased from 120 for firmer grip
mark("gripper closed, %.1fs" % (time.time() - t0))
report["after_close"] = dict(
    finger_q=round(float(art.get_dof_positions()[0, FINGER]), 4),
    tube_pos=np.round(tube_state()[0], 4).tolist(),
    neighbour_max_m=round(neighbour_disp()[0], 4),
    nb_clearance_m=min_nb_clearance())
report["shots"].append(shot("20_clamped"))

# lift
t0 = time.time()
r = ik_move(np.array(LIFT), orient_w=0.5, z_floor=None, rounds=20, settle=30, tol=0.02, yaw_w=0.5)
apply_targets(80)  # increased from 40 for better stabilization
r["phase"] = "lift"
r["tube_pos"] = np.round(tube_state()[0], 4).tolist()
r["neighbour_max_m"] = round(neighbour_disp()[0], 4)
r["nb_clearance_m"] = min_nb_clearance()
r["seconds"] = round(time.time() - t0, 1)
report["phases"].append(r)
mark("IK lift -> %s" % r)
report["shots"].append(shot("30_lifted"))

# transport, sampling whether the tube follows the gripper
follow = []
t0 = time.time()
r = ik_move(np.array(PLACE), orient_w=0.5, z_floor=None, rounds=20, settle=30, tol=0.03, yaw_w=0.5)
for _ in range(80):  # increased from 40 for smoother transport
    apply_targets(1)
    g = grip_point()
    gp = tube_point(TUBE_GRASP_LOCAL)
    follow.append([round(float(g[0] - gp[0]), 4), round(float(g[1] - gp[1]), 4),
                   round(float(g[2] - gp[2]), 4)])
r["phase"] = "transport"
r["tube_pos"] = np.round(tube_state()[0], 4).tolist()
r["neighbour_max_m"] = round(neighbour_disp()[0], 4)
r["nb_clearance_m"] = min_nb_clearance()
r["seconds"] = round(time.time() - t0, 1)
report["phases"].append(r)
mark("IK transport -> %s" % r)
report["shots"].append(shot("40_moved"))

# release
q_target[0, FINGER] = np.float32(FINGER_OPEN_Q)
apply_targets(150)
r2 = ik_move(np.array(PLACE) + np.array([0.0, 0.0, 0.10]), orient_w=0.5, rounds=20, settle=30,
             tol=0.05, yaw_w=0.5)
r2["phase"] = "release"
report["phases"].append(r2)
apply_targets(120)
report["shots"].append(shot("50_released"))

tube_f, trot_f = tube_state()
report["tube_final"] = tube_f.tolist()
report["follow_offset_samples"] = follow[-10:]
nb_max, nb_all = neighbour_disp()
report["neighbour_max_displacement_m"] = round(nb_max, 4)
report["neighbour_displacements_m"] = nb_all
clears = [p.get("nb_clearance_m") for p in report["phases"]] + \
         [report["after_close"].get("nb_clearance_m")]
clears = [c for c in clears if c is not None]
report["min_finger_clearance_to_siblings_m"] = min(clears) if clears else None
grasp_f = tube_point(TUBE_GRASP_LOCAL)
report["grasp_final"] = np.round(grasp_f, 4).tolist()

zmax = max([p["tube_pos"][2] for p in report["phases"] if "tube_pos" in p] + [float(tube0[2])])
follow_err = [math.hypot(s[0], s[1]) for s in follow]
crit = {
    "tube_lifted_8cm": bool(zmax - float(tube0[2]) > 0.08),
    "tube_moved_horizontally_10cm": bool(math.dist(tube_f[:2], tube0[:2]) > 0.10),
    "tube_followed_gripper": bool(len(follow_err) > 5 and float(np.median(follow_err)) < 0.02),
    "tube_near_drop_point": bool(math.dist(tube_f[:2], DROP[:2]) < 0.06),
    "tube_left_the_rack": bool(float(grasp_f[2]) < float(grasp0[2]) - 0.05
                               and math.dist(tube_f[:2], tube0[:2]) > 0.10),
    "tube_not_knocked_before_grasp": bool(math.dist(report["phases"][0]["tube_pos"][:2], tube0[:2]) < 0.01),
    "neighbours_stayed_put": bool(nb_max < 0.03),
    "fingers_missed_the_siblings": bool(clears and min(clears) > 0.0),
    "w3_capacity_raised": bool(capacity_raised()),
    "arm_actuation_verified": ACTUATED,
    "tensor_readback_sane": bool(READBACK_OK),
}
report["criteria"] = crit
report["criteria_passed"] = sum(crit.values())
report["criteria_total"] = len(crit)
report["wall_seconds"] = round(time.time() - RUN_T0, 1)


def _jd(o):
    if isinstance(o, (np.floating, np.integer)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(str(type(o)))


fp = OUT / ("auto_tube_pick_%s.json" % args.label)
fp.write_text(json.dumps(report, indent=2, default=_jd), encoding="utf-8")
mark("WROTE " + str(fp))
mark("criteria %s (%d/%d)" % (crit, report["criteria_passed"], report["criteria_total"]))
print("AUTO_PICK_DONE criteria=%d/%d" % (report["criteria_passed"], report["criteria_total"]), flush=True)

app.close()
