"""探测这台机器上 Isaac Sim 5.1 提供的"休眠/唤醒"相关 API，为盖子压到上限位后
力矩完全无效（180 步 0 位移）这一现象做可判定实验做准备。

只列名字，不做任何物理：避免把"我以为有这个接口"当成事实。
"""
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs/runtime_validation/sleep_api_probe.json"
os.environ["TEMP"] = os.environ["TMP"] = str(ROOT / "outputs/runtime_validation/_probe_tmp")
os.makedirs(os.environ["TEMP"], exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True,
                     "extra_args": ["--portable-root", str(Path.home() / "CellCultureLabRuntime"),
                                    "--/app/settings/persistent=false",
                                    "--/renderer/multiGpu/enabled=false"]})
import omni.usd, omni.physx
import omni.physics.tensors as tensors
from pxr import UsdPhysics, Gf, PhysxSchema

res = {"scope": "sleep-related API inventory for this Isaac Sim 5.1 build"}


def names(obj):
    return sorted(n for n in dir(obj) if not n.startswith("_"))


res["PhysxSchema_modules"] = [m for m in names(PhysxSchema) if "leep" in m or "RigidBody" in m or "Articulation" in m]
for mod in ("PhysxRigidBodyAPI", "PhysxSleepAPI", "PhysxArticulationAPI"):
    cls = getattr(PhysxSchema, mod, None)
    if cls is None:
        res[mod] = "MISSING"
        continue
    try:
        attrs = sorted(cls.GetAttributeNames()) if hasattr(cls, "GetAttributeNames") else None
    except Exception as exc:
        attrs = "GetAttributeNames failed: %s" % exc
    res[mod] = {"attrs": attrs,
                "sleep_like_methods": [m for m in names(cls) if "leep" in m or "Wake" in m]}

# 运行时视图上的唤醒/休眠接口
res["ArticulationView_methods"] = None
res["RigidBodyView_methods"] = None
res["SimulationView_methods_sleep"] = None
try:
    ctx = omni.usd.get_context()
    iface = omni.physx.get_physx_interface()
    tensors.reset()
    ctx.open_stage(str(ROOT / "assets/scenes/mini_closed_loop_v2.usda"))
    stage = ctx.get_stage()
    ps = UsdPhysics.Scene(stage.GetPrimAtPath("/physicsScene"))
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)
    api = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
    api.CreateEnableGPUDynamicsAttr(False)
    api.CreateTimeStepsPerSecondAttr(240)
    iface.start_simulation()
    iface.update_simulation(1 / 240, 0)
    view = tensors.create_simulation_view("numpy")
    res["SimulationView_methods_sleep"] = [m for m in names(view) if "leep" in m or "wake" in m.lower()]
    art = view.create_articulation_view("/Mini/Joints/world_fixed")
    res["ArticulationView_methods"] = [m for m in names(art) if "leep" in m or "wake" in m.lower()
                                       or "notify" in m.lower() or "set_velocit" in m]
    rb = view.create_rigid_body_view("/Mini/Bodies/*")
    res["RigidBodyView_methods"] = [m for m in names(rb) if "leep" in m or "wake" in m.lower()]
    res["physx_interface_sleep_methods"] = [m for m in names(iface) if "leep" in m or "wake" in m.lower()]
    # 资产上现存的休眠相关 authored 属性
    res["authored_sleep_attrs"] = {str(p.GetPath()): {a.GetName(): a.Get() for a in p.GetAttributes()
                                                      if "leep" in a.GetName()}
                                   for p in stage.Traverse() if any("leep" in a.GetName() for a in p.GetAttributes())}
    view.invalidate()
    iface.reset_simulation()
except Exception as exc:
    res["runtime_probe_error"] = "%s: %s" % (type(exc).__name__, exc)

OUT.write_text(json.dumps(res, indent=1, ensure_ascii=False, default=str), encoding="utf-8")
print("PROBE_WRITTEN", OUT, flush=True)
app.close()
